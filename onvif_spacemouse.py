#!/usr/bin/env python3
"""Control ONVIF PTZ cameras with a 3Dconnexion SpaceMouse.

Threads:
  SpaceMouse    - reads evdev events, tracks axis state, queues button events
  Pedal         - optional keyboard-type foot pedal
  NxWitness     - optional; cycles Nx Witness layouts by sending Generic Events
  CameraWorker  - owns all (blocking) ONVIF calls so input handling never stalls
  main          - fixed-rate loop: turns axis state into PTZ velocities and
                  button presses into actions
"""

import argparse
import logging
import queue
import select
import signal
import sys
import threading
import time
from datetime import timedelta

import evdev
import requests
import urllib3
import yaml
from evdev import ecodes
from onvif import ONVIFCamera
from zeep.transports import Transport

log = logging.getLogger("onvif-spacemouse")

AXIS_CODES = {
    "x": ecodes.REL_X, "y": ecodes.REL_Y, "z": ecodes.REL_Z,
    "rx": ecodes.REL_RX, "ry": ecodes.REL_RY, "rz": ecodes.REL_RZ,
}
BUTTON_CODES = {"left": ecodes.BTN_0, "right": ecodes.BTN_1}

DEFAULTS = {
    "device": {"name": "SpaceMouse", "path": None, "full_scale": 350,
               "stale_timeout": 1.0, "led_feedback": True},
    # Each output takes one or more sources; the strongest source wins.
    # Device frame: x right, y toward you, z down; rx/ry = rock cap, rz = twist.
    "axes": {
        "pan": [{"axis": "x", "invert": False}, {"axis": "ry", "invert": True}],
        "tilt": [{"axis": "y", "invert": True}, {"axis": "rx", "invert": True}],
        "zoom": [{"axis": "rz", "invert": False}],
    },
    "control": {
        "deadzone": 0.12,          # fraction of full scale ignored around centre
        "curve": 2.0,              # 1 = linear, >1 = finer control near centre
        "max_speed": 1.0,          # scales ONVIF velocity (-1..1)
        "separate_zoom": True,     # dominant motion wins: zoom OR pan/tilt
        "axis_lock": 0.5,          # drop pan or tilt if weaker than this x the other (0 = off)
        "step": 0.05,              # velocity quantisation, limits command spam
        "min_interval": 0.1,       # min seconds between ContinuousMove commands
        "move_timeout": 1.0,       # ONVIF Timeout; camera self-stops if we die
        "long_press": 0.8,
        "save_hold": 2.0,
        "tick": 0.02,
    },
    "buttons": {
        "left": {"short": "prev_preset", "long": "none"},
        "right": {"short": "next_preset", "long": "save_preset"},
    },
    # Keyboard-type foot pedal. keys: evdev key name -> preset name or button action.
    "pedal": {"name": None, "path": None, "keys": {},
              "long_press_save": True},   # hold (control.save_hold) saves to that pedal's preset
    # Nx Witness: each layout step sends a Generic Event (source, caption = layout name);
    # an Nx event rule per layout with the "Open Layout" action does the switching.
    "nx": {"url": None, "user": None, "password": None, "verify_tls": False,
           "source": "SpaceMouse", "layouts": []},
    # Burns the preset name into the camera's video (all streams of that video source).
    "overlay": {
        "enabled": False,
        "seconds": 10,
        "position": "UpperLeft",   # UpperLeft UpperRight LowerLeft LowerRight Center, or [x, y] in -1..1
        "font_size": None,         # camera default if unset
        # After `seconds`, park the last preset's name here instead of removing it (None = remove).
        "rest_position": None,
        "rest_font_size": None,    # None = same as font_size
    },
}


def merge(base, override):
    out = dict(base)
    for k, v in (override or {}).items():
        out[k] = merge(base[k], v) if isinstance(base.get(k), dict) and isinstance(v, dict) else v
    return out


def load_config(path):
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    cfg = merge(DEFAULTS, raw)
    if not cfg.get("cameras"):
        raise SystemExit(f"{path}: no cameras defined")
    for cam in cfg["cameras"]:
        cam.setdefault("name", cam["host"])
        cam.setdefault("port", 80)
        cam.setdefault("zoom", True)
        cam.setdefault("presets", ["Preset 1", "Preset 2", "Preset 3", "Preset 4"])
        cam.setdefault("locked", [])
        cam.setdefault("home", None)
        cam.setdefault("home_after", 120)
        for name in cam["locked"] + [cam["home"]] * bool(cam["home"]):
            if name not in cam["presets"]:
                log.warning("[%s] preset '%s' is not in presets", cam["name"], name)
    nx = cfg["nx"]
    if nx["url"]:
        nx["url"] = nx["url"].rstrip("/")
        if not nx["layouts"]:
            log.warning("nx: url set but no layouts listed")
    pedal = cfg["pedal"]
    pedal["commands"] = {}
    for key, target in (pedal["keys"] or {}).items():
        code = key_code(key)
        if code is None:
            raise SystemExit(f"{path}: pedal key '{key}' is not a known key name (e.g. KEY_1, KEY_F13)")
        if target in ACTIONS:
            pedal["commands"][code] = ACTIONS[target]
        else:
            if not any(target in c["presets"] for c in cfg["cameras"]):
                log.warning("pedal key %s -> '%s' is not a preset of any camera", key, target)
            pedal["commands"][code] = ("goto", target)
    return cfg


def key_code(name):
    """'KEY_F13', 'f13', 'a' or 1 -> evdev key code."""
    name = str(name).upper()
    if not name.startswith(("KEY_", "BTN_")):
        name = "KEY_" + name
    code = ecodes.ecodes.get(name)
    return code if isinstance(code, int) else None


# --------------------------------------------------------------------------- input

class SpaceMouse(threading.Thread):
    """Reads the SpaceMouse, surviving unplug/replug."""

    def __init__(self, cfg, stop):
        super().__init__(name="spacemouse", daemon=True)
        self.cfg = cfg
        self.stop = stop
        self.buttons = queue.Queue()
        self.lock = threading.Lock()
        self.values = {code: 0 for code in AXIS_CODES.values()}
        self.last_axis_event = 0.0
        self.dev = None
        self.led_lock = threading.Lock()

    def find(self):
        if self.cfg["path"]:
            return evdev.InputDevice(self.cfg["path"])
        for p in evdev.list_devices():
            d = evdev.InputDevice(p)
            caps = d.capabilities().get(ecodes.EV_REL, [])
            if self.cfg["name"].lower() in d.name.lower() and ecodes.REL_RZ in caps:
                return d
            d.close()
        return None

    def axes(self):
        """Axis values normalised to -1..1, zeroed if the device went quiet."""
        with self.lock:
            if time.monotonic() - self.last_axis_event > self.cfg["stale_timeout"]:
                return {name: 0.0 for name in AXIS_CODES}
            fs = float(self.cfg["full_scale"])
            return {name: max(-1.0, min(1.0, self.values[code] / fs))
                    for name, code in AXIS_CODES.items()}

    def blink(self, count=1, on=0.12, off=0.12):
        if not self.cfg["led_feedback"]:
            return

        def run():
            dev = self.dev
            with self.led_lock:   # never interleave two blink sequences
                try:
                    for _ in range(count):
                        dev.set_led(ecodes.LED_MISC, 0)
                        time.sleep(off)
                        dev.set_led(ecodes.LED_MISC, 1)
                        time.sleep(on)
                except Exception:
                    pass
        if self.dev:
            threading.Thread(target=run, daemon=True).start()

    def run(self):
        while not self.stop.is_set():
            try:
                self.dev = self.find()
            except OSError as e:
                log.warning("SpaceMouse open failed: %s", e)
                self.dev = None
            if not self.dev:
                self.stop.wait(2)
                continue
            log.info("SpaceMouse connected: %s (%s)", self.dev.name, self.dev.path)
            try:
                self.read_loop()
            except OSError as e:
                log.warning("SpaceMouse lost: %s", e)
            finally:
                with self.lock:
                    self.values = {c: 0 for c in self.values}
                try:
                    self.dev.close()
                except OSError:
                    pass
                self.dev = None

    def read_loop(self):
        while not self.stop.is_set():
            r, _, _ = select.select([self.dev.fd], [], [], 0.5)
            if not r:
                continue
            for ev in self.dev.read():
                if ev.type == ecodes.EV_REL and ev.code in self.values:
                    # SpaceMouse reports absolute displacement via REL events
                    with self.lock:
                        self.values[ev.code] = ev.value
                        self.last_axis_event = time.monotonic()
                elif ev.type == ecodes.EV_KEY and ev.value in (0, 1):
                    self.buttons.put((ev.code, bool(ev.value), time.monotonic()))


class Pedal(threading.Thread):
    """Keyboard-type foot pedal, grabbed exclusively so presses don't type anywhere else."""

    def __init__(self, cfg, save_hold, stop, out):
        super().__init__(name="pedal", daemon=True)
        self.cfg = cfg
        self.save_hold = save_hold
        self.stop = stop
        self.out = out
        self.dev = None

    def find(self):
        if self.cfg["path"]:
            return evdev.InputDevice(self.cfg["path"])
        found = []
        for p in evdev.list_devices():
            d = evdev.InputDevice(p)
            keys = set(d.capabilities().get(ecodes.EV_KEY, []))
            if self.cfg["name"].lower() in d.name.lower() and keys:
                found.append((bool(keys & set(self.cfg["commands"])), d))
            else:
                d.close()
        # pedals often expose several interfaces; prefer the one that can send our keys
        found.sort(key=lambda f: f[0], reverse=True)
        for _, d in found[1:]:
            d.close()
        return found[0][1] if found else None

    def run(self):
        missing_logged = False
        while not self.stop.is_set():
            try:
                self.dev = self.find()
            except OSError as e:
                log.warning("pedal open failed: %s", e)
                self.dev = None
            if not self.dev:
                if not missing_logged:
                    log.warning("pedal '%s' not found, waiting (see --list-devices)",
                                self.cfg["path"] or self.cfg["name"])
                    missing_logged = True
                self.stop.wait(2)
                continue
            missing_logged = False
            log.info("pedal connected: %s (%s)", self.dev.name, self.dev.path)
            try:
                try:
                    self.dev.grab()
                except OSError as e:
                    log.warning("pedal grab failed (presses may also type elsewhere): %s", e)
                self.read_loop()
            except OSError as e:
                log.warning("pedal lost: %s", e)
            finally:
                try:
                    self.dev.close()
                except OSError:
                    pass
                self.dev = None

    def read_loop(self):
        held = {}   # key code -> [press time, save already fired]
        try:
            while not self.stop.is_set():
                r, _, _ = select.select([self.dev.fd], [], [], 0.05 if held else 0.5)
                for ev in self.dev.read() if r else []:
                    if ev.type != ecodes.EV_KEY or ev.value == 2:   # ignore autorepeat
                        continue
                    command = self.cfg["commands"].get(ev.code)
                    if command is None:
                        if ev.value == 1:
                            log.info("pedal key %s not mapped", ecodes.KEY.get(ev.code, ev.code))
                        continue
                    if not self.can_save(command):
                        if ev.value == 1:
                            self.send(ev.code, command)
                    elif ev.value == 1:
                        held[ev.code] = [time.monotonic(), False]
                    elif ev.code in held and not held.pop(ev.code)[1]:
                        self.send(ev.code, command)   # tap: recall on release
                now = time.monotonic()
                for code, state in held.items():
                    if not state[1] and now - state[0] >= self.save_hold:
                        state[1] = True
                        self.send(code, ("save_to", self.cfg["commands"][code][1]))
        finally:
            held.clear()

    def can_save(self, command):
        return self.cfg["long_press_save"] and command[0] == "goto"

    def send(self, code, command):
        log.debug("pedal %s -> %s", ecodes.KEY.get(code, code), command)
        self.out.put(command)


# --------------------------------------------------------------------------- nx witness

class NxWitness(threading.Thread):
    """Cycles through cfg["layouts"], announcing each as an Nx Witness Generic Event."""

    def __init__(self, cfg, stop, on_event):
        super().__init__(name="nx", daemon=True)
        self.cfg = cfg
        self.stop = stop
        self.on_event = on_event
        self.queue = queue.Queue()
        self.index = -1
        self.http = requests.Session()
        self.http.verify = cfg["verify_tls"]
        if not cfg["verify_tls"]:   # Nx servers usually run a self-signed certificate
            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
        self.token = None
        self.digest = False      # pre-5.0 servers have no session login
        self.legacy_api = False  # pre-6.0 servers only have /api/createEvent

    def run(self):
        while not self.stop.is_set():
            try:
                step = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            layouts = self.cfg["layouts"]
            if self.index < 0:   # first press: next -> first layout, prev -> last
                self.index = 0 if step > 0 else len(layouts) - 1
            else:
                self.index = (self.index + step) % len(layouts)
            name = layouts[self.index]
            try:
                self.send_event(name)
                log.info("nx layout %d '%s'", self.index + 1, name)
                self.on_event("layout", self.index)
            except Exception as e:
                log.error("nx layout '%s' failed: %s", name, e)

    def login(self):
        r = self.http.post(self.cfg["url"] + "/rest/v1/login/sessions", timeout=5,
                           json={"username": self.cfg["user"], "password": self.cfg["password"]})
        if r.status_code == 404:
            self.digest = self.legacy_api = True
            self.http.auth = requests.auth.HTTPDigestAuth(self.cfg["user"], self.cfg["password"])
            return
        r.raise_for_status()
        self.token = r.json()["token"]
        self.http.headers["Authorization"] = "Bearer " + self.token

    def send_event(self, caption):
        event = {"source": self.cfg["source"], "caption": caption}
        for attempt in (1, 2):
            if not self.token and not self.digest:
                self.login()
            if self.legacy_api:
                r = self.http.get(self.cfg["url"] + "/api/createEvent", params=event, timeout=5)
            else:   # Nx 6+: the /api/ endpoints are disabled by default (403)
                r = self.http.post(self.cfg["url"] + "/rest/v4/events/generic",
                                   json=dict(event, state="instant"), timeout=5)
                if r.status_code == 404:   # pre-6.0 server
                    self.legacy_api = True
                    return self.send_event(caption)
            if r.status_code == 401 and attempt == 1 and not self.digest:
                self.token = None   # session expired: log in again
                continue
            if not r.ok:
                raise RuntimeError(f"HTTP {r.status_code}: {r.text[:200]}")
            return


class Commands:
    """Routes actions: layout steps to Nx Witness, everything else to the camera worker."""

    def __init__(self, worker, nx):
        self.worker = worker
        self.nx = nx

    def put(self, command):
        kind, arg = command
        if kind != "layout":
            self.worker.queue.put(command)
        elif self.nx:
            self.nx.queue.put(arg)
        else:
            log.warning("layout action used but nx: is not configured")


# --------------------------------------------------------------------------- cameras

def osd_position(pos):
    """Corner name, "Center", or [x, y] with -1..1 from left/bottom to right/top."""
    if pos == "Center":
        pos = [0, 0]
    if isinstance(pos, (list, tuple)):
        return {"Type": "Custom", "Pos": {"x": float(pos[0]), "y": float(pos[1])}}
    return {"Type": pos}


class Camera:
    def __init__(self, cfg):
        self.cfg = cfg
        self.name = cfg["name"]
        self.ptz = None
        self.token = None
        self.preset_index = -1   # last recalled preset slot
        self.media = None
        self.source = None       # video source config the overlay is drawn on
        self.osd_token = None
        self.osd_until = 0.0     # monotonic time the big message ends; 0 = none showing
        self.label = None        # preset the camera was last sent to, shown when parked
        self.last_activity = time.monotonic()   # operator input; drives return-to-home
        self.at_home = False

    def connect(self):
        c = self.cfg
        log.info("[%s] connecting to %s:%s", self.name, c["host"], c["port"])
        cam = ONVIFCamera(c["host"], c["port"], c["user"], c["password"],
                          adjust_time=c.get("adjust_time", True),
                          transport=Transport(timeout=5, operation_timeout=5))
        media = cam.create_media_service()
        ptz = cam.create_ptz_service()
        profiles = media.GetProfiles()
        want = c.get("profile")
        profile = next((p for p in profiles if want in (p.token, p.Name)), None) if want else \
            next((p for p in profiles if getattr(p, "PTZConfiguration", None)), None)
        if profile is None:
            raise RuntimeError(f"no PTZ-capable media profile (have: {[p.token for p in profiles]})")
        self.ptz, self.token = ptz, profile.token
        self.media = media
        vsc = getattr(profile, "VideoSourceConfiguration", None)
        self.source = vsc.token if vsc else None
        log.info("[%s] connected, profile %s", self.name, self.token)

    def move(self, pan, tilt, zoom, timeout):
        vel = {"PanTilt": {"x": pan, "y": tilt}}
        if self.cfg["zoom"]:
            vel["Zoom"] = {"x": zoom}
        req = {"ProfileToken": self.token, "Velocity": vel}
        if timeout:
            req["Timeout"] = timedelta(seconds=timeout)
        self.ptz.ContinuousMove(req)

    def halt(self):
        self.ptz.Stop({"ProfileToken": self.token, "PanTilt": True, "Zoom": self.cfg["zoom"]})

    def presets(self):
        return {p.Name: p.token for p in (self.ptz.GetPresets({"ProfileToken": self.token}) or [])}

    def goto_preset(self, name):
        token = self.presets().get(name)
        if token is None:
            log.warning("[%s] preset '%s' not saved yet", self.name, name)
            return False
        self.ptz.GotoPreset({"ProfileToken": self.token, "PresetToken": token})
        return True

    def save_preset(self, name):
        req = {"ProfileToken": self.token, "PresetName": name}
        token = self.presets().get(name)
        if token is not None:
            req["PresetToken"] = token   # overwrite existing preset
        self.ptz.SetPreset(req)

    def overlay_texts(self):
        names = self.cfg["presets"]
        return (set(names) | {f"{n} (empty)" for n in names} | {f"Saved: {n}" for n in names}
                | {f"{n} (locked)" for n in names})

    def clear_stale_overlays(self):
        """Remove overlays a previous run left behind (e.g. killed mid-display)."""
        self.osd_token, self.osd_until, self.label = None, 0.0, None
        if not self.source:
            return
        ours = self.overlay_texts()
        for osd in self.media.GetOSDs({"ConfigurationToken": self.source}) or []:
            text = getattr(getattr(osd, "TextString", None), "PlainText", None)
            if text in ours:
                self.media.DeleteOSD({"OSDToken": osd.token})
                log.info("[%s] removed leftover overlay '%s'", self.name, text)

    def show_text(self, text, position, font_size):
        text_cfg = {"Type": "Plain", "PlainText": text}
        if font_size:
            text_cfg["FontSize"] = font_size
        osd = {"token": self.osd_token or "", "VideoSourceConfigurationToken": {"_value_1": self.source},
               "Type": "Text", "Position": osd_position(position), "TextString": text_cfg}
        try:
            if self.osd_token:
                self.media.SetOSD({"OSD": osd})
                return
        except Exception as e:   # e.g. camera rebooted and forgot it: create a fresh one
            log.debug("[%s] SetOSD failed (%s), recreating overlay", self.name, e)
            osd["token"] = ""
        resp = self.media.CreateOSD({"OSD": osd})
        self.osd_token = getattr(resp, "OSDToken", resp)   # response may carry extra elements

    def hide_text(self):
        if self.osd_token:
            self.media.DeleteOSD({"OSDToken": self.osd_token})
        self.osd_token = None


class CameraWorker(threading.Thread):
    """Serialises all ONVIF traffic. Velocity updates are coalesced (latest wins)."""

    def __init__(self, cameras, control, overlay, stop, on_event):
        super().__init__(name="camera", daemon=True)
        self.cameras = cameras
        self.control = control
        self.overlay = overlay
        self.stop = stop
        self.on_event = on_event
        self.queue = queue.Queue()
        self.active = 0
        self.velocity = (0.0, 0.0, 0.0)
        self.sent = (0.0, 0.0, 0.0)
        self.last_send = 0.0
        self.retry_at = 0.0

    def camera(self):
        return self.cameras[self.active]

    def ensure_connected(self):
        cam = self.camera()
        if cam.ptz is not None:
            return True
        if time.monotonic() < self.retry_at:
            return False
        try:
            cam.connect()
            self.sent = (0.0, 0.0, 0.0)
        except Exception as e:
            log.error("[%s] connect failed: %s", cam.name, e)
            self.retry_at = time.monotonic() + 5
            return False
        if self.overlay["enabled"]:
            try:
                cam.clear_stale_overlays()
            except Exception as e:
                log.warning("[%s] overlay cleanup failed: %s", cam.name, e)
        return True

    def show_overlay(self, cam, text, label=None):
        """Show text large for `seconds`; `label` becomes what stays parked afterwards."""
        if not self.overlay["enabled"]:
            return
        if label is not None:
            cam.label = label
        o = self.overlay
        try:
            cam.show_text(text, o["position"], o["font_size"])
            cam.osd_until = time.monotonic() + o["seconds"]
        except Exception as e:
            log.warning("[%s] overlay failed: %s", cam.name, e)
            cam.osd_token, cam.osd_until = None, 0.0

    def expire_overlays(self):
        now = time.monotonic()
        o = self.overlay
        for cam in self.cameras:
            if not cam.osd_until or now < cam.osd_until:
                continue
            try:
                if o["rest_position"] and cam.label:
                    cam.show_text(cam.label, o["rest_position"], o["rest_font_size"] or o["font_size"])
                else:
                    cam.hide_text()
                cam.osd_until = 0.0
            except Exception as e:
                log.warning("[%s] overlay update failed: %s", cam.name, e)
                cam.osd_until = now + 5   # retry

    def remove_overlays(self):
        for cam in self.cameras:
            try:
                cam.hide_text()
            except Exception as e:
                log.warning("[%s] overlay removal failed: %s", cam.name, e)

    def run(self):
        while not self.stop.is_set():
            items = []
            try:
                items.append(self.queue.get(timeout=0.1))
                while True:
                    items.append(self.queue.get_nowait())
            except queue.Empty:
                pass
            if self.stop.is_set():
                break
            for kind, arg in items:
                if kind == "velocity":
                    self.velocity = arg
                else:
                    self.command(kind, arg)
            self.update_motion()
            self.return_home()
            self.expire_overlays()
        self.halt_active()
        self.remove_overlays()   # a parked label would go stale once nothing updates it

    def update_motion(self):
        if not self.ensure_connected():
            return
        now = time.monotonic()
        moving = any(self.velocity)
        if moving:
            self.camera().last_activity, self.camera().at_home = now, False
        timeout = self.control["move_timeout"]
        keepalive = moving and timeout and now - self.last_send > timeout * 0.5
        if self.velocity == self.sent and not keepalive:
            return
        if moving and now - self.last_send < self.control["min_interval"]:
            return
        cam = self.camera()
        try:
            if moving:
                cam.move(*self.velocity, timeout)
            else:
                cam.halt()
            self.sent, self.last_send = self.velocity, now
        except Exception as e:
            log.error("[%s] PTZ command failed: %s", cam.name, e)
            cam.ptz = None   # force reconnect
            self.retry_at = now + 2

    def halt_active(self):
        cam = self.camera()
        if cam.ptz is not None and any(self.sent):
            try:
                cam.halt()
            except Exception as e:
                log.warning("[%s] stop failed: %s", cam.name, e)
        self.sent = (0.0, 0.0, 0.0)

    def command(self, kind, arg):
        if kind == "switch":
            self.halt_active()
            self.velocity = (0.0, 0.0, 0.0)
            self.active = (self.active + arg) % len(self.cameras)
            self.retry_at = 0.0
            log.info("active camera -> %d: %s", self.active + 1, self.camera().name)
            if arg:   # not the initial selection at startup
                self.on_event("camera", self.active)
            self.ensure_connected()
            return
        if not self.ensure_connected():
            return
        cam = self.camera()
        names = cam.cfg["presets"]
        if not names:
            log.warning("[%s] no presets configured", cam.name)
            return
        cam.last_activity = time.monotonic()
        try:
            if kind == "goto":
                if arg not in names:
                    log.warning("[%s] no preset '%s'", cam.name, arg)
                    return
                self.halt_active()
                self.recall(cam, names.index(arg))
                self.on_event("preset", cam.preset_index)
            elif kind == "preset":
                self.halt_active()
                if cam.preset_index < 0:   # first press: right -> first preset, left -> last
                    index = 0 if arg > 0 else len(names) - 1
                else:
                    index = (cam.preset_index + arg) % len(names)
                self.recall(cam, index)
                # blink even if unsaved: cycling to an empty slot is how you pick where to save
                self.on_event("preset", cam.preset_index)
            elif kind in ("save", "save_to"):
                if kind == "save_to":
                    if arg not in names:
                        log.warning("[%s] no preset '%s'", cam.name, arg)
                        return
                    cam.preset_index = names.index(arg)
                cam.preset_index = max(cam.preset_index, 0)
                name = names[cam.preset_index]
                if name in cam.cfg["locked"]:
                    log.warning("[%s] preset %d '%s' is locked, not saving", cam.name, cam.preset_index + 1, name)
                    self.show_overlay(cam, f"{name} (locked)")
                    self.on_event("locked", cam.preset_index)
                    return
                cam.save_preset(name)
                log.info("[%s] saved preset %d '%s'", cam.name, cam.preset_index + 1, name)
                cam.at_home = name == cam.cfg["home"]
                self.show_overlay(cam, f"Saved: {name}", label=name)
                self.on_event("saved", cam.preset_index)
        except Exception as e:
            log.error("[%s] %s failed: %s", cam.name, kind, e)
            cam.ptz = None

    def recall(self, cam, index):
        cam.preset_index = index
        name = cam.cfg["presets"][index]
        if cam.goto_preset(name):
            log.info("[%s] recalled preset %d '%s'", cam.name, index + 1, name)
            cam.at_home = name == cam.cfg["home"]
            self.show_overlay(cam, name, label=name)
            return True
        self.show_overlay(cam, f"{name} (empty)")
        return False

    def return_home(self):
        """Send idle cameras back to their home preset, once per idle period."""
        now = time.monotonic()
        for cam in self.cameras:
            home = cam.cfg["home"]
            if not home or cam.at_home or cam.ptz is None or now - cam.last_activity < cam.cfg["home_after"]:
                continue
            cam.at_home = True   # set first: an unsaved or failing home must not retry every tick
            if home not in cam.cfg["presets"]:
                continue
            log.info("[%s] idle for %ss, returning home", cam.name, cam.cfg["home_after"])
            try:
                if cam is self.camera():
                    self.halt_active()
                self.recall(cam, cam.cfg["presets"].index(home))
            except Exception as e:
                log.error("[%s] return home failed: %s", cam.name, e)
                cam.ptz = None


# --------------------------------------------------------------------------- control loop

def shape(v, deadzone, curve):
    mag = abs(v)
    if mag <= deadzone:
        return 0.0
    mag = ((mag - deadzone) / (1.0 - deadzone)) ** curve
    return mag if v > 0 else -mag


def compute_velocity(axes, cfg):
    ctl = cfg["control"]
    out = {}
    for name in ("pan", "tilt", "zoom"):
        sources = cfg["axes"][name]
        if isinstance(sources, dict):
            sources = [sources]
        raw = [axes[s["axis"]] * s.get("scale", 1.0) * (-1 if s.get("invert") else 1)
               for s in sources]
        v = max(raw, key=abs, default=0.0)
        out[name] = shape(max(-1.0, min(1.0, v)), ctl["deadzone"], ctl["curve"]) * ctl["max_speed"]
    if ctl["separate_zoom"]:
        if abs(out["zoom"]) > max(abs(out["pan"]), abs(out["tilt"])):
            out["pan"] = out["tilt"] = 0.0
        else:
            out["zoom"] = 0.0
    lock = ctl["axis_lock"]
    if lock:
        if abs(out["tilt"]) < lock * abs(out["pan"]):
            out["tilt"] = 0.0
        elif abs(out["pan"]) < lock * abs(out["tilt"]):
            out["pan"] = 0.0
    step = ctl["step"]
    return tuple(round(round(out[k] / step) * step, 3) + 0.0 for k in ("pan", "tilt", "zoom"))


ACTIONS = {
    "prev_camera": ("switch", -1),
    "next_camera": ("switch", 1),
    "next_preset": ("preset", 1),
    "prev_preset": ("preset", -1),
    "save_preset": ("save", None),
    "next_layout": ("layout", 1),
    "prev_layout": ("layout", -1),
}


def run_service(cfg):
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    mouse = SpaceMouse(cfg["device"], stop)

    def on_event(kind, index):
        if kind == "saved":
            mouse.blink(1, on=0.1, off=0.8)
        elif kind == "locked":
            mouse.blink(6, on=0.05, off=0.05)
        elif kind == "layout":
            mouse.blink(index + 1, on=0.3, off=0.15)   # slower blinks than presets
        else:
            mouse.blink(index + 1)

    worker = CameraWorker([Camera(c) for c in cfg["cameras"]], cfg["control"], cfg["overlay"],
                          stop, on_event)
    mouse.start()
    worker.start()
    worker.queue.put(("switch", 0))   # connect to first camera
    nx = None
    if cfg["nx"]["url"] and cfg["nx"]["layouts"]:
        nx = NxWitness(cfg["nx"], stop, on_event)
        nx.start()
    commands = Commands(worker, nx)
    if cfg["pedal"]["name"] or cfg["pedal"]["path"]:
        Pedal(cfg["pedal"], cfg["control"]["save_hold"], stop, commands).start()

    ctl = cfg["control"]
    code_to_button = {v: k for k, v in BUTTON_CODES.items()}
    pressed = {}   # button -> [press time, long action already fired]
    last_vel = None

    while not stop.is_set():
        now = time.monotonic()
        while True:
            try:
                code, down, t = mouse.buttons.get_nowait()
            except queue.Empty:
                break
            button = code_to_button.get(code)
            if button is None:
                continue
            if down:
                pressed[button] = [t, False]
            elif button in pressed:
                t0, fired = pressed.pop(button)
                if not fired:
                    dispatch(cfg, commands, button, "short")

        for button, state in pressed.items():
            action = cfg["buttons"][button].get("long")
            hold = ctl["save_hold"] if action == "save_preset" else ctl["long_press"]
            if not state[1] and now - state[0] >= hold:
                state[1] = True
                dispatch(cfg, commands, button, "long")

        vel = compute_velocity(mouse.axes(), cfg)
        if vel != last_vel:
            worker.queue.put(("velocity", vel))
            last_vel = vel
        stop.wait(ctl["tick"])

    log.info("shutting down")
    worker.join(timeout=6)


def dispatch(cfg, commands, button, press):
    action = cfg["buttons"][button].get(press)
    if not action or action == "none":
        return
    if action not in ACTIONS:
        log.warning("unknown action '%s' for %s/%s", action, button, press)
        return
    log.debug("%s %s press -> %s", button, press, action)
    commands.put(ACTIONS[action])


def test_input(cfg):
    """Print live axis/velocity/button state to help tune the mapping."""
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    mouse = SpaceMouse(cfg["device"], stop)
    mouse.start()
    print("Move the SpaceMouse / press buttons. Ctrl+C to quit.")
    while not stop.is_set():
        axes = mouse.axes()
        pan, tilt, zoom = compute_velocity(axes, cfg)
        raw = " ".join(f"{k}:{v:+.2f}" for k, v in axes.items())
        sys.stdout.write(f"\r{raw} | pan {pan:+.2f} tilt {tilt:+.2f} zoom {zoom:+.2f}   ")
        sys.stdout.flush()
        try:
            code, down, _ = mouse.buttons.get_nowait()
            print(f"\nbutton {ecodes.BTN.get(code, code)} {'down' if down else 'up'}")
        except queue.Empty:
            pass
        stop.wait(0.05)
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-c", "--config", default="/etc/onvif-spacemouse/config.yaml")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--test-input", action="store_true",
                    help="show SpaceMouse values and resulting velocities, no cameras")
    ap.add_argument("--list-devices", action="store_true",
                    help="list input devices (to find the pedal's name)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", force=True)
    for noisy in ("zeep", "urllib3", "httpx"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    if args.list_devices:
        for p in evdev.list_devices():
            d = evdev.InputDevice(p)
            kind = "keys" if d.capabilities().get(ecodes.EV_KEY) else "no keys"
            print(f"{d.path:20} {d.name!r:45} ({kind})")
    elif args.test_input:
        try:
            cfg = load_config(args.config)
        except (OSError, SystemExit):
            cfg = merge(DEFAULTS, {})
        test_input(cfg)
    else:
        run_service(load_config(args.config))


if __name__ == "__main__":
    main()
