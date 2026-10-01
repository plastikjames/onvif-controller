# onvif-spacemouse

Drive ONVIF PTZ cameras with a 3Dconnexion SpaceMouse (tested with the Compact) on Linux.
Reads the SpaceMouse straight from evdev, so no spacenavd or 3Dconnexion driver is needed.

## Controls (defaults)

| Input | Action |
|---|---|
| Slide cap left/right | Pan |
| Slide cap forward/back | Tilt |
| Twist cap | Zoom |
| Left button, tap | Previous preset (cycles through the camera's `presets` list) |
| Right button, tap | Next preset |
| Right button, hold 2 s | Save current position to the current preset slot |

The LED blinks N times for preset N, and once slowly after a save.
With `overlay.enabled: true`, the camera also shows the preset name on its video for `overlay.seconds`
("name (empty)" for an unsaved slot, "Saved: name" after a save). The text is burned into every
stream from that camera, recordings and program output included.
Set `overlay.rest_position` to keep the last preset's name in a corner after that instead of removing it;
the name is removed when the service stops.
To save to slot 3: tap until you land on slot 3 (a slot that's empty just logs a warning), move the camera, then hold right.
Presets listed in a camera's `locked:` can be recalled but not saved over: holding right on one
just flickers the LED rapidly (and shows "name (locked)" if the overlay is on).
Set a camera's `home:` to a preset name to have it return there after `home_after` seconds
(default 120) with no joystick movement or button presses on that camera.
A keyboard-type USB foot pedal can recall presets directly: program each pedal to send a key and map
keys to preset names under `pedal:`. Find the pedal's device name with `--list-devices`; presses of
unmapped keys are logged with their key name. Tap a pedal to recall its preset (on release); hold it
for `control.save_hold` (2 s) to save the current position to it instead (`pedal.long_press_save`). The pedal is grabbed exclusively, so it doesn't type
into anything else.
With several cameras, map `prev_camera`/`next_camera` to a button in `buttons:` (e.g. left long-press).

## Nx Witness layouts

`next_layout` / `prev_layout` (left long-press in the shipped config) step through `nx.layouts` and send
each as an Nx Witness Generic Event (`source: SpaceMouse`, caption = layout name). In the Nx Desktop
Client, add one event rule per layout: **Event** Generic Event, Source contains `SpaceMouse`,
Caption contains `<layout name>`; **Action** Open Layout -> that layout, for the users/screens that
should switch. Avoid names where one contains another (`Stage` / `Stage 2`), since Nx matches on
"contains". The LED blinks N slow times for layout N.

## Try it

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python onvif_spacemouse.py --test-input          # check axes/directions, no cameras
cp config.example.yaml config.yaml && $EDITOR config.yaml
.venv/bin/python onvif_spacemouse.py -c config.yaml -v
```

If a direction is backwards, flip `invert` for that axis in `axes:`.

## Install as a service

```bash
sudo useradd --system --no-create-home --groups input onvif-spacemouse
sudo mkdir -p /opt/onvif-spacemouse /etc/onvif-spacemouse
sudo cp onvif_spacemouse.py requirements.txt /opt/onvif-spacemouse/
sudo python3 -m venv /opt/onvif-spacemouse/.venv
sudo /opt/onvif-spacemouse/.venv/bin/pip install -r /opt/onvif-spacemouse/requirements.txt
sudo install -m 600 -o onvif-spacemouse config.yaml /etc/onvif-spacemouse/config.yaml
sudo cp deploy/99-spacemouse.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
sudo cp deploy/onvif-spacemouse.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now onvif-spacemouse
journalctl -u onvif-spacemouse -f
```

If `spacenavd` is installed and running, it grabs the device exclusively; stop/disable it.

## Notes

- Movement uses ONVIF `ContinuousMove` with a 1 s `Timeout`, refreshed while you hold the cap, so a camera stops on its own if the service or network dies.
  Set `control.move_timeout: 0` if a camera rejects the Timeout field.
- `separate_zoom` stops twist and slide from both registering when you meant only one.
- Unplugging the SpaceMouse or losing a camera is handled; both reconnect automatically.
