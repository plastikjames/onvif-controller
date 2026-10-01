#!/usr/bin/env python3
"""Work out why an ONVIF camera refuses our login. Usage: diag_auth.py [config.yaml] [camera index]"""

import datetime
import re
import sys

import requests
import yaml
from onvif import ONVIFCamera
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

cfg_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0
cam = yaml.safe_load(open(cfg_path))["cameras"][idx]
host, port, user, pw = cam["host"], cam.get("port", 80), cam["user"], str(cam["password"])
url = f"http://{host}:{port}/onvif/device_service"
hdr = {"Content-Type": "application/soap+xml"}


def body(op):
    return ('<?xml version="1.0"?><s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">'
            f'<s:Body><{op} xmlns="http://www.onvif.org/ver10/device/wsdl"/></s:Body></s:Envelope>')


def text(xml):
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", xml)).strip()[:200]


print(f"camera {cam.get('name', host)} at {url}, user '{user}', password {len(pw)} chars")

r = requests.post(url, data=body("GetSystemDateAndTime"), headers=hdr, timeout=5)
m = re.search(r"UTCDateTime>(.*?)</[^>]*UTCDateTime", r.text, re.S)
print("camera UTC:", text(m.group(1)) if m else f"({r.status_code}) {text(r.text)}")
print("local  UTC:", datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S"))

for name, auth in [("no auth", None), ("HTTP digest", HTTPDigestAuth(user, pw)), ("HTTP basic", HTTPBasicAuth(user, pw))]:
    try:
        r = requests.post(url, data=body("GetDeviceInformation"), headers=hdr, auth=auth, timeout=5)
        ok = r.status_code == 200 and "Fault" not in r.text
        print(f"{name:12}: {'OK ' if ok else 'FAIL'} ({r.status_code}) {text(r.text)[:120]}")
    except Exception as e:
        print(f"{name:12}: FAIL {e}")

for adjust in (False, True):
    try:
        c = ONVIFCamera(host, port, user, pw, adjust_time=adjust)
        print(f"WS-Security adjust_time={adjust}: OK", c.devicemgmt.GetDeviceInformation())
    except Exception as e:
        print(f"WS-Security adjust_time={adjust}: FAIL {e}")
