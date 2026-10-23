#!/usr/bin/env python3
"""
devices.py - every hardware-specific string in one place.

This is the ONLY file that should need editing when the code moves to a
different Amiga, a different Pi, or a different set of USB receivers.

Resolution order for each port, first hit wins:

    1. environment variable        e.g.  export AMIGA_F9P=/dev/ttyACM0
    2. ~/.amiga_devices            shell-style KEY=value, one per line
    3. auto-detect by USB VID:PID  scans /dev/serial/by-id
    4. the College Station default baked in below

So on new hardware you normally change nothing: the USB VID:PID of a ZED-F9P
is the same everywhere, and auto-detect finds it under its new serial number.
The baked-in defaults are kept only so that the exact College Station rig is
recorded in version control.

The one case auto-detect cannot resolve by itself is the SiK radio pair: two
identical FTDI FT231X adapters, distinguishable only by serial number. Those
are assigned by position in the sorted device list unless you override them,
and `detect_devices.py` prints which is which so you can pin them down.

    python3 detect_devices.py            show what is plugged in now
    python3 detect_devices.py --write    write the current mapping to
                                         ~/.amiga_devices so it is frozen
"""

import glob
import os

# --- College Station rig, Oct 2026 ------------------------------------------
# Kept as the fallback and as a record. Do not edit these to make Dallas work;
# set the environment variables or write ~/.amiga_devices instead.

DEFAULTS = {
    "AMIGA_CAN_IFACE": "can0",
    "AMIGA_F9P": "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00",
    "AMIGA_LG580P": "/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5932003008-if00",
    "AMIGA_RADIO_A": "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E44NK-if00-port0",
    "AMIGA_RADIO_B": "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E5OQL-if00-port0",
}

# --- baud rates -------------------------------------------------------------
# F9P is USB CDC, so its baud is ignored by the hardware; pyserial wants a
# number. LG580P and the SiK radios are real UARTs behind USB bridges and
# these values matter.

F9P_BAUD = 38400
LG580P_BAUD = 460800
RADIO_BAUD = 57600

# --- CAN --------------------------------------------------------------------
# 250 kbps is the NMEA 2000 standard rate and is confirmed in the farm-ng FAQ.
# restart-ms 0 is deliberate: a self-recovering interface hides the faults we
# need to see. See docs/can-bus-map.md.

CAN_BITRATE = 250000
CAN_RESTART_MS = 0
CAN_TXQUEUELEN = 10

# --- auto-detect ------------------------------------------------------------
# by-id names embed the USB vendor and product strings, so matching on those
# substrings identifies the device model regardless of its serial number.

SIGNATURES = {
    "AMIGA_F9P": ["u-blox_GNSS_receiver", "u-blox_AG"],
    "AMIGA_LG580P": ["1a86_USB_Dual_Serial"],       # CH342 bridge inside the LG580P
    "_SIK": ["FTDI_FT231X_USB_UART"],               # both radios match this
}


def _by_id():
    return sorted(glob.glob("/dev/serial/by-id/*"))


def _config_file():
    path = os.path.expanduser("~/.amiga_devices")
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip().strip('"').strip("'")
    return out


def _detect(key, devs):
    if key in ("AMIGA_RADIO_A", "AMIGA_RADIO_B"):
        radios = [d for d in devs if any(s in d for s in SIGNATURES["_SIK"])]
        # keep only one node per physical adapter
        radios = [d for d in radios if d.endswith("-port0")]
        idx = 0 if key == "AMIGA_RADIO_A" else 1
        return radios[idx] if len(radios) > idx else None
    sigs = SIGNATURES.get(key)
    if not sigs:
        return None
    for d in devs:
        if any(s in d for s in sigs):
            return d
    return None


def resolve(key, quiet=True):
    """Return the device path for key, trying env, file, detect, default."""
    if key in os.environ:
        return os.environ[key]
    cfg = _config_file()
    if key in cfg:
        return cfg[key]
    found = _detect(key, _by_id())
    if found:
        return found
    return DEFAULTS[key]


def source_of(key):
    """Where resolve() got its answer. For detect_devices.py output."""
    if key in os.environ:
        return "env"
    if key in _config_file():
        return "~/.amiga_devices"
    if _detect(key, _by_id()):
        return "auto-detect"
    return "baked-in default"


# --- the names the scripts import -------------------------------------------

CAN_IFACE = os.environ.get("AMIGA_CAN_IFACE", DEFAULTS["AMIGA_CAN_IFACE"])
F9P_PORT = resolve("AMIGA_F9P")
LG580P_PORT = resolve("AMIGA_LG580P")
RADIO_A = resolve("AMIGA_RADIO_A")
RADIO_B = resolve("AMIGA_RADIO_B")
