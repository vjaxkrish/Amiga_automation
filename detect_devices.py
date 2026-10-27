#!/usr/bin/env python3
"""
detect_devices.py - show what is plugged in and how devices.py resolves it.

Run this FIRST on any new machine, before any other script in this repo.
It is the five-second check that catches a receiver that did not enumerate,
a radio on the wrong port, or a CAN adapter that came back under a new index.

    python3 detect_devices.py            report
    python3 detect_devices.py --write    freeze the current mapping into
                                         ~/.amiga_devices

The radio pair is the one ambiguous case: two identical FT231X adapters, told
apart only by serial number. Decide which is A (base side) and which is B
(rover side) by plugging in one at a time, then --write to pin it down.
"""

import argparse
import os
import subprocess
import sys

import devices

KEYS = ["AMIGA_F9P", "AMIGA_LG580P", "AMIGA_RADIO_A", "AMIGA_RADIO_B"]
LABELS = {
    "AMIGA_F9P": "F9P base receiver",
    "AMIGA_LG580P": "LG580P rover receiver",
    "AMIGA_RADIO_A": "SiK radio A (base side)",
    "AMIGA_RADIO_B": "SiK radio B (rover side)",
}


def can_report():
    iface = devices.CAN_IFACE
    print(f"\nCAN interface: {iface}")
    try:
        out = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", iface],
            capture_output=True, text=True, timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("  could not run `ip` - skipping")
        return
    if out.returncode != 0:
        print(f"  {iface} does not exist. Adapter unplugged, or run can_setup.sh")
        return
    text = out.stdout
    print(f"  present")
    for token, note in [
        ("ERROR-ACTIVE", "  state ERROR-ACTIVE - healthy"),
        ("ERROR-PASSIVE", "  state ERROR-PASSIVE - bus trouble, do not trust a run"),
        ("BUS-OFF", "  state BUS-OFF - controller has given up, reset the interface"),
    ]:
        if token in text:
            print(note)
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("bitrate"):
            print(f"  {s.split('restart-ms')[0].strip()}")
            if "250000" not in s:
                print("  !! bitrate is not 250000 - the Amiga bus is 250 kbps")
    if "bitrate" not in text:
        print("  !! no bitrate set - interface is down. Run can_setup.sh")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true",
                    help="freeze the current mapping into ~/.amiga_devices")
    args = ap.parse_args()

    present = devices._by_id()
    print(f"/dev/serial/by-id entries: {len(present)}")
    for d in present:
        print(f"  {os.path.basename(d)}")

    print("\nresolved roles:")
    resolved = {}
    missing = []
    for k in KEYS:
        path = devices.resolve(k)
        src = devices.source_of(k)
        exists = os.path.exists(path)
        mark = "ok " if exists else "MISSING"
        if not exists:
            missing.append(k)
        resolved[k] = path
        print(f"  {mark} {LABELS[k]:<26} {path}")
        print(f"      via {src}")

    can_report()

    if missing:
        print(f"\n{len(missing)} device(s) not found. Plug them in, or set the "
              f"environment variable, e.g.:")
        for k in missing:
            print(f"  export {k}=/dev/ttyUSB0")

    if args.write:
        path = os.path.expanduser("~/.amiga_devices")
        with open(path, "w") as fh:
            fh.write("# written by detect_devices.py --write\n")
            fh.write(f"AMIGA_CAN_IFACE={devices.CAN_IFACE}\n")
            for k in KEYS:
                fh.write(f"{k}={resolved[k]}\n")
        print(f"\nwrote {path}")

    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
