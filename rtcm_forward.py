#!/usr/bin/env python3
"""
rtcm_forward.py - radio B -> LG580P, continuously, from boot.

The receiver drops out of RTK within about a minute of corrections stopping,
and reacquiring a fix over the radio takes several minutes. So forwarding
must not be something a program does only while it happens to be running.
This runs as a service and nothing else forwards.

    python3 rtcm_forward.py           run in the foreground
    python3 rtcm_forward.py --status  one-shot: is it running, is it fixed

Installed as a systemd unit, this is the only process that opens the radio.
rover_log.py and amiga_nav.py read the receiver but leave forwarding alone.
"""

import argparse
import os
import signal
import sys
import time
from datetime import datetime, timezone

import json

import serial

BYID = "/dev/serial/by-id"
LG580P_FRAG = "usb-1a86_USB_Dual_Serial"
RADIO_FRAG = "DU0E5OQL"

LG580P_BAUD = 460800
RADIO_BAUD = 57600

LOG_PATH = os.path.expanduser("~/logs/rtcm_forward.log")

# The forwarder is the ONLY process that opens the radio or the receiver.
# Two readers on one serial port each get a fraction of the bytes and both
# starve, which looks exactly like a dead link. So position and heading are
# republished here and every other script reads this file instead of the
# port. /dev/shm is tmpfs: no disk, no wear, microsecond reads.
STATE_PATH = "/dev/shm/rover_state.json"
STATUS_PATH = STATE_PATH          # kept for older callers
REPORT_S = 60.0

STOP = False


def _sig(signum, frame):
    global STOP
    STOP = True


def find_port(frag):
    try:
        for n in sorted(os.listdir(BYID)):
            if frag in n:
                return os.path.join(BYID, n)
    except FileNotFoundError:
        pass
    return None


def publish(state):
    """Atomic write, so a reader never sees a half-written file."""
    tmp = STATE_PATH + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(state, f)
        os.replace(tmp, STATE_PATH)
    except OSError:
        pass


def log(msg, fh=None):
    line = f"{datetime.now(timezone.utc).isoformat(timespec='seconds')}  {msg}"
    print(line, flush=True)
    if fh:
        fh.write(line + "\n")
        fh.flush()


FIX = {"0": "none", "1": "standalone", "2": "dgps/sbas",
       "4": "rtk_fixed", "5": "rtk_float"}


def run():
    signal.signal(signal.SIGTERM, _sig)
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
    fh = open(LOG_PATH, "a")

    lg_port = find_port(LG580P_FRAG)
    rd_port = find_port(RADIO_FRAG)
    if not lg_port:
        log("FATAL: LG580P not found", fh)
        return 1
    if not rd_port:
        log(f"FATAL: radio {RADIO_FRAG} not found", fh)
        return 1
    log(f"forwarding {rd_port} -> {lg_port}", fh)

    total = 0
    fix_q = "?"
    nsat = "?"
    hdop = ""
    age = ""
    lat = lon = alt = None
    heading = pitch = baseline = None
    head_q = ""
    last_report = time.monotonic()
    last_fix = None
    buf = b""

    def dm(v, h):
        if not v:
            return None
        try:
            f = float(v)
        except ValueError:
            return None
        d = int(f // 100)
        out = d + (f - d * 100) / 60.0
        return -out if h in ("S", "W") else out

    def ck_ok(line):
        if not line.startswith("$") or "*" not in line:
            return False
        body, _, c = line[1:].partition("*")
        try:
            want = int(c[:2], 16)
        except ValueError:
            return False
        g = 0
        for ch in body:
            g ^= ord(ch)
        return g == want

    with serial.Serial(rd_port, RADIO_BAUD, timeout=0.02) as rad, \
         serial.Serial(lg_port, LG580P_BAUD, timeout=0.02) as lg:
        while not STOP:
            air = rad.read(2048)
            if air:
                lg.write(air)
                total += len(air)

            buf += lg.read(4096)
            if len(buf) > 65536:
                buf = buf[-4096:]
            fresh = False
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                sline = raw.decode("ascii", "ignore").strip()
                if not ck_ok(sline):
                    continue
                f = sline.split(",")
                if sline.startswith("$GNGGA") and len(f) > 9:
                    la, lo = dm(f[2], f[3]), dm(f[4], f[5])
                    if la is not None and lo is not None:
                        lat, lon = la, lo
                        fix_q, nsat, hdop = f[6], f[7], f[8]
                        alt = f[9]
                        age = f[13] if len(f) > 13 else ""
                        fresh = True
                elif sline.startswith("$PQTMTAR") and len(f) > 12:
                    def num(i):
                        try:
                            return float(f[i])
                        except (ValueError, IndexError):
                            return None
                    head_q = f[3]
                    baseline = num(5)
                    pitch = num(6)
                    heading = num(8)
                    fresh = True

            if fresh:
                publish({
                    "t_wall": time.time(),
                    "lat": lat, "lon": lon, "alt_m": alt,
                    "fix_q": fix_q, "nsat": nsat, "hdop": hdop,
                    "corr_age_s": age,
                    "heading_deg": heading, "pitch_deg": pitch,
                    "baseline_m": baseline, "head_q": head_q,
                    "rtcm_bytes": total,
                })

            now = time.monotonic()
            label = FIX.get(fix_q, fix_q)

            if label != last_fix:
                log(f"fix -> {label}  sats {nsat}  rtcm {total} B", fh)
                last_fix = label

            if now - last_report >= REPORT_S:
                last_report = now
                log(f"{label}  sats {nsat}  corr age {age or '-'}  "
                    f"rtcm {total} B", fh)

            time.sleep(0.005)

    log("stopping", fh)
    return 0


def status():
    try:
        with open(STATE_PATH) as f:
            st = json.load(f)
    except (OSError, ValueError):
        print("not running (no state file)")
        return 1
    stale = time.time() - st["t_wall"]
    if stale > 5:
        print(f"STALE: last update {stale:.1f} s ago - service may be dead")
        return 1
    label = FIX.get(st.get("fix_q"), st.get("fix_q"))
    h = st.get("heading_deg")
    print(f"{label}   sats {st.get('nsat')}   "
          f"corr age {st.get('corr_age_s') or '-'}   "
          f"heading {h if h is None else f'{h:.2f}'}   "
          f"rtcm {st.get('rtcm_bytes')} B   ({stale*1000:.0f} ms ago)")
    return 0 if label == "rtk_fixed" else 2


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--status", action="store_true")
    args = p.parse_args()
    try:
        return status() if args.status else run()
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
