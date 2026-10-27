#!/usr/bin/env python3
"""
rover_log.py - Phase 0 step 5. Log everything the controller will need.

Runs on the rover Pi with radio B, the LG580P and the CAN adapter attached.
Forwards RTCM from the radio into the receiver, reads position, heading and
wheel speed, and writes one CSV with a single time base.

    python3 rover_log.py drive              log a manual driving run
    python3 rover_log.py teach              same, but 'm' marks a waypoint
    python3 rover_log.py check              one-shot: are all three sources alive

While running:
    m       mark the current position (teach mode: a plot centre)
    n       mark and label the next mark with an incrementing plot id
    q       stop and write the summary

Output: logs/rover_YYYYmmdd_HHMMSS.csv plus a .json summary.

Everything is sampled on one monotonic clock. Each source updates
asynchronously; a row carries the most recent value from each, with an age
column so stale data is visible rather than silently assumed fresh.
"""

import argparse
import json
import math
import os
import select
import socket
import struct
import sys
import termios
import time
import tty
from datetime import datetime, timezone

import json

import serial

# --- devices ----------------------------------------------------------------

BYID = "/dev/serial/by-id"
LG580P_FRAG = "usb-1a86_USB_Dual_Serial"
RADIO_FRAG = "DU0E5OQL"              # radio B, rover side
CAN_IFACE = "can0"

LG580P_BAUD = 460800
RADIO_BAUD = 57600

COBID_TPDO1 = 0x18E
CAN_FRAME = struct.Struct("<IB3x8s")

# The GNSS antenna sits forward of the vehicle's centre of rotation, so the
# antenna fix is not where the machine "is" for control purposes - a turn in
# place swings it through an arc. Marks are therefore stored as the rotation
# centre, which is what amiga_nav drives to. Keep this equal to
# ANTENNA_OFFSET_M in amiga_nav.py; measure with `amiga_nav.py calib-offset`.
ANTENNA_OFFSET_M = 0.794
EARTH_R = 6371000.0

LOG_RATE_HZ = 10.0
STALE_WARN_S = 1.0

OUT_DIR = os.path.expanduser("~/logs")

COLUMNS = [
    "t_mono", "t_utc", "row_type", "label",
    "lat", "lon", "ctr_lat", "ctr_lon", "alt_m", "fix_q", "nsat", "hdop", "corr_age_s", "gga_age_s",
    "heading_deg", "pitch_deg", "baseline_m", "head_q", "head_nsat", "tar_age_s",
    "can_state", "speed_mm_s", "angular_mrad_s", "soc_pct", "can_age_s",
]

FIX_LABEL = {"0": "none", "1": "standalone", "2": "dgps", "4": "rtk_fixed",
             "5": "rtk_float"}


def find_port(fragment):
    try:
        for name in sorted(os.listdir(BYID)):
            if fragment in name:
                return os.path.join(BYID, name)
    except FileNotFoundError:
        pass
    return None


STATE_PATH = "/dev/shm/rover_state.json"


def read_state():
    try:
        with open(STATE_PATH) as f:
            st = json.load(f)
        if time.time() - st["t_wall"] < 2.0:
            return st
    except (OSError, ValueError, KeyError):
        pass
    return None


def rotation_centre(lat, lon, heading_deg):
    """Shift an antenna fix back along the heading to the rotation centre."""
    if lat is None or lon is None or heading_deg is None:
        return None, None
    th = math.radians(heading_deg)
    dn = -ANTENNA_OFFSET_M * math.cos(th)      # metres north
    de = -ANTENNA_OFFSET_M * math.sin(th)      # metres east
    clat = lat + math.degrees(dn / EARTH_R)
    clon = lon + math.degrees(de / (EARTH_R * math.cos(math.radians(lat))))
    return clat, clon


def nmea_ok(line):
    """Verify the NMEA checksum. Bad frames from a lossy radio are real."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, ck = line[1:].partition("*")
    try:
        want = int(ck[:2], 16)
    except ValueError:
        return False
    got = 0
    for c in body:
        got ^= ord(c)
    return got == want


def dm_to_deg(val, hemi):
    """NMEA ddmm.mmmm -> signed decimal degrees."""
    if not val:
        return None
    try:
        v = float(val)
    except ValueError:
        return None
    deg = int(v // 100)
    minutes = v - deg * 100
    out = deg + minutes / 60.0
    if hemi in ("S", "W"):
        out = -out
    return out


class Source:
    """Latest value plus the monotonic time it arrived."""

    def __init__(self):
        self.data = {}
        self.t = None

    def update(self, data, t):
        self.data = data
        self.t = t

    def age(self, now):
        return None if self.t is None else now - self.t


def open_can():
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((CAN_IFACE,))
    s.setblocking(False)
    return s


# --- parsing ----------------------------------------------------------------

def parse_gga(f):
    if len(f) < 10:
        return None
    return {
        "lat": dm_to_deg(f[2], f[3]),
        "lon": dm_to_deg(f[4], f[5]),
        "fix_q": f[6],
        "nsat": f[7],
        "hdop": f[8],
        "alt_m": f[9],
        "corr_age_s": f[13] if len(f) > 13 else "",
    }


def parse_tar(f):
    """$PQTMTAR,1,utc,quality,,baseline,pitch,,heading,accA,,accB,nsat"""
    if len(f) < 13:
        return None

    def num(i):
        try:
            return float(f[i])
        except (ValueError, IndexError):
            return None

    return {
        "head_q": f[3],
        "baseline_m": num(5),
        "pitch_deg": num(6),
        "heading_deg": num(8),
        "head_nsat": f[12].split("*")[0] if len(f) > 12 else "",
    }


# --- commands ---------------------------------------------------------------

def cmd_check(args):
    lg = find_port(LG580P_FRAG)
    radio = find_port(RADIO_FRAG)
    print(f"LG580P  {lg or 'NOT FOUND'}")
    print(f"radio B {radio or 'NOT FOUND'}")
    try:
        sock = open_can()
        print(f"CAN     {CAN_IFACE} open")
    except OSError as e:
        print(f"CAN     {CAN_IFACE} FAILED: {e}")
        sock = None

    if not (lg and radio and sock):
        print("\nnot all sources present")
        return 1

    print("\nsampling 5 s ...")
    seen = {"gga": 0, "tar": 0, "can": 0, "rtcm_bytes": 0}
    fix = "-"
    with serial.Serial(lg, LG580P_BAUD, timeout=0.05) as lgs, \
         serial.Serial(radio, RADIO_BAUD, timeout=0.05) as rad:
        buf = b""
        end = time.monotonic() + 5
        while time.monotonic() < end:
            air = rad.read(1024)
            if air:
                lgs.write(air)
                seen["rtcm_bytes"] += len(air)
            buf += lgs.read(2048)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                s = raw.decode("ascii", "ignore").strip()
                if not nmea_ok(s):
                    continue
                if s.startswith("$GNGGA"):
                    seen["gga"] += 1
                    g = parse_gga(s.split(","))
                    if g:
                        fix = FIX_LABEL.get(g["fix_q"], g["fix_q"])
                elif s.startswith("$PQTMTAR"):
                    seen["tar"] += 1
            while True:
                try:
                    raw = sock.recv(16)
                except (BlockingIOError, OSError):
                    break
                cid, dlc, data = CAN_FRAME.unpack(raw)
                if (cid & 0x1FFFFFFF) == COBID_TPDO1:
                    seen["can"] += 1

    print(f"  GGA      {seen['gga']:>5}   fix: {fix}")
    print(f"  PQTMTAR  {seen['tar']:>5}")
    print(f"  0x18E    {seen['can']:>5}")
    print(f"  RTCM in  {seen['rtcm_bytes']:>5} B")
    missing = [k for k in ("gga", "tar", "can") if seen[k] == 0]
    if missing:
        print(f"\nNO DATA FROM: {', '.join(missing)}")
        return 1
    if seen["rtcm_bytes"] == 0:
        print("\nno RTCM arriving - is the base running?")
    print("\nall sources alive")
    return 0


def cmd_log(args):
    teach = args.command == "teach"
    lg_port = find_port(LG580P_FRAG)
    radio_port = find_port(RADIO_FRAG)
    if not lg_port or not radio_port:
        print("missing LG580P or radio - run 'check'")
        return 1
    try:
        sock = open_can()
    except OSError as e:
        print(f"cannot open {CAN_IFACE}: {e}")
        return 1

    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUT_DIR, f"rover_{stamp}.csv")
    json_path = os.path.join(OUT_DIR, f"rover_{stamp}.json")

    gps = Source()
    att = Source()
    can = Source()

    marks = []
    plot_id = 1
    rows = 0
    rtcm_bytes = 0
    fix_counts = {}
    t0 = time.monotonic()

    print(f"logging to {csv_path}")
    print("  m = mark    n = mark as next plot id    q = stop\n")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    csv = open(csv_path, "w", buffering=1)
    csv.write(",".join(COLUMNS) + "\n")

    def snapshot(now, row_type="sample", label=""):
        g, a, c = gps.data, att.data, can.data
        clat, clon = rotation_centre(g.get("lat"), g.get("lon"),
                                     a.get("heading_deg"))
        vals = {
            "t_mono": f"{now - t0:.3f}",
            "t_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "row_type": row_type,
            "label": label,
            "lat": f"{g['lat']:.9f}" if g.get("lat") is not None else "",
            "lon": f"{g['lon']:.9f}" if g.get("lon") is not None else "",
            "ctr_lat": f"{clat:.9f}" if clat is not None else "",
            "ctr_lon": f"{clon:.9f}" if clon is not None else "",
            "alt_m": g.get("alt_m", ""),
            "fix_q": g.get("fix_q", ""),
            "nsat": g.get("nsat", ""),
            "hdop": g.get("hdop", ""),
            "corr_age_s": g.get("corr_age_s", ""),
            "gga_age_s": f"{gps.age(now):.2f}" if gps.age(now) is not None else "",
            "heading_deg": f"{a['heading_deg']:.4f}" if a.get("heading_deg") is not None else "",
            "pitch_deg": f"{a['pitch_deg']:.4f}" if a.get("pitch_deg") is not None else "",
            "baseline_m": f"{a['baseline_m']:.4f}" if a.get("baseline_m") is not None else "",
            "head_q": a.get("head_q", ""),
            "head_nsat": a.get("head_nsat", ""),
            "tar_age_s": f"{att.age(now):.2f}" if att.age(now) is not None else "",
            "can_state": c.get("state", ""),
            "speed_mm_s": c.get("speed", ""),
            "angular_mrad_s": c.get("angular", ""),
            "soc_pct": c.get("soc", ""),
            "can_age_s": f"{can.age(now):.2f}" if can.age(now) is not None else "",
        }
        csv.write(",".join(str(vals[k]) for k in COLUMNS) + "\n")
        return vals

    try:
        tty.setcbreak(fd)
        with serial.Serial(lg_port, LG580P_BAUD, timeout=0) as lgs, \
             serial.Serial(radio_port, RADIO_BAUD, timeout=0) as rad:

            lg_buf = b""
            period = 1.0 / LOG_RATE_HZ
            next_row = time.monotonic()
            next_status = time.monotonic() + 1.0

            while True:
                now = time.monotonic()

                # radio -> receiver
                air = rad.read(2048)
                if air:
                    lgs.write(air)
                    rtcm_bytes += len(air)

                # receiver -> parse
                lg_buf += lgs.read(4096)
                while b"\n" in lg_buf:
                    raw, lg_buf = lg_buf.split(b"\n", 1)
                    s = raw.decode("ascii", "ignore").strip()
                    if not nmea_ok(s):
                        continue
                    f = s.split(",")
                    if s.startswith("$GNGGA"):
                        d = parse_gga(f)
                        if d:
                            gps.update(d, now)
                            q = FIX_LABEL.get(d["fix_q"], d["fix_q"])
                            fix_counts[q] = fix_counts.get(q, 0) + 1
                    elif s.startswith("$PQTMTAR"):
                        d = parse_tar(f)
                        if d:
                            att.update(d, now)

                # CAN
                while True:
                    try:
                        raw = sock.recv(16)
                    except (BlockingIOError, OSError):
                        break
                    cid, dlc, data = CAN_FRAME.unpack(raw)
                    if (cid & 0x1FFFFFFF) != COBID_TPDO1 or dlc < 8:
                        continue
                    st, sp, ang = struct.unpack_from("<Bhh", data)
                    can.update({"state": st, "speed": sp, "angular": ang,
                                "soc": data[7]}, now)

                # keyboard
                if select.select([sys.stdin], [], [], 0)[0]:
                    c = sys.stdin.read(1).lower()
                    if c == "q":
                        break
                    if c in ("m", "n"):
                        label = f"plot_{plot_id:03d}" if (teach or c == "n") else f"mark_{len(marks)+1}"
                        v = snapshot(now, "mark", label)
                        # The mark's lat/lon is the ROTATION CENTRE, because
                        # that is the point amiga_nav drives to. The antenna
                        # fix is kept alongside it for reference.
                        marks.append({
                            "t_mono": v["t_mono"], "t_utc": v["t_utc"],
                            "label": label,
                            "lat": v["ctr_lat"] or v["lat"],
                            "lon": v["ctr_lon"] or v["lon"],
                            "ant_lat": v["lat"], "ant_lon": v["lon"],
                            "fix_q": v["fix_q"],
                            "heading_deg": v["heading_deg"],
                            "antenna_offset_m": ANTENNA_OFFSET_M,
                        })
                        rows += 1
                        if teach or c == "n":
                            plot_id += 1
                        sys.stdout.write(
                            f"\r  MARK {label}  "
                            f"{marks[-1]['lat']},{marks[-1]['lon']}  "
                            f"fix {FIX_LABEL.get(v['fix_q'], v['fix_q'])}"
                            f"  (rotation centre)\n")

                # periodic row
                if now >= next_row:
                    snapshot(now)
                    rows += 1
                    next_row += period
                    if next_row < now:
                        next_row = now + period

                # status line
                if now >= next_status:
                    next_status = now + 1.0
                    g, c = gps.data, can.data
                    q = FIX_LABEL.get(g.get("fix_q", ""), "-")
                    stale = []
                    for nm, src in (("gps", gps), ("att", att), ("can", can)):
                        a = src.age(now)
                        if a is None or a > STALE_WARN_S:
                            stale.append(nm)
                    warn = ("  STALE: " + ",".join(stale)) if stale else ""
                    sys.stdout.write(
                        f"\r  {now-t0:7.1f}s  {q:<11} sats {g.get('nsat','--'):>2}  "
                        f"hdg {att.data.get('heading_deg') or 0:7.2f}  "
                        f"spd {c.get('speed','-'):>5} mm/s  "
                        f"rows {rows:>6}  marks {len(marks):>3}{warn}   ")
                    sys.stdout.flush()

                time.sleep(0.002)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        csv.close()
        sock.close()

    total = time.monotonic() - t0
    summary = {
        "started_utc": stamp,
        "duration_s": round(total, 1),
        "rows": rows,
        "rtcm_bytes": rtcm_bytes,
        "fix_quality_counts": fix_counts,
        "marks": marks,
        "csv": csv_path,
    }
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n\n  {rows} rows over {total:.0f} s -> {csv_path}")
    print(f"  {len(marks)} marks -> {json_path}")
    if fix_counts:
        tot = sum(fix_counts.values())
        parts = [f"{k} {100*v/tot:.0f}%" for k, v in sorted(fix_counts.items())]
        print(f"  fix quality: {', '.join(parts)}")
    return 0


COMMANDS = {"drive": cmd_log, "teach": cmd_log, "check": cmd_check}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=COMMANDS)
    args = p.parse_args()
    return COMMANDS[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
