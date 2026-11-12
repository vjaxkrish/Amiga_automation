#!/usr/bin/env python3
"""
rover_log.py - Phase 0 step 5. Log everything the controller will need.

Runs on the rover Pi with radio B, the LG580P and the CAN adapter attached.
Reads position, heading and wheel speed and writes one CSV with a single
time base.

GNSS comes from rtcm_forward.service when it is running (the normal case):
the service keeps forwarding corrections and this reads its state file, so
there is nothing to stop or start. Only if the service is NOT running does
this open the radio and receiver itself and forward RTCM on its own.

    python3 rover_log.py drive              log a manual driving run
    python3 rover_log.py teach              same, but 'm' marks a waypoint
    python3 rover_log.py check              one-shot: are all three sources alive

While running:
    m       mark the current position (teach mode: a plot centre)
    n       mark and label the next mark with an incrementing plot id
    q       stop and write the summary

A mark is not one reading. Pressing m starts a 3 s average with the machine
STOPPED; the mark is refused, and the plot id not used up, unless every
reading in that window is RTK fixed, the machine did not move, and the
readings agree to within a couple of centimetres. Press m again to retry.
Teach each plot exactly as it will be driven: same route, same direction,
stopped square to the row. Errors that do not change between teach and
replay then cancel - see docs/navigation.md.

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

import serial

from vehicle import (ANTENNA_OFFSET_M, HEADING_OFFSET_DEG, true_heading,
                     rotation_centre, circular_mean_deg, metres_between)

# --- devices ----------------------------------------------------------------

BYID = "/dev/serial/by-id"
LG580P_FRAG = "usb-1a86_USB_Dual_Serial"
RADIO_FRAG = "DU0E5OQL"              # radio B, rover side
CAN_IFACE = "can0"

LG580P_BAUD = 460800
RADIO_BAUD = 57600

COBID_TPDO1 = 0x18E
CAN_FRAME = struct.Struct("<IB3x8s")

# Marks are stored as the vehicle's rotation centre, which is what amiga_nav
# drives to, computed with the shared constants in vehicle.py. The antenna
# fix and TRUE heading are stored too, and amiga_nav re-derives the centre
# from them, so a later change to ANTENNA_OFFSET_M does not strand old marks.
MARK_FORMAT = 2

# Averaging a mark. Measured noise on a stationary machine is ~1 cm; one
# reading bakes that noise into every future run.
MARK_AVG_S = 3.0               # average over this long, machine stopped
MARK_MIN_SAMPLES = 3           # fewer GGA than this in the window: refuse
MARK_MAX_POS_SD_M = 0.02       # readings scattered more than this: refuse
MARK_MAX_HDG_SD_DEG = 0.5      # heading scattered more than this: refuse
MARK_MAX_SPEED_MM_S = 10       # wheel speed above this: it was moving, refuse

LOG_RATE_HZ = 10.0
STALE_WARN_S = 1.0

OUT_DIR = os.path.expanduser("~/logs")

COLUMNS = [
    "t_mono", "t_utc", "row_type", "label",
    "lat", "lon", "ctr_lat", "ctr_lon", "alt_m", "fix_q", "nsat", "hdop", "corr_age_s", "gga_age_s",
    "heading_deg", "heading_true_deg", "pitch_deg", "baseline_m", "head_q", "head_nsat", "tar_age_s",
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


def average_mark(gga, raw_headings, speeds):
    """Average a stationary window into one mark.

    gga:          list of parsed GGA dicts (lat, lon, fix_q)
    raw_headings: list of PQTMTAR heading values, raw from the receiver
    speeds:       list of CAN wheel speeds in mm/s

    Returns (fields, None) on success or (None, reason) if the window is not
    good enough to teach from."""
    pts = [g for g in gga if g.get("lat") is not None and g.get("lon") is not None]
    if len(pts) < MARK_MIN_SAMPLES:
        return None, f"only {len(pts)} position readings in {MARK_AVG_S:.0f} s"
    not_fixed = [g["fix_q"] for g in pts if g.get("fix_q") != "4"]
    if not_fixed:
        return None, (f"{len(not_fixed)} of {len(pts)} readings not RTK fixed "
                      f"(fix {','.join(sorted(set(not_fixed)))})")
    if speeds and max(abs(v) for v in speeds) > MARK_MAX_SPEED_MM_S:
        return None, (f"machine moving ({max(abs(v) for v in speeds)} mm/s) - "
                      f"stop fully before marking")
    hdgs = [true_heading(h) for h in raw_headings if h is not None]
    if not hdgs:
        return None, "no heading readings"

    lat = sum(g["lat"] for g in pts) / len(pts)
    lon = sum(g["lon"] for g in pts) / len(pts)
    sd = math.sqrt(sum(metres_between(lat, lon, g["lat"], g["lon"]) ** 2
                       for g in pts) / len(pts))
    if sd > MARK_MAX_POS_SD_M:
        return None, (f"position scattered {sd*100:.1f} cm - multipath? "
                      f"limit {MARK_MAX_POS_SD_M*100:.0f} cm")
    hdg, hsd = circular_mean_deg(hdgs)
    if hsd > MARK_MAX_HDG_SD_DEG:
        return None, (f"heading scattered {hsd:.2f} deg, "
                      f"limit {MARK_MAX_HDG_SD_DEG} deg")

    clat, clon = rotation_centre(lat, lon, hdg)
    return {
        "lat": clat, "lon": clon,              # rotation centre: drive here
        "ant_lat": lat, "ant_lon": lon,        # mean antenna fix
        "heading_deg": round(hdg, 4),          # TRUE vehicle heading
        "fix_q": "4",
        "n_samples": len(pts),
        "pos_sd_cm": round(sd * 100, 2),
        "heading_sd_deg": round(hsd, 3),
        "antenna_offset_m": ANTENNA_OFFSET_M,
        "heading_offset_deg": HEADING_OFFSET_DEG,
    }, None


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


class ForwarderFeed:
    """GNSS readings from rtcm_forward.service, via its state file.

    The forwarder owns the radio and the receiver and keeps corrections
    flowing whether or not anything else is running, so the RTK fix
    survives between programs. Reading its file instead of the ports means
    the service never has to be stopped for logging or teaching."""

    name = "rtcm_forward"
    POLL_S = 0.02

    def __init__(self):
        self.next_poll = 0.0
        self.last_gga_n = None
        self.last_tar_n = None
        self.last_gga_key = None
        self.rtcm0 = None
        self.rtcm_bytes = 0

    def close(self):
        pass

    def poll(self, now):
        """Return a list of ('gga'|'tar', dict, age_s) for readings that are
        NEW since the last poll."""
        if now < self.next_poll:
            return []
        self.next_poll = now + self.POLL_S
        st = read_state()
        if st is None:
            return []
        age = max(0.0, time.time() - st["t_wall"])
        rb = st.get("rtcm_bytes") or 0
        if self.rtcm0 is None:
            self.rtcm0 = rb
        self.rtcm_bytes = rb - self.rtcm0
        out = []

        gga = {"lat": st.get("lat"), "lon": st.get("lon"),
               "fix_q": str(st.get("fix_q", "")), "nsat": st.get("nsat", ""),
               "hdop": st.get("hdop", ""), "alt_m": st.get("alt_m", ""),
               "corr_age_s": st.get("corr_age_s", "")}
        n = st.get("gga_n")
        if n is not None:
            new = n != self.last_gga_n
            self.last_gga_n = n
        else:
            # older forwarder without counters: a change in any field
            key = tuple(gga.values())
            new = key != self.last_gga_key
            self.last_gga_key = key
        if new and gga["lat"] is not None:
            out.append(("gga", gga, age))

        n = st.get("tar_n")
        if n is None or n != self.last_tar_n:
            self.last_tar_n = n
            if st.get("heading_deg") is not None:
                out.append(("tar", {"head_q": st.get("head_q", ""),
                                    "baseline_m": st.get("baseline_m"),
                                    "pitch_deg": st.get("pitch_deg"),
                                    "heading_deg": st.get("heading_deg"),
                                    "head_nsat": ""}, age))
        return out


class PortFeed:
    """GNSS readings straight from the ports, forwarding RTCM itself. Only
    used when rtcm_forward is NOT running - two readers on one serial port
    each get a fraction of the bytes and both starve."""

    name = "ports"

    def __init__(self, lg_port, radio_port):
        self.lg = serial.Serial(lg_port, LG580P_BAUD, timeout=0)
        self.rad = serial.Serial(radio_port, RADIO_BAUD, timeout=0)
        self.buf = b""
        self.rtcm_bytes = 0

    def close(self):
        for h in (self.lg, self.rad):
            try:
                h.close()
            except Exception:
                pass

    def poll(self, now):
        out = []
        air = self.rad.read(2048)
        if air:
            self.lg.write(air)
            self.rtcm_bytes += len(air)
        self.buf += self.lg.read(4096)
        if len(self.buf) > 65536:
            self.buf = self.buf[-4096:]
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            s = raw.decode("ascii", "ignore").strip()
            if not nmea_ok(s):
                continue
            f = s.split(",")
            if s.startswith("$GNGGA"):
                d = parse_gga(f)
                if d:
                    out.append(("gga", d, 0.0))
            elif s.startswith("$PQTMTAR"):
                d = parse_tar(f)
                if d:
                    out.append(("tar", d, 0.0))
        return out


def open_feed():
    """The forwarder's file if it is running, else the ports. Returns
    (feed, None) or (None, reason)."""
    if read_state() is not None:
        return ForwarderFeed(), None
    lg = find_port(LG580P_FRAG)
    radio = find_port(RADIO_FRAG)
    if not lg or not radio:
        return None, (f"rtcm_forward is not running and the ports are missing "
                      f"(LG580P {lg or 'NOT FOUND'}, radio B "
                      f"{radio or 'NOT FOUND'})")
    try:
        return PortFeed(lg, radio), None
    except (serial.SerialException, OSError) as e:
        return None, f"cannot open the GNSS ports: {e}"


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
    feed, why = open_feed()
    if feed is None:
        print(f"GNSS    {why}")
    else:
        print(f"GNSS    via {feed.name}"
              + ("" if feed.name == "rtcm_forward"
                 else "  (rtcm_forward not running - opening the ports)"))
    try:
        sock = open_can()
        print(f"CAN     {CAN_IFACE} open")
    except OSError as e:
        print(f"CAN     {CAN_IFACE} FAILED: {e}")
        sock = None

    if not (feed and sock):
        print("\nnot all sources present")
        return 1

    print("\nsampling 5 s ...")
    seen = {"gga": 0, "tar": 0, "can": 0}
    fix = "-"
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            for kind, d, _ in feed.poll(time.monotonic()):
                seen[kind] += 1
                if kind == "gga":
                    fix = FIX_LABEL.get(d["fix_q"], d["fix_q"])
            while True:
                try:
                    raw = sock.recv(16)
                except (BlockingIOError, OSError):
                    break
                cid, dlc, data = CAN_FRAME.unpack(raw)
                if (cid & 0x1FFFFFFF) == COBID_TPDO1:
                    seen["can"] += 1
            time.sleep(0.005)
    finally:
        feed.close()
        sock.close()

    print(f"  GGA      {seen['gga']:>5}   fix: {fix}")
    print(f"  PQTMTAR  {seen['tar']:>5}")
    print(f"  0x18E    {seen['can']:>5}")
    print(f"  RTCM in  {feed.rtcm_bytes:>5} B")
    missing = [k for k in ("gga", "tar", "can") if seen[k] == 0]
    if missing:
        print(f"\nNO DATA FROM: {', '.join(missing)}")
        return 1
    if feed.rtcm_bytes == 0:
        print("\nno RTCM arriving - is the base running?")
    print("\nall sources alive")
    return 0


def cmd_log(args):
    teach = args.command == "teach"
    feed, why = open_feed()
    if feed is None:
        print(f"{why} - run 'check'")
        return 1
    try:
        sock = open_can()
    except OSError as e:
        feed.close()
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
    pending = None             # a mark being averaged: label, t_end, samples
    rejected = 0
    plot_id = 1
    rows = 0
    fix_counts = {}
    t0 = time.monotonic()

    print(f"logging to {csv_path}")
    if feed.name == "rtcm_forward":
        print("  GNSS via rtcm_forward (the service keeps forwarding corrections)")
    else:
        print("  GNSS via the ports directly - rtcm_forward is not running")
    print("  m = mark    n = mark as next plot id    q = stop\n")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    csv = open(csv_path, "w", buffering=1)
    csv.write(",".join(COLUMNS) + "\n")

    def snapshot(now, row_type="sample", label=""):
        g, a, c = gps.data, att.data, can.data
        hdg_true = true_heading(a.get("heading_deg"))
        clat, clon = rotation_centre(g.get("lat"), g.get("lon"), hdg_true)
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
            "heading_true_deg": f"{hdg_true:.4f}" if hdg_true is not None else "",
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
        period = 1.0 / LOG_RATE_HZ
        next_row = time.monotonic()
        next_status = time.monotonic() + 1.0

        while True:
            now = time.monotonic()

            # GNSS: only readings that are new since the last poll
            for kind, d, age in feed.poll(now):
                if kind == "gga":
                    gps.update(d, now - age)
                    if pending is not None:
                        pending["gga"].append(d)
                    q = FIX_LABEL.get(d["fix_q"], d["fix_q"])
                    fix_counts[q] = fix_counts.get(q, 0) + 1
                else:
                    att.update(d, now - age)
                    if pending is not None:
                        pending["hdg"].append(d.get("heading_deg"))

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
                if pending is not None:
                    pending["spd"].append(sp)

            # keyboard
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1).lower()
                if c == "q":
                    break
                if c in ("m", "n") and pending is None:
                    numbered = teach or c == "n"
                    label = (f"plot_{plot_id:03d}" if numbered
                             else f"mark_{len(marks)+1}")
                    pending = {"label": label, "numbered": numbered,
                               "t_end": now + MARK_AVG_S,
                               "gga": [], "hdg": [], "spd": []}
                    sys.stdout.write(f"\r  {label}: hold still, averaging "
                                     f"{MARK_AVG_S:.0f} s ...{' ' * 40}\n")

            # finish a mark once its averaging window has closed
            if pending is not None and now >= pending["t_end"]:
                label = pending["label"]
                fields, why = average_mark(pending["gga"], pending["hdg"],
                                           pending["spd"])
                if fields is None:
                    rejected += 1
                    snapshot(now, "mark_rejected", label)
                    rows += 1
                    sys.stdout.write(
                        f"\r  REFUSED {label}: {why}. "
                        f"Not saved - press m to try again.\a\n")
                else:
                    v = snapshot(now, "mark", label)
                    rows += 1
                    mark = {"t_mono": v["t_mono"], "t_utc": v["t_utc"],
                            "label": label}
                    mark.update(fields)
                    marks.append(mark)
                    if pending["numbered"]:
                        plot_id += 1
                    sys.stdout.write(
                        f"\r  MARK {label}  {fields['lat']:.9f},"
                        f"{fields['lon']:.9f}  hdg {fields['heading_deg']:.1f}"
                        f"  sd {fields['pos_sd_cm']:.1f} cm / "
                        f"{fields['heading_sd_deg']:.2f} deg  "
                        f"n={fields['n_samples']}  (rotation centre)\n")
                pending = None

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
                    f"hdg {true_heading(att.data.get('heading_deg')) or 0:7.2f}  "
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
        feed.close()

    total = time.monotonic() - t0
    summary = {
        "format": MARK_FORMAT,
        "heading_ref": "true",
        "antenna_offset_m": ANTENNA_OFFSET_M,
        "heading_offset_deg": HEADING_OFFSET_DEG,
        "marks_rejected": rejected,
        "started_utc": stamp,
        "duration_s": round(total, 1),
        "rows": rows,
        "rtcm_bytes": feed.rtcm_bytes,
        "gnss_source": feed.name,
        "fix_quality_counts": fix_counts,
        "marks": marks,
        "csv": csv_path,
    }
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n\n  {rows} rows over {total:.0f} s -> {csv_path}")
    print(f"  {len(marks)} marks -> {json_path}"
          + (f"   ({rejected} refused)" if rejected else ""))
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
