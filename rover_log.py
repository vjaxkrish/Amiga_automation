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
    python3 rover_log.py teach              same, but 't' / 'm' mark waypoints
    python3 rover_log.py check              one-shot: are all three sources alive

    python3 teach_gui.py --map "2023 SA NTEP"   the same teach, with the plot
            map on screen: plots turn green as they are marked, and a
            skipped plot can be clicked to come back and teach it

    python3 rover_log.py drive --cols 9 --start-row 1
            teach with plot-map names: plots are labelled R{row}_C{col}
            in the order they are driven - along map rows, serpentine:
            odd rows C1 -> C{cols}, even rows C{cols} -> C1. For a block
            that starts further down, give its first row (--start-row 3).

While running:
    m       mark the current position (teach mode: a plot centre;
            with --cols: a headland point, never captured)
    t       mark and label the next plot (plot_001... or, with --cols,
            the next R{row}_C{col}). Was 'n' until 20 Nov; 'n' now does
            nothing, because it sits next to 'm'.
    s       with --cols: skip the next plot name without marking (a plot
            that is missing or cannot be reached)
    u       undo the last mark or skip (plot names go back to where they were)
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

try:
    import serial
except ImportError:          # a laptop without pyserial: teach_gui --demo
    serial = None

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
    if serial is None:
        return None, "pyserial is not installed (pip install pyserial)"
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


class PlotLabeler:
    """Names plots in the order they are driven.

    Without a grid: plot_001, plot_002, ...
    With a grid (cols): R{row}_C{col}, serpentine along map rows - odd rows
    run C1 -> C{cols}, even rows C{cols} -> C1 - matching the plot map in
    the camera app, so a capture lands in that plot's folder."""

    def __init__(self, cols=None, start_row=1):
        self.cols = cols
        self.start_row = start_row
        self.k = 0

    def rc(self, k=None):
        k = self.k if k is None else k
        row = self.start_row + k // self.cols
        pos = k % self.cols
        col = pos + 1 if row % 2 == 1 else self.cols - pos
        return row, col

    def current(self):
        if not self.cols:
            return f"plot_{self.k + 1:03d}"
        r, c = self.rc()
        return f"R{r}_C{c}"

    def advance(self):
        self.k += 1

    def back(self):
        self.k = max(0, self.k - 1)


class TeachSession:
    """One teach (or drive-log) session, with no screen or keyboard of its
    own. rover_log.py (terminal) and teach_gui.py (window) both drive this
    class, so there is exactly one copy of how a mark is taken, named,
    refused, undone and saved.

        s = TeachSession(feed, can_sock, cols=9, start_row=1)
        loop:  events = s.poll(time.monotonic())     # every few ms
        keys:  s.mark("plot") / s.mark("headland") / s.skip() / s.undo()
        end:   s.close()                              # writes the .json

    poll() returns a list of (kind, text, data) events for the caller to
    show: kind is mark | refused | averaging | skip | undo | info.

    The summary .json is rewritten after every mark, skip and undo, so a
    crash or a flat battery mid-teach loses nothing already marked."""

    def __init__(self, feed, sock, cols=None, start_row=1, teach=True,
                 out_dir=None, stamp=None):
        self.feed, self.sock = feed, sock
        self.teach = teach
        self.grid = bool(cols)
        self.labeler = PlotLabeler(cols, start_row or 1)
        self.out_dir = out_dir or OUT_DIR
        os.makedirs(self.out_dir, exist_ok=True)
        self.stamp = stamp or datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.out_dir, f"rover_{self.stamp}.csv")
        self.json_path = os.path.join(self.out_dir, f"rover_{self.stamp}.json")
        self.gps, self.att, self.can = Source(), Source(), Source()
        self.marks = []
        self.skipped = []          # plot names skipped, in order
        self.history = []          # undo stack: (action, name, k before, extra)
        self.picked = None         # (row, col) chosen out of order, or None
        self.pending = None        # a mark being averaged
        self.rejected = 0
        self.rows = 0
        self.fix_counts = {}
        self.t0 = time.monotonic()
        self.next_row = self.t0
        self.period = 1.0 / LOG_RATE_HZ
        self.csv = open(self.csv_path, "w", buffering=1)
        self.csv.write(",".join(COLUMNS) + "\n")
        self.closed = False

    # -- what the screen shows ------------------------------------------
    def next_plot(self):
        if self.picked:
            return f"R{self.picked[0]}_C{self.picked[1]}"
        return self.labeler.current()

    def next_rc(self):
        if not self.grid:
            return None
        return self.picked or self.labeler.rc()

    def taught_rcs(self):
        return {(m["row"], m["col"]) for m in self.marks
                if m.get("kind") == "plot" and "row" in m}

    def pick(self, row, col):
        """Make (row, col) the plot the next plot mark teaches, out of the
        usual order - to come back for a plot that was skipped. Afterwards
        the order carries on from where it was. Returns None if accepted,
        else the reason. pick(None, None) cancels."""
        if not self.grid:
            return "plot picking needs a plot map (--cols)"
        if self.pending is not None:
            return "a mark is averaging - wait for it"
        if row is None:
            self.picked = None
            return None
        if row < 1 or not 1 <= col <= self.labeler.cols:
            return f"R{row}_C{col} is not in the map"
        if (row, col) in self.taught_rcs():
            return (f"R{row}_C{col} is already taught (u undoes the last "
                    f"mark only)")
        if (row, col) == self.labeler.rc():
            self.picked = None          # that is the normal next plot anyway
            return None
        self.picked = (row, col)
        return None

    def _advance_past_taught(self):
        """Step the order on, over any plot already taught out of order."""
        self.labeler.advance()
        if not self.grid:
            return
        taught = self.taught_rcs()
        for _ in range(10000):
            if self.labeler.rc() not in taught:
                break
            self.labeler.advance()

    def status(self, now):
        g, a, c = self.gps.data, self.att.data, self.can.data
        stale = [nm for nm, src in (("gps", self.gps), ("heading", self.att),
                                    ("can", self.can))
                 if src.age(now) is None or src.age(now) > STALE_WARN_S]
        return {
            "fix": FIX_LABEL.get(g.get("fix_q", ""), "-"),
            "fix_q": g.get("fix_q", ""),
            "nsat": g.get("nsat", ""),
            "corr_age_s": g.get("corr_age_s", ""),
            "heading": true_heading(a.get("heading_deg")),
            "speed": c.get("speed"),
            "stale": stale,
            "averaging": (self.pending["label"], self.pending["t_end"] - now)
                         if self.pending else None,
            "marks": len(self.marks),
            "rows": self.rows,
            "elapsed": now - self.t0,
        }

    # -- actions --------------------------------------------------------
    def mark(self, kind, now=None):
        """kind 'plot' (named R/C or plot_NNN) or 'headland' (mark_N, never
        captured). In plain teach mode without a grid every mark is a plot,
        as before."""
        if self.pending is not None:
            return None
        now = time.monotonic() if now is None else now
        numbered = kind == "plot" or (self.teach and not self.grid)
        label = (self.next_plot() if numbered
                 else f"mark_{len(self.marks) + 1}")
        self.pending = {"label": label, "numbered": numbered,
                        "rc": self.next_rc() if numbered else None,
                        "picked": bool(numbered and self.picked),
                        "t_end": now + MARK_AVG_S,
                        "gga": [], "hdg": [], "spd": []}
        return label

    def skip(self):
        """Skip the next plot in the usual order. With a plot picked, that
        pick is cancelled instead (nothing is skipped)."""
        if self.pending is not None or not self.grid:
            return None
        if self.picked:
            self.picked = None
            return None
        name = self.labeler.current()
        self.history.append(("skip", name, self.labeler.k, None))
        self.skipped.append(name)
        self._advance_past_taught()
        self.save()
        return name

    def undo(self, now=None):
        """Undo the last mark OR skip. The plot names go back to exactly
        where they were before it (a skip after the mark is undone first)."""
        if self.pending is not None or not self.history:
            return None
        now = time.monotonic() if now is None else now
        action, what, k_before, extra = self.history.pop()
        self.labeler.k = k_before
        self.picked = None
        if action == "skip":
            self.skipped.remove(what)
        else:
            gone = self.marks.pop()
            self.snapshot(now, "mark_undone", gone["label"])
            what = gone["label"]
            if extra and extra.get("was_skipped"):
                self.skipped.append(what)     # back to grey, not red
        self.save()
        return action, what

    # -- the loop ---------------------------------------------------------
    def poll(self, now):
        events = []
        for kind, d, age in self.feed.poll(now):
            if kind == "gga":
                self.gps.update(d, now - age)
                if self.pending is not None:
                    self.pending["gga"].append(d)
                q = FIX_LABEL.get(d["fix_q"], d["fix_q"])
                self.fix_counts[q] = self.fix_counts.get(q, 0) + 1
            else:
                self.att.update(d, now - age)
                if self.pending is not None:
                    self.pending["hdg"].append(d.get("heading_deg"))

        while True:
            try:
                raw = self.sock.recv(16)
            except (BlockingIOError, OSError):
                break
            cid, dlc, data = CAN_FRAME.unpack(raw)
            if (cid & 0x1FFFFFFF) != COBID_TPDO1 or dlc < 8:
                continue
            st, sp, ang = struct.unpack_from("<Bhh", data)
            self.can.update({"state": st, "speed": sp, "angular": ang,
                             "soc": data[7]}, now)
            if self.pending is not None:
                self.pending["spd"].append(sp)

        if self.pending is not None and now >= self.pending["t_end"]:
            events.append(self._finish_mark(now))

        if now >= self.next_row:
            self.snapshot(now)
            self.next_row += self.period
            if self.next_row < now:
                self.next_row = now + self.period
        return events

    def _finish_mark(self, now):
        p, self.pending = self.pending, None
        label = p["label"]
        fields, why = average_mark(p["gga"], p["hdg"], p["spd"])
        if fields is None:
            self.rejected += 1
            self.snapshot(now, "mark_rejected", label)
            return ("refused", f"REFUSED {label}: {why}. Not saved - "
                               f"press the same key to try again.",
                    {"label": label, "why": why})
        v = self.snapshot(now, "mark", label)
        mark = {"t_mono": v["t_mono"], "t_utc": v["t_utc"], "label": label,
                "kind": "plot" if p["numbered"] else "headland"}
        if self.grid and p["numbered"]:
            mark["row"], mark["col"] = p["rc"]
        if p["picked"]:
            mark["out_of_order"] = True
        mark.update(fields)
        was_skipped = label in self.skipped
        self.history.append(("mark", label, self.labeler.k,
                             {"was_skipped": was_skipped}))
        if was_skipped:
            self.skipped.remove(label)        # taught after all
        self.marks.append(mark)
        if p["numbered"]:
            if p["picked"]:
                self.picked = None            # back to the usual order
                if self.labeler.rc() in self.taught_rcs():
                    self._advance_past_taught()
            else:
                self._advance_past_taught()
        self.save()
        return ("mark",
                f"MARK {label}  {fields['lat']:.9f},{fields['lon']:.9f}  "
                f"hdg {fields['heading_deg']:.1f}  sd {fields['pos_sd_cm']:.1f}"
                f" cm / {fields['heading_sd_deg']:.2f} deg  "
                f"n={fields['n_samples']}  (rotation centre)", mark)

    def snapshot(self, now, row_type="sample", label=""):
        g, a, c = self.gps.data, self.att.data, self.can.data
        hdg_true = true_heading(a.get("heading_deg"))
        clat, clon = rotation_centre(g.get("lat"), g.get("lon"), hdg_true)
        gage, tage, cage = self.gps.age(now), self.att.age(now), self.can.age(now)
        vals = {
            "t_mono": f"{now - self.t0:.3f}",
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
            "gga_age_s": f"{gage:.2f}" if gage is not None else "",
            "heading_deg": f"{a['heading_deg']:.4f}" if a.get("heading_deg") is not None else "",
            "heading_true_deg": f"{hdg_true:.4f}" if hdg_true is not None else "",
            "pitch_deg": f"{a['pitch_deg']:.4f}" if a.get("pitch_deg") is not None else "",
            "baseline_m": f"{a['baseline_m']:.4f}" if a.get("baseline_m") is not None else "",
            "head_q": a.get("head_q", ""),
            "head_nsat": a.get("head_nsat", ""),
            "tar_age_s": f"{tage:.2f}" if tage is not None else "",
            "can_state": c.get("state", ""),
            "speed_mm_s": c.get("speed", ""),
            "angular_mrad_s": c.get("angular", ""),
            "soc_pct": c.get("soc", ""),
            "can_age_s": f"{cage:.2f}" if cage is not None else "",
        }
        self.csv.write(",".join(str(vals[k]) for k in COLUMNS) + "\n")
        self.rows += 1
        return vals

    # -- saving -----------------------------------------------------------
    def summary(self):
        lab = self.labeler
        return {
            "format": MARK_FORMAT,
            "heading_ref": "true",
            "antenna_offset_m": ANTENNA_OFFSET_M,
            "heading_offset_deg": HEADING_OFFSET_DEG,
            "marks_rejected": self.rejected,
            "grid": ({"cols": lab.cols, "start_row": lab.start_row,
                      "order": "serpentine along map rows",
                      "skipped": list(self.skipped)} if self.grid else None),
            "started_utc": self.stamp,
            "duration_s": round(time.monotonic() - self.t0, 1),
            "rows": self.rows,
            "rtcm_bytes": self.feed.rtcm_bytes,
            "gnss_source": self.feed.name,
            "fix_quality_counts": self.fix_counts,
            "marks": self.marks,
            "csv": self.csv_path,
        }

    def save(self):
        """Write the summary atomically: a crash mid-write leaves the
        previous version, never half a file."""
        tmp = self.json_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.summary(), f, indent=2)
        os.replace(tmp, self.json_path)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.save()
        for h in (self.csv, self.sock, self.feed):
            try:
                h.close()
            except Exception:
                pass


def open_sources():
    """(feed, can_sock, None) or (None, None, reason)."""
    feed, why = open_feed()
    if feed is None:
        return None, None, f"{why} - run 'check'"
    try:
        sock = open_can()
    except OSError as e:
        feed.close()
        return None, None, f"cannot open {CAN_IFACE}: {e}"
    return feed, sock, None


def cmd_log(args):
    teach = args.command == "teach"
    feed, sock, why = open_sources()
    if feed is None:
        print(why)
        return 1
    s = TeachSession(feed, sock, cols=getattr(args, "cols", None),
                     start_row=getattr(args, "start_row", 1), teach=teach)
    grid = s.grid

    print(f"logging to {s.csv_path}")
    if grid:
        print(f"  plot names from the map grid, {args.cols} columns, starting "
              f"at row {s.labeler.start_row}: first plot is {s.next_plot()}")
        print("  t = plot   m = headland point   s = skip a plot   "
              "u = undo   q = save")
    else:
        print("  m = mark    t = mark as next plot id    u = undo    q = stop")
    if feed.name == "rtcm_forward":
        print("  GNSS via rtcm_forward (the service keeps forwarding corrections)")
    else:
        print("  GNSS via the ports directly - rtcm_forward is not running")
    print("  (teach_gui.py does the same with a plot map on screen)\n")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    pad = " " * 40
    try:
        tty.setcbreak(fd)
        next_status = time.monotonic() + 1.0
        while True:
            now = time.monotonic()
            for kind, text, _ in s.poll(now):
                bell = "\a" if kind == "refused" else ""
                sys.stdout.write(f"\r  {text}{bell}{pad}\n")
                if kind == "mark" and grid:
                    sys.stdout.write(f"  next plot: {s.next_plot()}\n")

            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1).lower()
                if c == "q":
                    break
                if c == "n":
                    sys.stdout.write(f"\r  'n' does nothing now - plots are "
                                     f"'t' (n sits next to m){pad}\n")
                elif c in ("m", "t") and s.pending is None:
                    label = s.mark("plot" if c == "t" else "headland", now)
                    sys.stdout.write(f"\r  {label}: hold still, averaging "
                                     f"{MARK_AVG_S:.0f} s ...{pad}\n")
                elif c == "s" and grid and s.pending is None:
                    name = s.skip()
                    sys.stdout.write(f"\r  skipped {name} (not marked){pad}\n")
                    sys.stdout.write(f"  next plot: {s.next_plot()}\n")
                elif c == "u" and s.pending is None:
                    r = s.undo(now)
                    if r is None:
                        sys.stdout.write(f"\r  nothing to undo{pad}\n")
                    else:
                        sys.stdout.write(f"\r  UNDONE {r[0]} {r[1]}{pad}\n")
                        if grid:
                            sys.stdout.write(f"  next plot: {s.next_plot()}\n")

            if now >= next_status:
                next_status = now + 1.0
                st = s.status(now)
                warn = ("  STALE: " + ",".join(st["stale"])) if st["stale"] else ""
                sys.stdout.write(
                    f"\r  {st['elapsed']:7.1f}s  {st['fix']:<11} sats "
                    f"{st['nsat'] or '--':>2}  hdg {st['heading'] or 0:7.2f}  "
                    f"spd {st['speed'] if st['speed'] is not None else '-':>5} mm/s  "
                    f"rows {st['rows']:>6}  marks {st['marks']:>3}{warn}   ")
                sys.stdout.flush()
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        s.close()

    total = time.monotonic() - s.t0
    print(f"\n\n  {s.rows} rows over {total:.0f} s -> {s.csv_path}")
    print(f"  {len(s.marks)} marks -> {s.json_path}"
          + (f"   ({s.rejected} refused)" if s.rejected else ""))
    if s.fix_counts:
        tot = sum(s.fix_counts.values())
        parts = [f"{k} {100*v/tot:.0f}%" for k, v in sorted(s.fix_counts.items())]
        print(f"  fix quality: {', '.join(parts)}")
    return 0


COMMANDS = {"drive": cmd_log, "teach": cmd_log, "check": cmd_check}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=COMMANDS)
    p.add_argument("--cols", type=int, default=None,
                   help="plot-map columns: name plots R{row}_C{col}, serpentine")
    p.add_argument("--start-row", type=int, default=1,
                   help="with --cols: the map row this teach starts on")
    args = p.parse_args()
    if args.cols is not None and args.cols < 1:
        p.error("--cols must be at least 1")
    return COMMANDS[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
