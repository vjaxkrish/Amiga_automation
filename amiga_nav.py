#!/usr/bin/env python3
"""
amiga_nav.py - Phase 0 step 6. Pure pursuit path following and waypoint drive.

    replay   <csv>              simulate the controller against a logged path
    follow   <csv>              drive the path recorded in that log
    goto     <lat> <lon>        drive to one point and stop
    mission  <json>             drive the marks in a rover_log teach summary
    plan     <json>             print what mission would do. No hardware.
    yaw-test                    smallest turn command obeyed while rolling

`replay` needs no hardware and no robot. It starts a simulated vehicle
offset from the path and reports whether the controller converges and what
cross-track error it holds. Run that before anything drives.

ARM AUTO ON THE DASHBOARD SCREEN before any live command. The Pi owns the
state machine and velocity only after that.
"""

import argparse
import csv as csvmod
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

# ============================================================================
#  CALIBRATION - verify these on the machine before trusting any of it
# ============================================================================

# HEADING_OFFSET_DEG and ANTENNA_OFFSET_M live in vehicle.py, shared with
# rover_log.py. They used to be defined separately in each file and drifted
# apart (0.65 here, 0.794 there), and rover_log never applied the heading
# offset at all. See vehicle.py and docs/calibration.md.
from vehicle import (ANTENNA_OFFSET_M, HEADING_OFFSET_DEG, EARTH_R,
                     true_heading, rotation_centre, metres_between)

# Sign of the angular rate field in AmigaRpdo1. Verified on the wire
# 2026-10-27: positive mrad/s turns the vehicle left (counter-clockwise seen
# from above).
ANGULAR_SIGN = +1

# Measured on turf, 2026-10-27: below ~40 mm/s the machine does not break
# static friction. At 40 mm/s it stops immediately on a zero command, so no
# deceleration overshoot to compensate for down here.
CREEP_SPEED_MM_S = 40

# Measured on turf, 2026-10-27: below roughly 150-200 mrad/s the machine does
# not rotate at all. Any non-zero angular command is raised to this magnitude,
# because a command below it is not a gentle correction, it is no correction.
ANGULAR_MIN_MRAD_S = 220

# The 220 floor was measured turning IN PLACE. Whether the same floor applies
# while rolling is NOT measured: a rolling skid-steer usually yaws on much
# smaller commands. Applying 220 at 500 mm/s turns a 40 mrad/s correction
# into a 5x over-steer, which is the likely cause of the 8 deg angled
# approach seen at plot_001 on 2026-11-12. Measure with `yaw-test`, then set
# this. Until then it equals the in-place floor - behaviour unchanged.
ANGULAR_MIN_ROLLING_MRAD_S = 220
ROLLING_ABOVE_MM_S = 100       # |linear| above this counts as rolling

# yaw-test 2026-11-12, wheel rate (0x18E) against GPS heading at ~300 mm/s
# on turf: the wheels follow the command, but only ~10-25% of it becomes
# rotation, about the same either way - the rest is scrub. Pure pursuit
# asks for the rotation it NEEDS, so while rolling that is scaled up by
# STEER_YAW_GAIN, and capped below the rate at which the inside wheels stop
# and the machine pivots instead (seen at 600 mrad/s at 300 mm/s).
# Both effectively OFF by default: with steering at the mark, the 220 mrad/s
# floor in Rover.send() already commands far more than pure pursuit asks
# for, and scaling it up changed nothing in simulation. Kept for row-line
# following, where they mattered.
STEER_YAW_GAIN = 1.0
# Row-line lookahead. Must be well beyond how far the machine rolls before a
# turn takes effect (~0.3 s delay + ~1 s ramp at 500 mm/s = ~0.6 m): a 0.6 m
# lookahead oscillated in simulation and overshot the line by 27 cm.
ROW_LOOKAHEAD_M = 2.0
TURN_PER_SPEED_MAX = 99.0      # rolling cap, |mrad/s| per mm/s (1.5 avoids
                               # the inner-wheel stop seen at 600 / 300)

# ============================================================================
#  Tuning
# ============================================================================

CRUISE_SPEED_MM_S = 500        # between waypoints
MAX_SPEED_MM_S = 800           # hard clamp in code
MAX_ANGULAR_MRAD_S = 600

LOOKAHEAD_M = 1.5              # pure pursuit lookahead
LOOKAHEAD_MIN_M = 0.6
LOOKAHEAD_GAIN = 1.2           # lookahead = gain * speed_m_s, within bounds

# The bearing to a nearby target is dominated by RTK noise: at 0.4 m, 2 cm of
# jitter swings it 2.9 deg. Pure pursuit then chases noise and the heading
# diverges exactly where it should be settling. Observed 2026-10-27: heading
# error went -0.6 -> +10.1 deg over the last 70 cm, and creep locked the bad
# bearing. So the approach line is fixed while the bearing is still reliable,
# and the machine drives straight from there.
# Locking the line at 2 m meant driving 2 m with no lateral feedback, and
# the machine crabs: measured 2026-10-27, cross-track grew 0 -> 8 cm over
# that run with heading holding within 1 deg of the locked bearing. Skid
# steer on turf tracks a slow curve on equal commands. So correction stays
# on until the last 30 cm, with a deadband so it is not chasing RTK noise.
BEARING_LOCK_M = 1.2           # stop re-aiming inside this distance
# Was 0.03. Every plot on 2026-11-12 settled at +2.7 to +2.8 cm cross - just
# under the old deadband, so it was never corrected.
CROSS_CORRECT_M = 0.015        # steer back only when further off the line

# In-row legs: when a leg's approach is within IN_ROW_DEG of the mark's
# taught heading, the machine follows the ROW LINE - the line through the
# mark along its taught heading - for the whole leg, steering to a point
# ahead on that line (extended past the mark). It converges onto the line
# AND parallel to it, so it arrives square and the final turn is ~1 deg.
# Steering at the target point instead (the old way) arrives angled: on
# 2026-11-12 plot_001 came in 8 deg off and the final turn moved it 4.4 cm.
# OFF by default. On turf only ~10-25% of the wheels' turn becomes rotation,
# with ~1 s lag and stick-slip (yaw-tests 2026-11-12). Following the line
# needs turning authority the machine does not have: in simulation it ended
# 4-6 cm off sideways on the 12 Nov route, where steering at the mark itself
# (the default) stayed under 2 cm - at 12%, 18% and 25% alike.
ROW_LINE_FOLLOW = False
IN_ROW_DEG = 20.0
ROW_LINE_MAX_CROSS_M = 0.5     # further off the row line than this at the
                               # start of a leg: drive to the point instead
# In-row: also steer when angled more than this. Not smaller: on turf the
# machine crabs sideways ~4% of forward speed (2026-10-27), and holding a
# steady 2-3 deg angle is what cancels that - including through the
# unsteered creep. Straightening it fully would just let the crab win.
HEADING_CORRECT_DEG = 4.0
ALONG_FILTER_N = 3             # arrival / fix decisions use the mean of the
                               # last N distinct position readings
STEER_MIN_SPEED_MM_S = 0       # steer at any speed above creep. A 150 mm/s
                               # gate was tried; on turf it let the last half
                               # metre drift and gained nothing in simulation

# Turning in place on turf MOVES the machine: 12 and 35 cm for two ~95 deg
# pendant pivots, 45 cm during an 88 deg mission turn (2026-11-12). After a
# final turn, re-check the along-row error and creep straight forward or
# back to fix it. Straight moves do not scuff the turf; sideways error
# cannot be fixed without turning, so it is reported, not chased.
POST_TURN_FIX_M = 0.015        # fix along-row error larger than this
POST_TURN_FIX_MAX_M = 0.25     # ...but not more - that is a fault, not slip
POST_TURN_FIX_TRIES = 2
CHECK_SETTLE_S = 0.5           # stand still this long before judging, and
                               # judge on the MEAN of the readings in it -
                               # one RTK reading is +/- 1 cm on its own

# Speed is RAMPED from cruise to creep across the approach, not switched at
# the creep boundary. The dashboard owns the deceleration and it is not
# instant: measured 2026-10-27, a 500 -> 40 mm/s step took about 0.5 s and
# 20 cm, so the machine was still doing 490 mm/s well inside the creep zone
# and only the last few centimetres were actually slow.
DECEL_START_M = 1.2            # begin shedding speed here
APPROACH_SETTLE_S = 0.6        # hold creep speed this long before accepting
                               # arrival, so the stop is from a slow roll
CREEP_DISTANCE_M = 0.30        # switch to creep speed inside this
ARRIVE_RADIUS_M = 0.01         # stop when the along-track remainder is under this
FINAL_HEADING_TOL_DEG = 1.5    # accept the final heading inside this
TURN_COARSE_DEG = 8.0          # above this, turn continuously; below, pulse
PULSE_MIN_S = 0.10             # shortest rotation pulse that still moves
PULSE_MAX_S = 0.45
PULSE_REST_S = 0.35            # settle and re-measure between pulses
TURN_SLOW_MRAD_S = 260         # rate for the final heading turn - above the
                               # rotational breakaway, not below it
# Steering during creep is OFF, deliberately. The machine will not rotate
# below roughly 220 mrad/s, so the smallest correction available is 12 deg/s -
# which against a 3 deg error is not a correction, it is a lurch. The result
# is bang-bang oscillation that grows rather than settles.
#
# Over a 30 cm creep, 5 deg of uncorrected heading error puts the machine
# 2.6 cm to one side. That is inside RTK noise, so there is nothing worth
# correcting and no way to correct it gently if there were.
CREEP_STEER = False
CREEP_STEER_GAIN = 25.0        # only used if CREEP_STEER is turned back on
CREEP_STEER_MAX = 300
CREEP_STEER_DEADBAND_DEG = 6.0
CREEP_ABORT_HEADING_DEG = 25   # creep steering has no target feedback, so a
CREEP_ABORT_OVERSHOOT_M = 0.5  # fault integrates - stop rather than wander
HEADING_ALIGN_DEG = 25         # turn in place if the target is further off than this
TURN_IN_PLACE_MRAD_S = 350     # rate used while aligning, zero linear speed
ALIGN_ASSIST_MM_S = 0          # small forward speed during align if it stalls
ALIGN_STALL_S = 12.0           # give up aligning after this with no progress
HEADING_RESUME_DEG = 8         # hysteresis: stop aligning once inside this

DWELL_S = 5.0                  # stop time at each mission waypoint
CONTROL_HZ = 20.0
MAX_START_DISTANCE_M = 20.0    # refuse a mission whose first point is further

# Safety gates
VERBOSE = True                 # one line per state change and 4 Hz otherwise
REQUIRE_RTK_FIXED = True
MAX_DATA_AGE_S = 0.5           # position or heading older than this is a GPS fault
MAX_CORR_AGE_S = 5.0           # corrections older than this is a GPS fault

# GPS faults HOLD rather than abort. The machine stops where it is, keeps
# AUTO_ACTIVE alive with zero commands (so neither the watchdog nor the
# operator has to intervene), and carries on when the fix is back. Ending
# the run on the first float meant a two-second multipath blip ended a
# 100-plot session. Only after GPS_HOLD_MAX_S does it give up.
GPS_HOLD_MAX_S = 90.0          # give up waiting for the fix after this
# A fix that has just come back is not trusted at once - a freshly resolved
# ambiguity can be wrong, and it shows as a jump that settles. Require it to
# hold this long before moving again.
GPS_RESUME_GOOD_S = 2.0

# ============================================================================
#  Hardware
# ============================================================================

BYID = "/dev/serial/by-id"
LG580P_FRAG = "usb-1a86_USB_Dual_Serial"
RADIO_FRAG = "DU0E5OQL"
CAN_IFACE = "can0"

LG580P_BAUD = 460800
RADIO_BAUD = 57600

COBID_RPDO1 = 0x20E
COBID_TPDO1 = 0x18E
COBID_HEARTBEAT = 0x70E
CAN_FRAME = struct.Struct("<IB3x8s")

STATE_AUTO_READY = 4
STATE_AUTO_ACTIVE = 5


# ============================================================================
#  Geometry
# ============================================================================

class Frame:
    """Local tangent plane. Small areas only - good to mm over a plot."""

    def __init__(self, lat0, lon0):
        self.lat0 = lat0
        self.lon0 = lon0
        self.k = math.cos(math.radians(lat0))

    def to_xy(self, lat, lon):
        x = math.radians(lon - self.lon0) * EARTH_R * self.k
        y = math.radians(lat - self.lat0) * EARTH_R
        return x, y

    def to_ll(self, x, y):
        lat = self.lat0 + math.degrees(y / EARTH_R)
        lon = self.lon0 + math.degrees(x / (EARTH_R * self.k))
        return lat, lon


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def bearing_to(dx, dy):
    """Bearing from north, clockwise, of the vector (east, north)."""
    return math.degrees(math.atan2(dx, dy)) % 360.0


def control_point(frame, lat, lon, heading_deg):
    """Vehicle rotation centre in local metres, from the antenna fix."""
    x, y = frame.to_xy(lat, lon)
    th = math.radians(heading_deg)
    return (x - ANTENNA_OFFSET_M * math.sin(th),
            y - ANTENNA_OFFSET_M * math.cos(th))


def vehicle_frame(dx, dy, heading_deg):
    """Return (forward, left) components of a vector in the vehicle frame."""
    th = math.radians(heading_deg)
    fwd = dx * math.sin(th) + dy * math.cos(th)
    left = -dx * math.cos(th) + dy * math.sin(th)
    return fwd, left


# ============================================================================
#  Path
# ============================================================================

def load_path_csv(path, min_spacing=0.25):
    """Logged track -> list of (lat, lon), thinned to min_spacing metres."""
    pts = []
    with open(path) as fh:
        for r in csvmod.DictReader(fh):
            if r.get("row_type") != "sample" or not r.get("lat"):
                continue
            try:
                pts.append((float(r["lat"]), float(r["lon"])))
            except ValueError:
                continue
    if not pts:
        raise SystemExit(f"no positions in {path}")

    frame = Frame(pts[0][0], pts[0][1])
    out = [pts[0]]
    lx, ly = frame.to_xy(*pts[0])
    for la, lo in pts[1:]:
        x, y = frame.to_xy(la, lo)
        if math.hypot(x - lx, y - ly) >= min_spacing:
            out.append((la, lo))
            lx, ly = x, y
    return out


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_marks_json(path, verbose=True):
    """Teach summary -> list of marks, each a dict:
        label, lat, lon     rotation centre to drive to
        heading             TRUE vehicle heading when taught, or None
        fix_q, pos_sd_cm    teach quality where recorded

    The rotation centre is RECOMPUTED from the stored antenna fix and heading
    using the current vehicle.py constants, rather than trusted from the
    file. That makes the mark file independent of whatever offset the logger
    happened to use, and it repairs files taught before 2026-11-11, which
    applied the antenna offset along the RAW receiver heading - 180 deg
    wrong - and so stored a centre 1.6 m in front of the real one.

    Format 1 (no "format" key): heading_deg is the raw receiver value.
    Format 2: heading_deg is the true vehicle heading."""
    with open(path) as fh:
        data = json.load(fh)
    fmt = int(data.get("format", 1))
    out = []
    moved = []
    for m in data.get("marks", []):
        ant_lat, ant_lon = _f(m.get("ant_lat")), _f(m.get("ant_lon"))
        hdg = _f(m.get("heading_deg"))
        if hdg is not None and fmt < 2:
            hdg = true_heading(hdg)
        if ant_lat is not None and ant_lon is not None and hdg is not None:
            lat, lon = rotation_centre(ant_lat, ant_lon, hdg)
        else:
            lat, lon = _f(m.get("lat")), _f(m.get("lon"))
        if lat is None or lon is None:
            continue
        old_lat, old_lon = _f(m.get("lat")), _f(m.get("lon"))
        if old_lat is not None and old_lon is not None:
            d = metres_between(old_lat, old_lon, lat, lon)
            if d > 0.01:
                moved.append((m.get("label"), d))
        out.append({"label": m["label"], "lat": lat, "lon": lon,
                    "heading": hdg, "fix_q": str(m.get("fix_q", "")),
                    "pos_sd_cm": _f(m.get("pos_sd_cm"))})
    if not out:
        raise SystemExit(f"no marks in {path}")
    if verbose and moved:
        worst = max(d for _, d in moved)
        print(f"note: {len(moved)} of {len(out)} marks re-derived from the "
              f"antenna fix with the current vehicle.py constants "
              f"(moved up to {worst*100:.0f} cm from the stored centre).")
        if fmt < 2:
            print("      This file predates the 2026-11-11 heading fix; the "
                  "stored centres were wrong and are not used.")
    bad = [m["label"] for m in out if m["fix_q"] and m["fix_q"] != "4"]
    if verbose and bad:
        print(f"WARNING: taught without RTK fixed: {', '.join(bad)}. "
              f"Re-teach these.")
    return out


# ============================================================================
#  Pure pursuit
# ============================================================================

class PurePursuit:
    def __init__(self, frame, pts, loop=False):
        self.frame = frame
        self.xy = [frame.to_xy(la, lo) for la, lo in pts]
        self.loop = loop
        self.idx = 0

    def target(self, x, y, lookahead):
        """Lookahead point ahead of the nearest path point, and progress."""
        # advance the index to the nearest point, never backwards
        best, bi = None, self.idx
        for i in range(self.idx, min(self.idx + 60, len(self.xy))):
            px, py = self.xy[i]
            d = math.hypot(px - x, py - y)
            if best is None or d < best:
                best, bi = d, i
        self.idx = bi

        acc = 0.0
        i = bi
        while i + 1 < len(self.xy) and acc < lookahead:
            ax, ay = self.xy[i]
            bx, by = self.xy[i + 1]
            acc += math.hypot(bx - ax, by - ay)
            i += 1
        return self.xy[i], best, (i >= len(self.xy) - 1)

    def remaining(self, x, y):
        d = 0.0
        for i in range(self.idx, len(self.xy) - 1):
            ax, ay = self.xy[i]
            bx, by = self.xy[i + 1]
            d += math.hypot(bx - ax, by - ay)
        return d


def ramp_speed(remaining_m):
    """Cruise far out, creep close in, linear between. Commanding the step
    change directly does not work: the dashboard's own deceleration limit
    means the machine arrives at the creep zone still at cruise speed."""
    if remaining_m >= DECEL_START_M:
        return CRUISE_SPEED_MM_S
    if remaining_m <= CREEP_DISTANCE_M:
        return CREEP_SPEED_MM_S
    f = (remaining_m - CREEP_DISTANCE_M) / (DECEL_START_M - CREEP_DISTANCE_M)
    return int(CREEP_SPEED_MM_S + f * (CRUISE_SPEED_MM_S - CREEP_SPEED_MM_S))


def rolling_turn(mrad, speed_mm_s):
    """What to command so the machine actually yaws by about `mrad` while
    rolling at `speed_mm_s`: scale up for scrub, cap below a pivot."""
    m = mrad * STEER_YAW_GAIN
    cap = TURN_PER_SPEED_MAX * abs(speed_mm_s)
    return max(-cap, min(cap, m))


def steer(x, y, heading_deg, tx, ty, speed_mm_s):
    """Pure pursuit curvature -> angular rate in mrad/s."""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 0.0, 0.0, 0.0
    fwd, left = vehicle_frame(dx, dy, heading_deg)
    curvature = 2.0 * left / (dist * dist)          # 1/m, positive = left
    v = speed_mm_s / 1000.0
    omega = curvature * v                            # rad/s
    mrad = ANGULAR_SIGN * omega * 1000.0
    mrad = max(-MAX_ANGULAR_MRAD_S, min(MAX_ANGULAR_MRAD_S, mrad))
    bearing_err = wrap180(bearing_to(dx, dy) - heading_deg)
    return mrad, dist, bearing_err



class AlignGate:
    """Hysteresis around turn-in-place, so the controller does not chatter
    between driving and aligning at the threshold."""

    def __init__(self):
        self.aligning = False

    def apply(self, berr, speed, mrad):
        if self.aligning:
            if abs(berr) < HEADING_RESUME_DEG:
                self.aligning = False
        elif abs(berr) > HEADING_ALIGN_DEG:
            self.aligning = True
        if self.aligning:
            return 0, ANGULAR_SIGN * math.copysign(TURN_IN_PLACE_MRAD_S, berr)
        return speed, mrad


# ============================================================================
#  replay - offline, no hardware
# ============================================================================

def cmd_replay(args):
    pts = load_path_csv(args.path)
    frame = Frame(pts[0][0], pts[0][1])
    pp = PurePursuit(frame, pts)

    # start offset from the path so convergence is actually tested
    sx, sy = frame.to_xy(*pts[0])
    x = sx + args.offset
    y = sy
    nx, ny = frame.to_xy(*pts[min(5, len(pts) - 1)])
    heading = bearing_to(nx - sx, ny - sy)

    dt = 1.0 / CONTROL_HZ
    t = 0.0
    errs = []
    print(f"path: {len(pts)} points, {pp.remaining(sx, sy):.1f} m")
    print(f"start offset {args.offset:+.1f} m lateral, heading {heading:.1f} deg\n")
    print(f"{'t':>7} {'speed':>7} {'omega':>8} {'x-track':>9} {'to go':>8}")

    gate = AlignGate()
    last_print = -1.0
    while t < args.max_time:
        speed = CRUISE_SPEED_MM_S
        la = max(LOOKAHEAD_MIN_M,
                 min(LOOKAHEAD_M, LOOKAHEAD_GAIN * speed / 1000.0))
        (tx, ty), xtrack, at_end = pp.target(x, y, la)
        togo = pp.remaining(x, y)
        if at_end and togo < ARRIVE_RADIUS_M:
            print(f"\nreached the end at t={t:.1f} s")
            break

        if togo < CREEP_DISTANCE_M:
            speed = CREEP_SPEED_MM_S

        mrad, dist, berr = steer(x, y, heading, tx, ty, speed)

        # Pure pursuit curvature goes to zero at 180 deg of bearing error, so
        # a target behind the vehicle produces no steering at all and it
        # drives away forever. Turn in place instead.
        speed, mrad = gate.apply(berr, speed, mrad)

        # kinematic vehicle model
        v = speed / 1000.0
        omega = (mrad * ANGULAR_SIGN) / 1000.0
        heading = (heading + math.degrees(omega) * dt) % 360.0
        th = math.radians(heading)
        x += v * math.sin(th) * dt
        y += v * math.cos(th) * dt
        t += dt
        errs.append(xtrack)

        if t - last_print >= 2.0:
            last_print = t
            print(f"{t:7.1f} {speed:7.0f} {mrad:8.1f} {xtrack:9.3f} {togo:8.1f}")

    if errs:
        settled = errs[len(errs) // 4:]
        print(f"\ncross-track error: initial {errs[0]:.2f} m, "
              f"max after settling {max(settled):.3f} m, "
              f"mean after settling {sum(settled)/len(settled):.3f} m")
        if max(settled) > 0.5:
            print("NOT CONVERGING - check ANGULAR_SIGN and LOOKAHEAD_M")
        else:
            print("converged")
    return 0


# ============================================================================
#  Live plumbing
# ============================================================================

STATE_PATH = "/dev/shm/rover_state.json"


def read_state():
    """Position and heading as republished by rtcm_forward.service, or None.

    Only one process may read a serial port: two readers each get a fraction
    of the bytes and both starve. The forwarder owns the radio and the
    receiver; everything else reads this."""
    try:
        with open(STATE_PATH) as f:
            st = json.load(f)
        if time.time() - st["t_wall"] < 2.0:
            return st
    except (OSError, ValueError, KeyError):
        pass
    return None


def forwarder_running():
    return read_state() is not None


def find_port(frag):
    try:
        for n in sorted(os.listdir(BYID)):
            if frag in n:
                return os.path.join(BYID, n)
    except FileNotFoundError:
        pass
    return None


def nmea_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, ck = line[1:].partition("*")
    try:
        want = int(ck[:2], 16)
    except ValueError:
        return False
    g = 0
    for c in body:
        g ^= ord(c)
    return g == want


def dm_to_deg(v, h):
    if not v:
        return None
    try:
        f = float(v)
    except ValueError:
        return None
    d = int(f // 100)
    out = d + (f - d * 100) / 60.0
    return -out if h in ("S", "W") else out


class Rover:
    """Position, heading and CAN feedback, with ages."""

    def __init__(self):
        self.lat = self.lon = None
        self.fix_q = "0"
        self.corr_age = None
        self.t_pos = None
        self.heading = None
        self.t_head = None
        self.speed = 0
        self.state = None
        self.t_can = None
        self.sock = None
        self.angular_floor = True     # yaw-test turns it off
        self.lg = None
        self.radio = None
        self.from_state = False
        self._buf = b""

    def open(self):
        # rtcm_forward.service owns BOTH serial ports when it is running and
        # republishes position and heading. Opening them here as well would
        # split every byte stream two ways.
        self.from_state = forwarder_running()
        if self.from_state:
            print("reading position and heading from rtcm_forward")
            self.lg = self.radio = None
        else:
            lgp = find_port(LG580P_FRAG)
            rdp = find_port(RADIO_FRAG)
            if not lgp or not rdp:
                raise SystemExit("LG580P or radio not found")
            print("note: rtcm_forward not running, using the ports directly")
            self.lg = serial.Serial(lgp, LG580P_BAUD, timeout=0)
            self.radio = serial.Serial(rdp, RADIO_BAUD, timeout=0)
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((CAN_IFACE,))
        self.sock.setblocking(False)

    def close(self):
        for h in (self.lg, self.radio, self.sock):
            if h is None:
                continue
            try:
                h.close()
            except Exception:
                pass

    def poll(self, now):
        if self.from_state:
            st = read_state()
            if st and st.get("lat") is not None:
                self.lat, self.lon = st["lat"], st["lon"]
                self.fix_q = st.get("fix_q", "0")
                try:
                    self.corr_age = float(st.get("corr_age_s") or "")
                except ValueError:
                    self.corr_age = None
                age = max(0.0, time.time() - st["t_wall"])
                self.t_pos = now - age
                if st.get("heading_deg") is not None:
                    self.heading = true_heading(st["heading_deg"])
                    self.t_head = now - age
            self._poll_can(now)
            return

        if self.radio is not None:
            air = self.radio.read(2048)
            if air:
                self.lg.write(air)

        self._buf += self.lg.read(4096)
        while b"\n" in self._buf:
            raw, self._buf = self._buf.split(b"\n", 1)
            s = raw.decode("ascii", "ignore").strip()
            if not nmea_ok(s):
                continue
            f = s.split(",")
            if s.startswith("$GNGGA") and len(f) > 9:
                la = dm_to_deg(f[2], f[3])
                lo = dm_to_deg(f[4], f[5])
                if la is not None and lo is not None:
                    self.lat, self.lon = la, lo
                    self.fix_q = f[6]
                    self.t_pos = now
                    try:
                        self.corr_age = float(f[13]) if len(f) > 13 and f[13] else None
                    except ValueError:
                        self.corr_age = None
            elif s.startswith("$PQTMTAR") and len(f) > 8:
                try:
                    h = float(f[8])
                except (ValueError, IndexError):
                    continue
                self.heading = true_heading(h)
                self.t_head = now

        self._poll_can(now)

    def _poll_can(self, now):
        while True:
            try:
                raw = self.sock.recv(16)
            except (BlockingIOError, OSError):
                break
            cid, dlc, data = CAN_FRAME.unpack(raw)
            if (cid & 0x1FFFFFFF) == COBID_TPDO1 and dlc >= 8:
                st, sp, _ = struct.unpack_from("<Bhh", data)
                self.state, self.speed, self.t_can = st, sp, now

    def send(self, state, lin, ang):
        lin = int(max(-MAX_SPEED_MM_S, min(MAX_SPEED_MM_S, lin)))
        # A non-zero angular command below the rotational breakaway produces
        # no rotation at all, so raise it rather than sending something the
        # machine will ignore.
        if self.angular_floor:
            floor = (ANGULAR_MIN_ROLLING_MRAD_S if abs(lin) > ROLLING_ABOVE_MM_S
                     else ANGULAR_MIN_MRAD_S)
            if ang and abs(ang) < floor:
                ang = math.copysign(floor, ang)
        ang = int(max(-MAX_ANGULAR_MRAD_S, min(MAX_ANGULAR_MRAD_S, ang)))
        self.sock.send(CAN_FRAME.pack(
            COBID_RPDO1, 8, struct.pack("<Bhh3x", state, lin, ang)))

    def gps_fault(self, now):
        """A reason the POSITION cannot be trusted right now, or None.

        These are recoverable: a multipath blip, a dropped radio packet, the
        receiver briefly falling to float. drive_loop holds the machine still
        and waits for them to clear rather than ending the run."""
        if self.t_pos is None or now - self.t_pos > MAX_DATA_AGE_S:
            return "position stale"
        if self.t_head is None or now - self.t_head > MAX_DATA_AGE_S:
            return "heading stale"
        if REQUIRE_RTK_FIXED and self.fix_q != "4":
            return f"fix quality {self.fix_q}, want 4"
        if self.corr_age is not None and self.corr_age > MAX_CORR_AGE_S:
            return f"corrections {self.corr_age:.1f} s old"
        return None

    def hard_fault(self, now):
        """A reason the MACHINE cannot be commanded, or None. Not waited out:
        CAN gone or the Dashboard out of auto needs a person."""
        if self.t_can is None or now - self.t_can > MAX_DATA_AGE_S:
            return "CAN stale"
        if self.state not in (STATE_AUTO_READY, STATE_AUTO_ACTIVE):
            return f"dashboard state {self.state}, not auto"
        return None

    def unsafe(self, now):
        """Return a reason to stop, or None."""
        return self.hard_fault(now) or self.gps_fault(now)


def wait_ready(rover, timeout=600.0):
    print("waiting for position, heading, CAN and RTK fix ...")
    end = time.monotonic() + timeout
    t0 = time.monotonic()
    last = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        rover.poll(now)
        why = rover.unsafe(now)
        if why is not None and now - last >= 5.0:
            last = now
            print(f"  {now - t0:5.0f}s  {why}")
        if why is None:
            print(f"ready: {rover.lat:.8f},{rover.lon:.8f} "
                  f"heading {rover.heading:.1f}\n")
            return True
        time.sleep(0.02)
    print(f"NOT READY: {why}")
    print("arm auto on the Dashboard screen if the state is not 4 or 5.")
    return False


class Keyboard:
    """Non-blocking 'q' to stop. Does nothing when stdin is not a terminal
    (a test harness, or a run started from a script)."""

    def __init__(self):
        self.fd = None
        self.old = None
        try:
            if sys.stdin.isatty():
                self.fd = sys.stdin.fileno()
                self.old = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
        except (termios.error, ValueError, OSError):
            self.fd = None

    def quit_pressed(self):
        if self.fd is None:
            return False
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower() == "q"
        return False

    def restore(self):
        if self.fd is not None and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class GpsHold:
    """Tracks a GPS-fault hold: when it started, why, and whether the fix
    has been good for long enough to move again."""

    def __init__(self):
        self.since = None          # monotonic time the hold began
        self.reason = None
        self.good_since = None     # fix good again since this time
        self.count = 0             # holds in this leg
        self.total_s = 0.0         # time spent holding in this leg

    @property
    def active(self):
        return self.since is not None

    def update(self, now, gps_why):
        """Return 'run', 'hold', 'resume' (hold just ended) or 'timeout'."""
        if not self.active:
            if gps_why is None:
                return "run"
            self.since, self.reason, self.good_since = now, gps_why, None
            self.count += 1
            return "hold"
        if gps_why is not None:
            self.reason = gps_why
            self.good_since = None
            if now - self.since > GPS_HOLD_MAX_S:
                return "timeout"
            return "hold"
        if self.good_since is None:
            self.good_since = now
        if now - self.good_since >= GPS_RESUME_GOOD_S:
            self.total_s += now - self.since
            self.since = None
            return "resume"
        return "hold"


def drive_loop(rover, step_fn, label="", on_resume=None, stats=None):
    """Run step_fn(now, rover) -> (speed, angular, done, status). 20 Hz.

    GPS faults (stale position or heading, fix not RTK fixed, old
    corrections) put the loop in a HOLD: zero speed, AUTO_ACTIVE kept alive,
    step_fn not called. When the fix has been good for GPS_RESUME_GOOD_S it
    calls on_resume(now, held_s) so the controller can shift its timers, and
    carries on. Hard faults (CAN, Dashboard state) stop immediately.

    stats, if given, is a dict filled with hold count and time."""
    period = 1.0 / CONTROL_HZ
    nxt = time.monotonic()
    t_start = nxt
    last_line = 0.0
    last_word = None
    kb = Keyboard()
    hold = GpsHold()
    last_hold_print = 0.0
    stopped_why = None
    try:
        while True:
            now = time.monotonic()
            rover.poll(now)

            if kb.quit_pressed():
                stopped_why = "operator"
                break

            why = rover.hard_fault(now)
            if why:
                stopped_why = why
                break

            h = hold.update(now, rover.gps_fault(now))
            if h == "timeout":
                stopped_why = (f"GPS lost {now - hold.since:.0f} s "
                               f"({hold.reason}), gave up")
                break
            if h == "resume":
                print(f"  {now - t_start:6.2f}  {label}GPS back, resuming "
                      f"(hold #{hold.count})")
                if on_resume is not None:
                    on_resume(now)
            if h == "hold":
                # Keep commanding: zero motion, AUTO_ACTIVE, at the normal
                # rate. Going silent would trip the watchdog, and that needs
                # a person at the screen.
                if now >= nxt:
                    try:
                        rover.send(STATE_AUTO_ACTIVE, 0, 0)
                    except OSError as e:
                        stopped_why = f"CAN TX failed ({e.errno})"
                        break
                    nxt += period
                    if nxt < now:
                        nxt = now + period
                if now - last_hold_print >= 2.0:
                    last_hold_print = now
                    print(f"  {now - t_start:6.2f}  {label}HOLD  {hold.reason}"
                          f"  ({now - hold.since:4.1f} s of "
                          f"{GPS_HOLD_MAX_S:.0f})")
                time.sleep(0.002)
                continue

            speed, ang, done, status = step_fn(now, rover)
            if done:
                # print the final status before breaking - it carries the
                # arrival numbers, which are the whole point of the run
                print(f"  {now - t_start:6.2f}  {label}{status}")
                stopped_why = "complete"
                break

            if now >= nxt:
                try:
                    rover.send(STATE_AUTO_ACTIVE, speed, ang)
                except OSError as e:
                    stopped_why = f"CAN TX failed ({e.errno})"
                    break
                nxt += period
                if nxt < now:
                    nxt = now + period
                word = status.split()[0]
                if VERBOSE:
                    if word != last_word or now - last_line >= 0.25:
                        print(f"  {now - t_start:6.2f}  {label}{status}")
                        last_line = now
                else:
                    sys.stdout.write(f"\r  {label}{status}   ")
                    sys.stdout.flush()
                last_word = word
            time.sleep(0.002)
    finally:
        kb.restore()
        if hold.active:
            hold.total_s += time.monotonic() - hold.since
        if stats is not None:
            stats["gps_holds"] = hold.count
            stats["gps_hold_s"] = round(hold.total_s, 1)
        for _ in range(10):
            try:
                rover.send(STATE_AUTO_READY, 0, 0)
            except OSError:
                break
            time.sleep(period)
    print(f"\n  stopped: {stopped_why}")
    return stopped_why


# ============================================================================
#  goto / follow / mission
# ============================================================================

class GotoController:
    """Drive to one point and stop on it, then optionally square up.

    Turn direction is LATCHED once aligning starts. Near 180 degrees of
    bearing error the sign of that error flips on noise alone, so choosing
    the direction fresh every tick makes the machine rock either side of the
    antipode forever instead of committing to a turn. Observed 2026-10-27:
    105 s stuck at +/-179 deg, never completing.

    The approach bearing is LOCKED when creep begins. Steering after that
    holds that bearing rather than re-aiming at the target, because the
    bearing to a point 3 cm away is almost entirely RTK noise.

    Arrival is judged on the along-track remainder, not radial distance.
    Radial distance can never reach zero through noise; along-track crosses
    zero cleanly.
    """

    def __init__(self, frame, tx, ty, final_heading=None):
        self.frame = frame
        self.tx, self.ty = tx, ty
        self.final_heading = final_heading
        self.state = "align"
        self.lock_bearing = None
        self.arrive_err = None
        self.pulse_until = 0.0
        self.pulse_rest = 0.0
        self.creep_since = None
        self.turn_dir = None          # latched +1 / -1
        self.turn_start = None
        self.turn_start_err = None
        self.decided = False          # row mode chosen on the first tick
        self.row_mode = False
        self.min_along = None
        self.turned_err = None        # error after the final turn
        self.fix_tries = 0
        self.fix_dir = 0
        self.check_at = 0.0
        self.check_samples = []
        self.af = []                  # recent (t_pos, along) for arrival
        self.af_tag = None

    def _latch(self, err, now):
        if self.turn_dir is None:
            # Bearing increases clockwise from north, so a POSITIVE bearing
            # error means the target is to the RIGHT and the machine must
            # turn right - which is a negative angular rate, since positive
            # mrad/s turns left. Getting this backwards makes the error grow
            # instead of shrink: observed 2026-10-27, 13 deg -> 77 deg over
            # 12 s of turning the wrong way.
            self.turn_dir = -1.0 if err > 0 else 1.0
            self.turn_start = now
            self.turn_start_err = abs(err)
        return self.turn_dir

    def _release(self):
        self.turn_dir = None
        self.turn_start = None

    def on_resume(self, now):
        """Called by drive_loop after a GPS hold. The leg carries on with
        its locked approach line and latched turn direction intact - starting
        the leg afresh would re-aim at a target that may be 5 cm away, where
        the bearing is pure noise, and could turn the machine in place inside
        a plot. Only the clocks are reset, because the machine sat still."""
        if self.turn_start is not None:
            self.turn_start = now              # stall timer restarts
        if self.creep_since is not None:
            self.creep_since = now             # re-earn the settle gate
        self.pulse_until = 0.0
        self.pulse_rest = now + PULSE_REST_S   # settle, re-measure, then pulse

    def _row_errors(self, x, y, line_brg):
        """along (remaining, + = target ahead) and cross (+ = target to the
        left of the line... i.e. machine to the right) on a line bearing."""
        lb = math.radians(line_brg)
        dx, dy = self.tx - x, self.ty - y
        return (dx * math.sin(lb) + dy * math.cos(lb),
                -dx * math.cos(lb) + dy * math.sin(lb))

    def _filt(self, rover, along, tag):
        """Mean of the last ALONG_FILTER_N DISTINCT position readings.
        One RTK reading is +/- 1 cm; stopping on the first one that dips
        under the arrival radius stops early by about that much."""
        if tag != self.af_tag:
            self.af, self.af_tag = [], tag
        if not self.af or self.af[-1][0] != rover.t_pos:
            self.af.append((rover.t_pos, along))
            self.af = self.af[-ALONG_FILTER_N:]
        return sum(a for _, a in self.af) / len(self.af)

    def step(self, now, rover):
        x, y = control_point(self.frame, rover.lat, rover.lon, rover.heading)
        dx, dy = self.tx - x, self.ty - y
        dist = math.hypot(dx, dy)
        brg = bearing_to(dx, dy)
        berr = wrap180(brg - rover.heading)

        if not self.decided:
            self.decided = True
            if ROW_LINE_FOLLOW and self.final_heading is not None and \
                    abs(wrap180(brg - self.final_heading)) <= IN_ROW_DEG:
                _, c0 = self._row_errors(x, y, self.final_heading)
                if abs(c0) <= ROW_LINE_MAX_CROSS_M:
                    self.row_mode = True
                    self.lock_bearing = self.final_heading

        if self.state == "align":
            if self.row_mode:
                berr = wrap180(self.lock_bearing - rover.heading)
            if abs(berr) <= HEADING_RESUME_DEG:
                self._release()
                self.state = "drive"
            else:
                d = self._latch(berr, now)
                elapsed = now - self.turn_start
                if elapsed > ALIGN_STALL_S and \
                        abs(berr) > self.turn_start_err - 10.0:
                    return 0, 0, True, \
                        f"STALLED aligning: {elapsed:.0f} s, err still " \
                        f"{berr:+.1f} deg. Raise TURN_IN_PLACE_MRAD_S."
                return ALIGN_ASSIST_MM_S, ANGULAR_SIGN * d * TURN_IN_PLACE_MRAD_S, \
                    False, f"align   err {berr:+7.2f} deg  turning " \
                           f"{'left' if d > 0 else 'right'}  {elapsed:4.1f}s"

        if self.state == "drive":
            if abs(berr) > HEADING_ALIGN_DEG and self.lock_bearing is None:
                self.state = "align"
                d = self._latch(berr, now)
                return 0, ANGULAR_SIGN * d * TURN_IN_PLACE_MRAD_S, False, \
                    f"align   err {berr:+7.2f} deg   {dist:6.3f} m to go"

            # Fix the approach line while the bearing is still trustworthy.
            if self.lock_bearing is None and dist <= BEARING_LOCK_M:
                self.lock_bearing = brg

            if self.lock_bearing is None:
                mrad, _, _ = steer(x, y, rover.heading, self.tx, self.ty,
                                   CRUISE_SPEED_MM_S)
                mrad = rolling_turn(mrad, CRUISE_SPEED_MM_S)
                return CRUISE_SPEED_MM_S, mrad, False, \
                    f"drive   err {berr:+7.2f} deg   {dist:6.3f} m to go   " \
                    f"{CRUISE_SPEED_MM_S} mm/s"

            # Locked: drive the line, no steering, speed set by how far is left
            lb = math.radians(self.lock_bearing)
            along = dx * math.sin(lb) + dy * math.cos(lb)
            cross = -dx * math.cos(lb) + dy * math.sin(lb)
            herr = wrap180(self.lock_bearing - rover.heading)

            if along <= CREEP_DISTANCE_M and self.creep_since is None:
                self.creep_since = now
            if self.min_along is None or along < self.min_along:
                self.min_along = along
            along_f = self._filt(rover, along, "drive") \
                if along <= CREEP_DISTANCE_M else along

            slow_enough = (self.creep_since is not None and
                           now - self.creep_since >= APPROACH_SETTLE_S)
            if along_f <= ARRIVE_RADIUS_M and not slow_enough:
                # Arrived on paper but still carrying speed. Hold position
                # rather than stopping mid-deceleration.
                return 0, 0, False, \
                    f"settle  along {along*100:6.2f} cm  " \
                    f"cross {cross*100:+6.2f} cm"

            if along_f <= ARRIVE_RADIUS_M:
                self.arrive_err = math.hypot(along_f, cross)
                if self.final_heading is None:
                    return 0, 0, True, \
                        f"arrived  along {along*100:+5.2f} cm  " \
                        f"cross {cross*100:+6.2f} cm"
                self._release()
                self.state = "turn"
            else:
                if abs(herr) > CREEP_ABORT_HEADING_DEG:
                    return 0, 0, True, \
                        f"ABORT: heading drifted {herr:+.1f} deg off the " \
                        f"locked approach line"
                if along > self.min_along + CREEP_ABORT_OVERSHOOT_M:
                    return 0, 0, True, \
                        f"ABORT: {along*100:.0f} cm out and growing"
                if along <= CREEP_DISTANCE_M:
                    # Last 30 cm: straight, no steering. Any correction here
                    # is a 12 deg/s lurch against a 1 cm error.
                    return CREEP_SPEED_MM_S, 0, False, \
                        f"creep   along {along*100:6.2f} cm  " \
                        f"cross {cross*100:+6.2f} cm  hdg err {herr:+6.2f} deg"

                speed = ramp_speed(along)
                mrad = 0
                tag = "final "
                if self.row_mode:
                    # Pure pursuit to a point ON THE ROW LINE, lookahead
                    # ahead of the machine, extended past the mark: pulls it
                    # onto the line and parallel to it.
                    if (abs(cross) > CROSS_CORRECT_M or
                            abs(herr) > HEADING_CORRECT_DEG) and \
                            speed >= STEER_MIN_SPEED_MM_S:
                        la = ROW_LOOKAHEAD_M
                        ux, uy = math.sin(lb), math.cos(lb)
                        cx_ = self.tx + (la - along) * ux
                        cy_ = self.ty + (la - along) * uy
                        mrad, _, _ = steer(x, y, rover.heading, cx_, cy_, speed)
                        mrad = rolling_turn(mrad, speed)
                        tag = "row   "
                    else:
                        tag = "row ok"
                elif abs(cross) > CROSS_CORRECT_M and \
                        speed >= STEER_MIN_SPEED_MM_S:
                    mrad, _, _ = steer(x, y, rover.heading, self.tx, self.ty,
                                       speed)
                    mrad = rolling_turn(mrad, speed)
                    tag = "corr  "
                return speed, mrad, False, \
                    f"{tag}  along {along*100:6.2f} cm  cross {cross*100:+6.2f} cm  " \
                    f"{speed:3d} mm/s  hdg {herr:+6.2f} deg"

        if self.state == "creep":
            self.state = "drive"      # creep is now part of the locked run

        if self.state == "turn":
            herr = wrap180(self.final_heading - rover.heading)
            if abs(herr) <= FINAL_HEADING_TOL_DEG:
                self._release()
                self.state = "check"
                self.check_at = now + CHECK_SETTLE_S
                self.check_samples = []
                return 0, 0, False, \
                    f"turn    heading err {herr:+7.2f} deg  squared, checking"

            # Below about 220 mrad/s the machine will not rotate, so a small
            # correction cannot be made slowly - it has to be made briefly.
            # Pulse: turn at full rate for a moment, stop, re-measure.
            if abs(herr) > TURN_COARSE_DEG:
                d = self._latch(herr, now)
                elapsed = now - self.turn_start
                if elapsed > ALIGN_STALL_S and \
                        abs(herr) > self.turn_start_err - 5.0:
                    return 0, 0, True, \
                        f"stopped {self.arrive_err*100:.2f} cm from target; " \
                        f"final turn stalled at {herr:+.1f} deg"
                return 0, ANGULAR_SIGN * d * TURN_SLOW_MRAD_S, False, \
                    f"turn    heading err {herr:+7.2f} deg  coarse"

            self._release()
            if now < self.pulse_until:
                d = -1.0 if herr > 0 else 1.0
                return 0, ANGULAR_SIGN * d * TURN_SLOW_MRAD_S, False, \
                    f"turn    heading err {herr:+7.2f} deg  pulse"
            if now < self.pulse_rest:
                return 0, 0, False, \
                    f"turn    heading err {herr:+7.2f} deg  settling"
            on = min(PULSE_MAX_S,
                     max(PULSE_MIN_S,
                         math.radians(abs(herr)) / (TURN_SLOW_MRAD_S / 1000.0)))
            self.pulse_until = now + on
            self.pulse_rest = self.pulse_until + PULSE_REST_S
            return 0, 0, False, \
                f"turn    heading err {herr:+7.2f} deg  pulse {on*1000:.0f} ms"

        if self.state == "check":
            a_now, c_now = self._row_errors(x, y, self.final_heading)
            if now < self.check_at:
                if now >= self.check_at - CHECK_SETTLE_S / 2:
                    self.check_samples.append((a_now, c_now))
                return 0, 0, False, "check   settling"
            self.check_samples.append((a_now, c_now))
            n = len(self.check_samples)
            along = sum(a for a, _ in self.check_samples) / n
            cross = sum(c for _, c in self.check_samples) / n
            herr = wrap180(self.final_heading - rover.heading)
            if self.turned_err is None:
                self.turned_err = math.hypot(along, cross)
            if abs(along) > POST_TURN_FIX_M and \
                    abs(along) <= POST_TURN_FIX_MAX_M and \
                    self.fix_tries < POST_TURN_FIX_TRIES:
                self.fix_tries += 1
                self.fix_dir = 1 if along > 0 else -1
                self.state = "fix"
                return 0, 0, False, \
                    f"check   turn moved it: along {along*100:+.2f} cm, " \
                    f"creeping {'forward' if along > 0 else 'back'}"
            note = ""
            if abs(along) > POST_TURN_FIX_MAX_M:
                note = "  (along error too large to creep-fix)"
            return 0, 0, True, \
                f"squared up: arrived {self.arrive_err*100:.2f} cm, after turn " \
                f"{self.turned_err*100:.2f} cm, now along {along*100:+.2f} " \
                f"cross {cross*100:+.2f} cm, heading err {herr:+.2f} deg{note}"

        if self.state == "fix":
            # Straight, no steering, at creep speed - forward or reverse.
            along, cross = self._row_errors(x, y, self.final_heading)
            along = self._filt(rover, along, f"fix{self.fix_tries}")
            if along * self.fix_dir <= ARRIVE_RADIUS_M:
                self.state = "check"
                self.check_at = now + CHECK_SETTLE_S
                self.check_samples = []
                return 0, 0, False, \
                    f"fix     along {along*100:+6.2f} cm  done"
            return self.fix_dir * CREEP_SPEED_MM_S, 0, False, \
                f"fix     along {along*100:+6.2f} cm  cross {cross*100:+6.2f} cm"

        return 0, 0, True, "done"


def make_goto_step(frame, tx, ty, final_heading=None):
    return GotoController(frame, tx, ty, final_heading).step


def cmd_goto(args):
    rover = Rover()
    rover.open()
    try:
        if not wait_ready(rover):
            return 1
        frame = Frame(rover.lat, rover.lon)
        tx, ty = frame.to_xy(args.lat, args.lon)
        d = math.hypot(tx, ty)
        print(f"target {d:.1f} m away")
        if d > MAX_START_DISTANCE_M:
            print(f"refusing: further than {MAX_START_DISTANCE_M} m")
            return 1
        fh = args.heading
        if fh is None and not args.no_square:
            fh = bearing_to(tx, ty)      # end facing the way it approached
        print(f"  final heading: {'none' if fh is None else f'{fh:.1f} deg'}")
        print("  q to stop\n")
        ctl = GotoController(frame, tx, ty, fh)
        drive_loop(rover, ctl.step, on_resume=ctl.on_resume)
    finally:
        rover.close()
    return 0


def cmd_follow(args):
    pts = load_path_csv(args.path)
    rover = Rover()
    rover.open()
    try:
        if not wait_ready(rover):
            return 1
        frame = Frame(rover.lat, rover.lon)
        pp = PurePursuit(frame, pts)
        sx, sy = pp.xy[0]
        cx, cy = frame.to_xy(rover.lat, rover.lon)
        d0 = math.hypot(sx - cx, sy - cy)
        print(f"path {len(pts)} points, start {d0:.1f} m away")
        if d0 > MAX_START_DISTANCE_M:
            print(f"refusing: start further than {MAX_START_DISTANCE_M} m")
            return 1
        print("  q to stop\n")
        gate = AlignGate()

        def step(now, rover):
            x, y = frame.to_xy(rover.lat, rover.lon)
            speed = min(CRUISE_SPEED_MM_S, args.speed)
            la = max(LOOKAHEAD_MIN_M,
                     min(LOOKAHEAD_M, LOOKAHEAD_GAIN * speed / 1000.0))
            (tx, ty), xtrack, at_end = pp.target(x, y, la)
            togo = pp.remaining(x, y)
            if at_end and togo < ARRIVE_RADIUS_M:
                return 0, 0, True, "path complete"
            if togo < CREEP_DISTANCE_M:
                speed = CREEP_SPEED_MM_S
            mrad, _, berr = steer(x, y, rover.heading, tx, ty, speed)
            speed, mrad = gate.apply(berr, speed, mrad)
            return speed, mrad, False, \
                f"x-track {xtrack:5.2f} m  to go {togo:6.1f} m  " \
                f"{speed:4d} mm/s  {mrad:+7.1f} mrad/s"

        drive_loop(rover, step)
    finally:
        rover.close()
    return 0


BIG_TURN_DEG = 20.0             # final turns above this are flagged in the plan


def final_heading_for(mark, approach_brg, override=None):
    """Heading to square up to at a mark.

    The TAUGHT heading, so each plot is imaged facing the way it was taught
    and the robot turns only as much as it did then. A single --heading for
    the whole mission is wrong on a serpentine route: every second row is
    driven the other way, and pinning it to one bearing turns the machine
    180 deg in place on every plot of that row - the pivot that took the
    motors to 220 F and scuffs the turf."""
    if override is not None:
        return override
    if mark.get("heading") is not None:
        return mark["heading"]
    return approach_brg


def plan_legs(frame, marks, start_xy, override=None):
    """Leg-by-leg plan: distance, approach bearing, final heading, and the
    in-place turn that final heading implies on arrival."""
    legs = []
    px, py = start_xy
    for m in marks:
        tx, ty = frame.to_xy(m["lat"], m["lon"])
        d = math.hypot(tx - px, ty - py)
        brg = bearing_to(tx - px, ty - py) if d > 1e-6 else None
        fh = final_heading_for(m, brg, override)
        turn = wrap180(fh - brg) if (fh is not None and brg is not None) else 0.0
        legs.append({"label": m["label"], "dist": d, "approach": brg,
                     "final": fh, "turn": turn})
        px, py = tx, ty
    return legs


def print_plan(legs):
    print(f"  {'mark':<10} {'leg m':>6} {'approach':>9} {'final':>7} {'turn':>7}")
    for L in legs:
        ap = "-" if L["approach"] is None else f"{L['approach']:.1f}"
        fh = "-" if L["final"] is None else f"{L['final']:.1f}"
        flag = "   <- turns in place" if abs(L["turn"]) > BIG_TURN_DEG else ""
        print(f"  {L['label']:<10} {L['dist']:6.2f} {ap:>9} {fh:>7} "
              f"{L['turn']:+7.1f}{flag}")
    big = [L["label"] for L in legs if abs(L["turn"]) > BIG_TURN_DEG]
    if big:
        print(f"\n  {len(big)} mark(s) end with a turn over {BIG_TURN_DEG:.0f} deg: "
              f"{', '.join(big)}.\n  That should only be headland transit "
              f"points. A plot here means it was taught facing a different "
              f"way than it is approached.")


def cmd_plan(args):
    """Print the mission plan from a teach file. No hardware."""
    marks = load_marks_json(args.path)
    frame = Frame(marks[0]["lat"], marks[0]["lon"])
    legs = plan_legs(frame, marks, frame.to_xy(marks[0]["lat"], marks[0]["lon"]),
                     args.heading)
    print(f"{len(marks)} marks from {args.path}\n")
    print_plan(legs)
    return 0


RUN_COLUMNS = ["t_utc", "label", "result", "stop_err_cm", "along_cm",
               "cross_cm", "heading_err_deg", "fix_q", "corr_age_s",
               "gps_ok_at_capture", "gps_fault_in_dwell", "gps_holds",
               "gps_hold_s", "lat", "lon", "heading_deg"]


def stop_error(frame, rover, tx, ty, final_hdg):
    """Where the rotation centre stopped relative to the mark, in the frame
    of the final heading: along > 0 is PAST the mark, cross > 0 is to the
    LEFT. This is the residual a crop in post-processing would undo."""
    x, y = control_point(frame, rover.lat, rover.lon, rover.heading)
    ex, ey = x - tx, y - ty
    ref = final_hdg if final_hdg is not None else rover.heading
    along, left = vehicle_frame(ex, ey, ref)
    herr = wrap180(rover.heading - ref)
    return math.hypot(ex, ey), along, left, herr


def cmd_mission(args):
    marks = load_marks_json(args.path)
    base = os.path.splitext(args.path)[0]
    done_path = base + "_completed.txt"
    completed = set()
    if os.path.exists(done_path) and args.resume:
        completed = set(open(done_path).read().split())
        print(f"resuming, {len(completed)} already done")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_path = f"{base}_run_{stamp}.csv"

    rover = Rover()
    rover.open()
    try:
        if not wait_ready(rover):
            return 1
        frame = Frame(rover.lat, rover.lon)
        todo = [m for m in marks if m["label"] not in completed]
        if not todo:
            print("nothing left to do")
            return 0

        cx, cy = control_point(frame, rover.lat, rover.lon, rover.heading)
        legs = plan_legs(frame, todo, (cx, cy), args.heading)
        print(f"{len(todo)} waypoints\n")
        print_plan(legs)
        print()
        if args.heading is not None:
            print(f"NOTE: --heading {args.heading:.1f} overrides the taught "
                  f"heading at EVERY mark.\n")

        if legs[0]["dist"] > MAX_START_DISTANCE_M:
            print(f"refusing: first waypoint {legs[0]['dist']:.1f} m away, "
                  f"limit {MAX_START_DISTANCE_M} m")
            return 1

        with open(done_path, "a") as done_fh, \
                open(run_path, "w", buffering=1) as run_fh:
            w = csvmod.writer(run_fh)
            w.writerow(RUN_COLUMNS)
            print(f"run log -> {run_path}\n")

            for m in todo:
                label = m["label"]
                tx, ty = frame.to_xy(m["lat"], m["lon"])
                cx, cy = control_point(frame, rover.lat, rover.lon,
                                       rover.heading)
                final_hdg = final_heading_for(m, bearing_to(tx - cx, ty - cy),
                                              args.heading)
                ctl = GotoController(frame, tx, ty, final_hdg)
                stats = {}
                why = drive_loop(rover, ctl.step, label=f"{label}  ",
                                 on_resume=ctl.on_resume, stats=stats)
                if why != "complete":
                    w.writerow([datetime.now(timezone.utc).isoformat(
                        timespec="seconds"), label, f"stopped: {why}"]
                        + [""] * (len(RUN_COLUMNS) - 3))
                    print(f"  stopping at {label}: {why}")
                    print(f"  completed plots are in {done_path}; "
                          f"carry on with --resume")
                    return 1

                err, along, cross, herr = stop_error(frame, rover, tx, ty,
                                                     final_hdg)
                print(f"  {label}: stopped {err*100:.1f} cm from target "
                      f"(along {along*100:+.1f}, cross {cross*100:+.1f} cm), "
                      f"dwell {args.dwell:.0f} s")

                # Fix quality checked at the moment of capture - this is
                # where it matters most. drive_loop only returns "complete"
                # with a good fix, but say so explicitly and record it.
                now = time.monotonic()
                rover.poll(now)
                gps_ok = rover.gps_fault(now) is None
                fix_q, corr_age = rover.fix_q, rover.corr_age

                # ---- capture hook goes here ----
                # Trigger the camera app, confirm it wrote, then continue.
                # Flag and continue on failure; do not abort the session.
                # Pass it: label, gps_ok, err/along/cross, fix_q, corr_age.

                dwell_fault = None
                t_end = time.monotonic() + args.dwell
                while time.monotonic() < t_end:
                    now = time.monotonic()
                    rover.poll(now)
                    hf = rover.hard_fault(now)
                    if hf:
                        print(f"  stopping during dwell at {label}: {hf}")
                        return 1
                    gf = rover.gps_fault(now)
                    if gf and dwell_fault is None:
                        dwell_fault = gf
                    rover.send(STATE_AUTO_ACTIVE, 0, 0)
                    time.sleep(1.0 / CONTROL_HZ)

                result = "ok"
                if not gps_ok or dwell_fault:
                    result = "flag_gps"
                    print(f"  {label}: FLAGGED - GPS "
                          f"{'not good at capture' if not gps_ok else dwell_fault}")
                if stats.get("gps_holds"):
                    print(f"  {label}: {stats['gps_holds']} GPS hold(s), "
                          f"{stats['gps_hold_s']:.0f} s")
                w.writerow([
                    datetime.now(timezone.utc).isoformat(timespec="seconds"),
                    label, result, f"{err*100:.2f}", f"{along*100:.2f}",
                    f"{cross*100:.2f}", f"{herr:.2f}", fix_q,
                    "" if corr_age is None else f"{corr_age:.1f}",
                    int(gps_ok), dwell_fault or "",
                    stats.get("gps_holds", 0), stats.get("gps_hold_s", 0.0),
                    f"{rover.lat:.9f}", f"{rover.lon:.9f}",
                    f"{rover.heading:.2f}"])
                done_fh.write(label + "\n")
                done_fh.flush()

        print("\nmission complete")
    finally:
        rover.close()
    return 0


# ============================================================================
#  yaw-test: how small a turn command does the machine obey while ROLLING?
# ============================================================================

class YawTest:
    """Drive straight, then hold a small constant angular command, then
    straight again - once per requested rate, alternating left and right so
    the machine stays roughly on a line. Records heading through each turn
    window and fits the yaw rate actually achieved."""

    SETTLE_S = 1.0          # straight before each turn
    TAIL_S = 0.5            # straight after
    SKIP_S = 0.4            # ignore the start of the turn window (response lag)

    def __init__(self, speed, rates, secs):
        self.speed, self.secs = speed, secs
        self.rates = [r * (1 if i % 2 == 0 else -1) for i, r in enumerate(rates)]
        self.i = 0
        self.t0 = None
        self.samples = []
        self.results = []

    def on_resume(self, now):
        self.t0 = None              # restart the current trial after a hold
        self.samples = []

    def step(self, now, rover):
        if self.i >= len(self.rates):
            return 0, 0, True, "yaw-test done"
        if self.t0 is None:
            self.t0 = now
            self.samples = []
        t = now - self.t0
        cmd = self.rates[self.i]
        if t < self.SETTLE_S:
            return self.speed, 0, False, f"trial {self.i+1}: straight"
        if t < self.SETTLE_S + self.secs:
            if t >= self.SETTLE_S + self.SKIP_S:
                self.samples.append((now, rover.heading))
            return self.speed, cmd, False, \
                f"trial {self.i+1}: {cmd:+d} mrad/s  hdg {rover.heading:.1f}"
        if t < self.SETTLE_S + self.secs + self.TAIL_S:
            return self.speed, 0, False, f"trial {self.i+1}: straight"
        self.results.append((cmd, fit_yaw_mrad_s(self.samples)))
        self.i += 1
        self.t0 = None
        return self.speed, 0, False, "next"


def fit_yaw_mrad_s(samples):
    """Least-squares yaw rate from (t, heading_deg) samples, in the same
    sign convention as commands (positive = left = heading decreasing)."""
    if len(samples) < 3:
        return None
    t0, h0 = samples[0]
    ts, hs = [], []
    acc = 0.0
    prev = h0
    for t, h in samples:
        acc += wrap180(h - prev)
        prev = h
        ts.append(t - t0)
        hs.append(acc)
    n = len(ts)
    mt, mh = sum(ts) / n, sum(hs) / n
    den = sum((a - mt) ** 2 for a in ts)
    if den <= 0:
        return None
    slope = sum((a - mt) * (b - mh) for a, b in zip(ts, hs)) / den  # deg/s
    return -ANGULAR_SIGN * math.radians(slope) * 1000.0


def yaw_report(results):
    print(f"\n  {'commanded':>10} {'achieved':>10} {'ratio':>6}   (mrad/s, + = left)")
    floor = None
    for cmd, got in results:
        if got is None:
            print(f"  {cmd:>+10d} {'-':>10}")
            continue
        r = got / cmd if cmd else 0.0
        print(f"  {cmd:>+10d} {got:>+10.0f} {r:>6.2f}")
        if floor is None and r >= 0.5:
            floor = abs(cmd)
    if floor is None:
        print("\n  No rate reached half of what was asked. Keep "
              "ANGULAR_MIN_ROLLING_MRAD_S at 220 and try larger rates.")
    else:
        print(f"\n  Smallest rate that mostly worked: {floor} mrad/s. Set\n"
              f"  ANGULAR_MIN_ROLLING_MRAD_S = {floor} in amiga_nav.py and "
              f"re-run a mission.")
    return floor


def cmd_yaw_test(args):
    rates = [int(v) for v in args.mrad.split(",")]
    per = YawTest.SETTLE_S + args.secs + YawTest.TAIL_S
    dist = args.speed / 1000.0 * per * len(rates)
    print(f"yaw-test: {len(rates)} trials at {args.speed} mm/s, rates "
          f"{rates} mrad/s alternating left/right.")
    print(f"  Drives about {dist:.1f} m forward. Open ground, NOT on plots.")
    if dist > 12.0:
        print("refusing: more than 12 m. Fewer rates or shorter --secs.")
        return 1
    rover = Rover()
    rover.open()
    try:
        if not wait_ready(rover):
            return 1
        rover.angular_floor = False     # send exactly what is asked
        yt = YawTest(args.speed, rates, args.secs)
        print("  q to stop\n")
        why = drive_loop(rover, yt.step, label="yaw  ", on_resume=yt.on_resume)
        if yt.results:
            yaw_report(yt.results)
        if why != "complete":
            print(f"\n  stopped early: {why}")
            return 1
    finally:
        rover.close()
    return 0


# ============================================================================

def cmd_calib_offset(args):
    """Turn 180 deg in place and measure how far the antenna moved.
    Half that chord is the antenna's offset from the rotation centre."""
    rover = Rover()
    rover.open()
    try:
        if not wait_ready(rover):
            return 1
        frame = Frame(rover.lat, rover.lon)
        start = frame.to_xy(rover.lat, rover.lon)
        target = (rover.heading + args.turn) % 360.0
        print(f"turning {args.turn:.0f} deg in place. Hands off.\n")

        class Spin:
            def __init__(self):
                self.dir = None

            def step(self, now, r):
                e = wrap180(target - r.heading)
                if abs(e) <= 2.0:
                    return 0, 0, True, f"reached {r.heading:.1f} deg"
                if self.dir is None:
                    self.dir = -1.0 if e > 0 else 1.0
                return 0, ANGULAR_SIGN * self.dir * TURN_IN_PLACE_MRAD_S, \
                    False, f"spin    err {e:+7.2f} deg"

        drive_loop(rover, Spin().step)
        end = frame.to_xy(rover.lat, rover.lon)
        chord = math.hypot(end[0] - start[0], end[1] - start[1])
        half = math.radians(args.turn / 2.0)
        offset = chord / (2.0 * math.sin(half)) if math.sin(half) else 0.0
        print(f"\n  antenna moved {chord*100:.1f} cm over {args.turn:.0f} deg")
        print(f"  ANTENNA_OFFSET_M = {offset:.3f}")
        print(f"  (currently {ANTENNA_OFFSET_M:.3f} in vehicle.py - average several "
              f"readings before changing it)")
    finally:
        rover.close()
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("replay")
    r.add_argument("path")
    r.add_argument("--offset", type=float, default=1.0,
                   help="start this far off the path, metres")
    r.add_argument("--max-time", type=float, default=600.0)

    f = sub.add_parser("follow")
    f.add_argument("path")
    f.add_argument("--speed", type=int, default=CRUISE_SPEED_MM_S)

    g = sub.add_parser("goto")
    g.add_argument("lat", type=float)
    g.add_argument("lon", type=float)
    g.add_argument("--heading", type=float, default=None,
                   help="square up to this bearing after arriving")
    g.add_argument("--no-square", action="store_true",
                   help="stop on the point without a final heading turn")

    c = sub.add_parser("calib-offset")
    c.add_argument("--turn", type=float, default=180.0,
                   help="degrees to spin (default 180)")

    m = sub.add_parser("mission")
    m.add_argument("path")
    m.add_argument("--resume", action="store_true")
    m.add_argument("--heading", type=float, default=None,
                   help="override: square EVERY mark to this bearing instead "
                        "of its taught heading. Wrong on a serpentine route.")
    m.add_argument("--dwell", type=float, default=DWELL_S,
                   help=f"seconds stopped at each plot (default {DWELL_S:.0f})")

    yt = sub.add_parser("yaw-test",
                        help="measure the smallest turn obeyed while rolling")
    yt.add_argument("--speed", type=int, default=300)
    yt.add_argument("--mrad", default="40,80,120,160,220")
    yt.add_argument("--secs", type=float, default=2.0)

    pl = sub.add_parser("plan", help="print the mission plan, no hardware")
    pl.add_argument("path")
    pl.add_argument("--heading", type=float, default=None)

    args = p.parse_args()
    fn = {"replay": cmd_replay, "follow": cmd_follow,
          "goto": cmd_goto, "mission": cmd_mission, "plan": cmd_plan,
          "yaw-test": cmd_yaw_test,
          "calib-offset": cmd_calib_offset}[args.cmd]
    try:
        return fn(args)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 0


if __name__ == "__main__":
    sys.exit(main())
