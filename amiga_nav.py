#!/usr/bin/env python3
"""
amiga_nav.py - Phase 0 step 6. Pure pursuit path following and waypoint drive.

    replay   <csv>              simulate the controller against a logged path
    follow   <csv>              drive the path recorded in that log
    goto     <lat> <lon>        drive to one point and stop
    mission  <json>             drive the marks in a rover_log teach summary

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

# The LG580P reports heading 180 deg from the direction of travel with the
# mast wired Antenna-1 forward, Antenna-2 rear. Measured 2026-10-27 against
# course-over-ground: circular mean -177.6 deg over 1839 moving samples,
# circular sd 5.5 deg. Set to 0.0 if the antenna cables are ever swapped.
HEADING_OFFSET_DEG = 180.0

# Sign of the angular rate field in AmigaRpdo1. NOT YET VERIFIED on the wire.
# +1 means positive mrad/s turns the vehicle left (counter-clockwise seen
# from above). If the robot steers away from the path in `follow`, flip this
# to -1. Check it with amiga_teleop.py before driving a path.
ANGULAR_SIGN = +1

# Measured on turf, 2026-10-27: below ~40 mm/s the machine does not break
# static friction. At 40 mm/s it stops immediately on a zero command, so no
# deceleration overshoot to compensate for down here.
# The GNSS antenna sits forward of the vehicle's centre of rotation, so a
# turn in place swings it through an arc of chord 2*L*sin(theta/2) even
# though the machine has not translated. Measured 2026-10-27: a 167 deg
# square-up moved the reported position 117 cm, a 75 deg one moved it 90 cm.
# All control therefore runs on the rotation centre, not on the antenna.
# Measure with: python3 amiga_nav.py calib-offset
ANTENNA_OFFSET_M = 0.65        # antenna ahead of the rotation centre

CREEP_SPEED_MM_S = 40

# Measured on turf, 2026-10-27: below roughly 150-200 mrad/s the machine does
# not rotate at all. Any non-zero angular command is raised to this magnitude,
# because a command below it is not a gentle correction, it is no correction.
ANGULAR_MIN_MRAD_S = 220

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
CROSS_CORRECT_M = 0.03         # steer back only when further off the line

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
MAX_DATA_AGE_S = 0.5           # stop if position or heading goes stale
MAX_CORR_AGE_S = 5.0           # stop if corrections stop arriving

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

EARTH_R = 6371000.0


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


def load_marks_json(path):
    with open(path) as fh:
        data = json.load(fh)
    out = []
    for m in data.get("marks", []):
        if m.get("lat") and m.get("lon"):
            out.append((m["label"], float(m["lat"]), float(m["lon"])))
    if not out:
        raise SystemExit(f"no marks in {path}")
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
                    self.heading = (st["heading_deg"]
                                    + HEADING_OFFSET_DEG) % 360.0
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
                self.heading = (h + HEADING_OFFSET_DEG) % 360.0
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
        if ang and abs(ang) < ANGULAR_MIN_MRAD_S:
            ang = math.copysign(ANGULAR_MIN_MRAD_S, ang)
        ang = int(max(-MAX_ANGULAR_MRAD_S, min(MAX_ANGULAR_MRAD_S, ang)))
        self.sock.send(CAN_FRAME.pack(
            COBID_RPDO1, 8, struct.pack("<Bhh3x", state, lin, ang)))

    def unsafe(self, now):
        """Return a reason to stop, or None."""
        if self.t_pos is None or now - self.t_pos > MAX_DATA_AGE_S:
            return "position stale"
        if self.t_head is None or now - self.t_head > MAX_DATA_AGE_S:
            return "heading stale"
        if self.t_can is None or now - self.t_can > MAX_DATA_AGE_S:
            return "CAN stale"
        if REQUIRE_RTK_FIXED and self.fix_q != "4":
            return f"fix quality {self.fix_q}, want 4"
        if self.corr_age is not None and self.corr_age > MAX_CORR_AGE_S:
            return f"corrections {self.corr_age:.1f} s old"
        if self.state not in (STATE_AUTO_READY, STATE_AUTO_ACTIVE):
            return f"dashboard state {self.state}, not auto"
        return None


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


def drive_loop(rover, step_fn, label=""):
    """Run step_fn(now, rover) -> (speed, angular, done, status). 20 Hz."""
    period = 1.0 / CONTROL_HZ
    nxt = time.monotonic()
    t_start = nxt
    last_line = 0.0
    last_word = None
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    stopped_why = None
    try:
        tty.setcbreak(fd)
        while True:
            now = time.monotonic()
            rover.poll(now)

            if select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.read(1).lower() == "q":
                    stopped_why = "operator"
                    break

            why = rover.unsafe(now)
            if why:
                stopped_why = why
                break

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
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
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

    def step(self, now, rover):
        x, y = control_point(self.frame, rover.lat, rover.lon, rover.heading)
        dx, dy = self.tx - x, self.ty - y
        dist = math.hypot(dx, dy)
        brg = bearing_to(dx, dy)
        berr = wrap180(brg - rover.heading)

        if self.state == "align":
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

            slow_enough = (self.creep_since is not None and
                           now - self.creep_since >= APPROACH_SETTLE_S)
            if along <= ARRIVE_RADIUS_M and not slow_enough:
                # Arrived on paper but still carrying speed. Hold position
                # rather than stopping mid-deceleration.
                return 0, 0, False, \
                    f"settle  along {along*100:6.2f} cm  " \
                    f"cross {cross*100:+6.2f} cm"

            if along <= ARRIVE_RADIUS_M:
                self.arrive_err = math.hypot(along, cross)
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
                if along > BEARING_LOCK_M + CREEP_ABORT_OVERSHOOT_M:
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
                if abs(cross) > CROSS_CORRECT_M:
                    mrad, _, _ = steer(x, y, rover.heading, self.tx, self.ty,
                                       speed)
                    tag = "corr  "
                return speed, mrad, False, \
                    f"{tag}  along {along*100:6.2f} cm  cross {cross*100:+6.2f} cm  " \
                    f"{speed:3d} mm/s  hdg {herr:+6.2f} deg"

        if self.state == "creep":
            self.state = "drive"      # creep is now part of the locked run

        if self.state == "turn":
            herr = wrap180(self.final_heading - rover.heading)
            if abs(herr) <= FINAL_HEADING_TOL_DEG:
                return 0, 0, True, \
                    f"squared up: {self.arrive_err*100:.2f} cm, " \
                    f"heading err {herr:+.2f} deg"

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
        drive_loop(rover, make_goto_step(frame, tx, ty, fh))
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


def cmd_mission(args):
    marks = load_marks_json(args.path)
    rover = Rover()
    rover.open()
    done_path = os.path.splitext(args.path)[0] + "_completed.txt"
    completed = set()
    if os.path.exists(done_path) and args.resume:
        completed = set(open(done_path).read().split())
        print(f"resuming, {len(completed)} already done")

    try:
        if not wait_ready(rover):
            return 1
        frame = Frame(rover.lat, rover.lon)
        todo = [m for m in marks if m[0] not in completed]
        print(f"{len(todo)} waypoints\n")

        first = todo[0]
        fx, fy = frame.to_xy(first[1], first[2])
        cx, cy = frame.to_xy(rover.lat, rover.lon)
        d0 = math.hypot(fx - cx, fy - cy)
        if d0 > MAX_START_DISTANCE_M:
            print(f"refusing: first waypoint {d0:.1f} m away, "
                  f"limit {MAX_START_DISTANCE_M} m")
            return 1

        with open(done_path, "a") as done_fh:
            for label, la, lo in todo:
                tx, ty = frame.to_xy(la, lo)
                cx, cy = frame.to_xy(rover.lat, rover.lon)
                # args.heading pins every plot to the row bearing. Without it
                # each plot squares to however it happened to be approached,
                # which varies leg by leg.
                final_hdg = args.heading
                if final_hdg is None:
                    final_hdg = bearing_to(tx - cx, ty - cy)
                why = drive_loop(rover,
                                 make_goto_step(frame, tx, ty, final_hdg),
                                 label=f"{label}  ")
                if why != "complete":
                    print(f"  aborting at {label}: {why}")
                    return 1

                x, y = control_point(frame, rover.lat, rover.lon,
                                     rover.heading)
                err = math.hypot(tx - x, ty - y)
                print(f"  {label}: stopped {err*100:.1f} cm from target, "
                      f"dwell {args.dwell:.0f} s")

                # ---- capture hook goes here ----
                # Trigger the camera app, confirm it wrote, then continue.
                # Flag and continue on failure; do not abort the session.
                t_end = time.monotonic() + args.dwell
                while time.monotonic() < t_end:
                    rover.poll(time.monotonic())
                    rover.send(STATE_AUTO_ACTIVE, 0, 0)
                    time.sleep(1.0 / CONTROL_HZ)

                done_fh.write(label + "\n")
                done_fh.flush()

        print("\nmission complete")
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
        print(f"  (currently set to {ANTENNA_OFFSET_M:.3f})")
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
                   help="square every plot to this bearing (the row bearing)")
    m.add_argument("--dwell", type=float, default=DWELL_S,
                   help=f"seconds stopped at each plot (default {DWELL_S:.0f})")

    args = p.parse_args()
    fn = {"replay": cmd_replay, "follow": cmd_follow,
          "goto": cmd_goto, "mission": cmd_mission,
          "calib-offset": cmd_calib_offset}[args.cmd]
    try:
        return fn(args)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 0


if __name__ == "__main__":
    sys.exit(main())
