#!/usr/bin/env python3
"""
test_gps_handling.py - offline tests for the 2026-11-11 GPS-error changes.

    python3 test_gps_handling.py

No hardware. The mission tests drive a simulated Amiga through the real
amiga_nav code (GotoController, drive_loop, cmd_mission, Rover.send) on a
fake clock, so a 2-row serpentine mission with a GPS dropout runs in about a
second.

What each group proves:
  Shared constants   teach and replay agree on where the vehicle is
  Mark averaging     a mark is refused unless fixed, still and consistent
  GPS hold           a dropout pauses the run and it carries on afterwards
  Per-plot heading   a serpentine route does not spin in the plots
"""

import argparse
import io
import json
import math
import os
import random
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vehicle                                   # noqa: E402
import amiga_nav as nav                          # noqa: E402
import rover_log                                 # noqa: E402

LAT0, LON0 = 30.6100, -96.3400                   # College Station-ish
PITCH = 1.22                                     # 4 ft plot centres


# ============================================================================
#  Simulation
# ============================================================================

class FakeTime:
    """Stand-in for the time module inside amiga_nav: sleep advances the
    clock instantly, so real-time control loops run in milliseconds."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.001)
        if self.t > 1000.0 + 3000.0:
            raise RuntimeError("simulation ran past 3000 s - controller stuck")


class FakeSock:
    def __init__(self, rover):
        self.rover = rover

    def send(self, raw):
        cid, dlc, data = nav.CAN_FRAME.unpack(raw)
        st, lin, ang = struct.unpack_from("<Bhh", data)
        self.rover.cmd = (st, lin, ang)

    def close(self):
        pass


class SimRover(nav.Rover):
    """A skid-steer on turf, driven through the real Rover.send().

    The TRUE antenna offset is vehicle.ANTENNA_OFFSET_M, so the sim and the
    software agree; the shared-constant test covers what happens when they
    did not."""

    def __init__(self, clock, frame, start_xy, start_hdg, pos_noise=0.005,
                 hdg_noise=0.1, seed=1):
        super().__init__()
        self.clock = clock
        self.frame = frame
        self.cx, self.cy = start_xy
        self.h = start_hdg
        self.cmd = (nav.STATE_AUTO_READY, 0, 0)
        self.rng = random.Random(seed)
        self.pos_noise = pos_noise
        self.hdg_noise = hdg_noise
        self.t_last = None
        self.dropouts = []          # (trigger(sim) -> bool, seconds)
        self.dropout_until = None
        self.lost_forever = None    # trigger -> fix never returns
        self.plots = []             # (x, y) plot centres, for rotation audit
        self.rot_in_plot = {}       # plot index -> degrees rotated in place

    def open(self):
        self.from_state = False
        self.sock = FakeSock(self)

    def close(self):
        pass

    def _integrate(self, now):
        if self.t_last is None:
            self.t_last = now
            return
        dt = now - self.t_last
        self.t_last = now
        st, lin, ang = self.cmd
        if st != nav.STATE_AUTO_ACTIVE:
            lin, ang = 0, 0
        # positive mrad/s turns LEFT, heading is clockwise from north
        dh = -math.degrees(ang / 1000.0) * dt
        if dh and lin == 0:
            for i, (px, py) in enumerate(self.plots):
                if math.hypot(self.cx - px, self.cy - py) < 0.3:
                    self.rot_in_plot[i] = self.rot_in_plot.get(i, 0.0) + abs(dh)
        self.h = (self.h + dh) % 360.0
        v = lin / 1000.0
        th = math.radians(self.h)
        self.cx += v * math.sin(th) * dt
        self.cy += v * math.cos(th) * dt
        self.speed = lin

    def poll(self, now):
        self._integrate(now)
        for trig, secs in list(self.dropouts):
            if trig(self):
                self.dropouts.remove((trig, secs))
                self.dropout_until = now + secs
        lost = self.dropout_until is not None and now < self.dropout_until
        if self.lost_forever is not None and self.lost_forever(self):
            self.lost_forever = lambda s: True
            lost = True
        L = vehicle.ANTENNA_OFFSET_M
        th = math.radians(self.h)
        ax = self.cx + L * math.sin(th) + self.rng.gauss(0, self.pos_noise)
        ay = self.cy + L * math.cos(th) + self.rng.gauss(0, self.pos_noise)
        self.lat, self.lon = self.frame.to_ll(ax, ay)
        self.heading = (self.h + self.rng.gauss(0, self.hdg_noise)) % 360.0
        self.fix_q = "5" if lost else "4"
        self.corr_age = 1.5
        self.t_pos = self.t_head = self.t_can = now
        self.state = nav.STATE_AUTO_ACTIVE


def antenna_reading(frame, cx, cy, hdg_true, rng=None, noise=0.0):
    """What the receiver reports with the vehicle centre at (cx, cy):
    antenna lat/lon and the RAW heading (180 deg off on this mast)."""
    L = vehicle.ANTENNA_OFFSET_M
    th = math.radians(hdg_true)
    ax, ay = cx + L * math.sin(th), cy + L * math.cos(th)
    if rng:
        ax += rng.gauss(0, noise)
        ay += rng.gauss(0, noise)
    lat, lon = frame.to_ll(ax, ay)
    return lat, lon, (hdg_true - vehicle.HEADING_OFFSET_DEG) % 360.0


def teach_file(path, frame, points, rng_seed=7):
    """Write a format-2 teach summary by running rover_log.average_mark on
    synthetic stationary readings at each (label, x, y, heading)."""
    rng = random.Random(rng_seed)
    marks = []
    for label, x, y, h in points:
        gga, hdg = [], []
        for _ in range(30):                     # 3 s at 10 Hz
            la, lo, raw = antenna_reading(frame, x, y, h, rng, 0.006)
            gga.append({"lat": la, "lon": lo, "fix_q": "4"})
            hdg.append(raw + rng.gauss(0, 0.08))
        fields, why = rover_log.average_mark(gga, hdg, [0, 2, -3, 1])
        assert fields, why
        m = {"label": label}
        m.update(fields)
        marks.append(m)
    with open(path, "w") as fh:
        json.dump({"format": rover_log.MARK_FORMAT, "marks": marks}, fh)


def serpentine(rows=2, plots=4):
    """Plot centres and headland transit points, in drive order.
    Row 1 runs north, row 2 back south; plots 1.22 m apart; turns 5 m out."""
    pts, plot_xy = [], []
    y0 = 2.0
    n = 1
    for r in range(rows):
        x = r * PITCH
        north = (r % 2 == 0)
        ys = [y0 + i * PITCH for i in range(plots)]
        if not north:
            ys = ys[::-1]
        h = 0.0 if north else 180.0
        for y in ys:
            pts.append((f"plot_{n:03d}", x, y, h))
            plot_xy.append((x, y))
            n += 1
        if r < rows - 1:
            yend = (ys[-1] + 5.0) if north else (ys[-1] - 5.0)
            pts.append((f"turn_{r+1}a", x, yend, h))
            pts.append((f"turn_{r+1}b", x + PITCH, yend, (h + 180.0) % 360.0))
    return pts, plot_xy


def run_mission(points, plot_xy, heading_override=None, setup=None,
                dwell=1.0, seed=1):
    """Teach `points`, then run the real cmd_mission against a SimRover."""
    tmp = tempfile.mkdtemp()
    frame = nav.Frame(LAT0, LON0)
    teach = os.path.join(tmp, "teach.json")
    teach_file(teach, frame, points)

    clock = FakeTime()
    sim = SimRover(clock, frame, (0.0, 0.0), 0.0, seed=seed)
    sim.plots = plot_xy
    if setup:
        setup(sim)

    saved_time, saved_rover = nav.time, nav.Rover
    nav.time = clock
    nav.Rover = lambda: sim
    try:
        args = argparse.Namespace(path=teach, resume=False,
                                  heading=heading_override, dwell=dwell)
        out = io.StringIO()
        with redirect_stdout(out):
            rc = nav.cmd_mission(args)
    finally:
        nav.time, nav.Rover = saved_time, saved_rover

    runs = [f for f in os.listdir(tmp) if "_run_" in f]
    rows = []
    if runs:
        import csv
        with open(os.path.join(tmp, runs[0])) as fh:
            rows = list(csv.DictReader(fh))
    with open(os.path.join(tmp, "teach_completed.txt")) as fh:
        done = fh.read().split()
    return rc, rows, done, sim, out.getvalue()


# ============================================================================
#  1. Shared constants
# ============================================================================

class SharedConstants(unittest.TestCase):

    def test_one_definition(self):
        self.assertIs(nav.ANTENNA_OFFSET_M, vehicle.ANTENNA_OFFSET_M)
        self.assertIs(rover_log.ANTENNA_OFFSET_M, vehicle.ANTENNA_OFFSET_M)
        self.assertFalse(hasattr(rover_log, "EARTH_R") and
                         rover_log.EARTH_R is not vehicle.EARTH_R)

    def test_centre_is_behind_antenna(self):
        frame = nav.Frame(LAT0, LON0)
        for h in (0, 37, 90, 180, 271):
            la, lo, raw = antenna_reading(frame, 3.0, 4.0, h)
            clat, clon = vehicle.rotation_centre(la, lo, vehicle.true_heading(raw))
            x, y = frame.to_xy(clat, clon)
            self.assertAlmostEqual(x, 3.0, places=3)
            self.assertAlmostEqual(y, 4.0, places=3)

    def test_teach_and_controller_agree(self):
        """A mark taught at a spot is the spot the controller drives to."""
        frame = nav.Frame(LAT0, LON0)
        tmp = tempfile.mkdtemp()
        p = os.path.join(tmp, "t.json")
        teach_file(p, frame, [("plot_001", 2.0, 5.0, 33.0)])
        with redirect_stdout(io.StringIO()):
            m = nav.load_marks_json(p)[0]
        x, y = frame.to_xy(m["lat"], m["lon"])
        self.assertLess(math.hypot(x - 2.0, y - 5.0), 0.01)
        self.assertAlmostEqual(m["heading"], 33.0, delta=0.1)
        # and the controller's own estimate of the vehicle there is the mark
        la, lo, raw = antenna_reading(frame, 2.0, 5.0, 33.0)
        cx, cy = nav.control_point(frame, la, lo, vehicle.true_heading(raw))
        self.assertLess(math.hypot(cx - x, cy - y), 0.01)

    def test_old_teach_file_is_repaired(self):
        """A format-1 file (raw heading, old rover_log maths) loads to the
        right place, not 1.6 m in front of it."""
        frame = nav.Frame(LAT0, LON0)
        la, lo, raw = antenna_reading(frame, 1.0, 1.0, 120.0)
        # the old rover_log: offset 0.794 applied along the RAW heading
        old_lat, old_lon = vehicle.rotation_centre(la, lo, raw, offset_m=0.794)
        ox, oy = frame.to_xy(old_lat, old_lon)
        self.assertGreater(math.hypot(ox - 1.0, oy - 1.0), 1.5)  # the bug
        tmp = tempfile.mkdtemp()
        p = os.path.join(tmp, "old.json")
        with open(p, "w") as fh:
            json.dump({"marks": [{"label": "plot_001",
                                  "lat": f"{old_lat:.9f}",
                                  "lon": f"{old_lon:.9f}",
                                  "ant_lat": f"{la:.9f}",
                                  "ant_lon": f"{lo:.9f}", "fix_q": "4",
                                  "heading_deg": f"{raw:.4f}",
                                  "antenna_offset_m": 0.794}]}, fh)
        out = io.StringIO()
        with redirect_stdout(out):
            m = nav.load_marks_json(p)[0]
        x, y = frame.to_xy(m["lat"], m["lon"])
        self.assertLess(math.hypot(x - 1.0, y - 1.0), 0.005)
        self.assertAlmostEqual(m["heading"], 120.0, places=3)
        self.assertIn("predates", out.getvalue())


# ============================================================================
#  2. Mark averaging
# ============================================================================

class MarkAveraging(unittest.TestCase):

    def window(self, n=30, noise=0.006, fix="4", h=10.0, seed=3):
        frame = nav.Frame(LAT0, LON0)
        rng = random.Random(seed)
        gga, hdg = [], []
        for _ in range(n):
            la, lo, raw = antenna_reading(frame, 0, 0, h, rng, noise)
            gga.append({"lat": la, "lon": lo, "fix_q": fix})
            hdg.append(raw)
        return frame, gga, hdg

    def test_good_window_accepted_and_averaged(self):
        frame, gga, hdg = self.window()
        f, why = rover_log.average_mark(gga, hdg, [0, 1, -2])
        self.assertIsNone(why)
        x, y = frame.to_xy(f["lat"], f["lon"])
        # averaged: well inside the 6 mm single-reading noise
        self.assertLess(math.hypot(x, y), 0.004)
        self.assertEqual(f["n_samples"], 30)

    def test_one_float_reading_refuses(self):
        _, gga, hdg = self.window()
        gga[12]["fix_q"] = "5"
        f, why = rover_log.average_mark(gga, hdg, [0])
        self.assertIsNone(f)
        self.assertIn("not RTK fixed", why)

    def test_moving_refuses(self):
        _, gga, hdg = self.window()
        f, why = rover_log.average_mark(gga, hdg, [0, 40, 0])
        self.assertIsNone(f)
        self.assertIn("moving", why)

    def test_scatter_refuses(self):
        _, gga, hdg = self.window(noise=0.05)
        f, why = rover_log.average_mark(gga, hdg, [0])
        self.assertIsNone(f)
        self.assertIn("scattered", why)

    def test_too_few_refuses(self):
        _, gga, hdg = self.window(n=2)
        f, why = rover_log.average_mark(gga, hdg, [0])
        self.assertIsNone(f)

    def test_heading_mean_across_north(self):
        m, sd = vehicle.circular_mean_deg([359.8, 0.2, 359.9, 0.1])
        self.assertAlmostEqual(wrap(m), 0.0, places=6)
        self.assertLess(sd, 0.5)


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


# ============================================================================
#  3. GPS hold
# ============================================================================

class GpsHoldLogic(unittest.TestCase):

    def test_hold_resume(self):
        h = nav.GpsHold()
        self.assertEqual(h.update(0.0, None), "run")
        self.assertEqual(h.update(1.0, "fix quality 5, want 4"), "hold")
        self.assertEqual(h.update(5.0, None), "hold")      # good, not long enough
        self.assertEqual(h.update(6.5, None), "hold")
        self.assertEqual(h.update(7.1, None), "resume")
        self.assertEqual(h.update(7.2, None), "run")
        self.assertEqual(h.count, 1)

    def test_flicker_restarts_good_timer(self):
        h = nav.GpsHold()
        h.update(0.0, "x")
        h.update(1.0, None)
        h.update(2.5, "x")                                 # fell back out
        self.assertEqual(h.update(3.0, None), "hold")
        self.assertEqual(h.update(4.5, None), "hold")      # 1.5 s since good
        self.assertEqual(h.update(5.1, None), "resume")

    def test_timeout(self):
        h = nav.GpsHold()
        h.update(0.0, "x")
        self.assertEqual(h.update(nav.GPS_HOLD_MAX_S + 0.1, "x"), "timeout")


class MissionWithDropout(unittest.TestCase):

    def test_dropout_mid_approach_pauses_and_finishes(self):
        pts, plots = serpentine(rows=2, plots=4)

        def setup(sim):
            # lose the fix 0.6 m short of plot_003, for 8 s
            sim.dropouts.append(
                (lambda s: s.cx < 0.5 and s.cy > 2.0 + 2 * PITCH - 0.6, 8.0))

        rc, rows, done, sim, out = run_mission(pts, plots, setup=setup)
        self.assertEqual(rc, 0, out[-2000:])
        self.assertEqual(len(done), len(pts))
        self.assertIn("HOLD", out)
        self.assertIn("GPS back, resuming", out)
        by = {r["label"]: r for r in rows}
        self.assertGreaterEqual(int(by["plot_003"]["gps_holds"]), 1)
        for r in rows:
            if r["label"].startswith("plot_"):
                self.assertEqual(r["result"], "ok", r)
                self.assertLess(float(r["stop_err_cm"]), 3.0, r)

    def test_fix_never_returns_stops_cleanly_and_resumable(self):
        pts, plots = serpentine(rows=1, plots=4)
        saved = nav.GPS_HOLD_MAX_S
        nav.GPS_HOLD_MAX_S = 20.0

        def setup(sim):
            sim.lost_forever = lambda s: s.cy > 2.0 + PITCH + 0.5

        try:
            rc, rows, done, sim, out = run_mission(pts, plots, setup=setup)
        finally:
            nav.GPS_HOLD_MAX_S = saved
        self.assertEqual(rc, 1)
        self.assertEqual(done, ["plot_001", "plot_002"])
        self.assertIn("GPS lost", rows[-1]["result"])
        self.assertIn("--resume", out)


# ============================================================================
#  4. Per-plot heading
# ============================================================================

class PerPlotHeading(unittest.TestCase):

    def test_taught_heading_no_spins_in_plots(self):
        pts, plots = serpentine(rows=2, plots=4)
        rc, rows, done, sim, out = run_mission(pts, plots)
        self.assertEqual(rc, 0, out[-2000:])
        worst = max(sim.rot_in_plot.values() or [0.0])
        self.assertLess(worst, 10.0, sim.rot_in_plot)
        for r in rows:
            if r["label"].startswith("plot_"):
                self.assertLess(abs(float(r["heading_err_deg"])), 2.5, r)

    def test_single_heading_override_spins_row_two(self):
        """The old behaviour, for contrast: --heading 0 on a serpentine."""
        pts, plots = serpentine(rows=2, plots=4)
        rc, rows, done, sim, out = run_mission(pts, plots,
                                               heading_override=0.0)
        row2 = [sim.rot_in_plot.get(i, 0.0) for i in range(4, 8)]
        self.assertTrue(any(r > 150 for r in row2), row2)

    def test_plan_flags_only_headland(self):
        pts, _ = serpentine(rows=2, plots=4)
        tmp = tempfile.mkdtemp()
        frame = nav.Frame(LAT0, LON0)
        p = os.path.join(tmp, "t.json")
        teach_file(p, frame, pts)
        with redirect_stdout(io.StringIO()):
            marks = nav.load_marks_json(p)
        legs = nav.plan_legs(frame, marks, (0.0, 0.0))
        big = [L["label"] for L in legs if abs(L["turn"]) > nav.BIG_TURN_DEG]
        self.assertEqual(big, ["turn_1b"])


# ============================================================================
#  5. The real rover_log teach loop, against fake serial and CAN
# ============================================================================

def _nmea(body):
    ck = 0
    for c in body:
        ck ^= ord(c)
    return f"${body}*{ck:02X}\r\n".encode()


def _dm(deg, pos, neg):
    h = pos if deg >= 0 else neg
    deg = abs(deg)
    d = int(deg)
    return f"{d * 100 + (deg - d) * 60:.7f}", h


class _FakeLG:
    """LG580P: GGA + PQTMTAR at 10 Hz from a script of (lat, lon, raw, fix)."""

    def __init__(self, world):
        self.world = world
        self.next = 0.0

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def write(self, b):
        pass

    def read(self, n):
        import time as _t
        now = _t.monotonic()
        if now < self.next:
            return b""
        self.next = now + 0.1
        lat, lon, raw, fix = self.world()
        la, lah = _dm(lat, "N", "S")
        lo, loh = _dm(lon, "E", "W")
        gga = _nmea(f"GNGGA,120000.00,{la},{lah},{lo},{loh},{fix},28,0.6,"
                    f"80.0,M,-24.0,M,1.5,0000")
        tar = _nmea(f"PQTMTAR,1,120000.00,4,,1.3315,0.5,,{raw:.4f},0.1,,0.1,26")
        return gga + tar


class _FakeRadio:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def read(self, n):
        return b""


class _FakeCan:
    def __init__(self):
        self.n = 0

    def recv(self, n):
        self.n += 1
        if self.n % 50:
            raise BlockingIOError
        data = struct.pack("<Bhh3x", 5, 0, 0)
        return rover_log.CAN_FRAME.pack(rover_log.COBID_TPDO1, 8, data)

    def close(self):
        pass


class TeachLoop(unittest.TestCase):

    def test_teach_loop_averages_and_refuses(self):
        import time as _t
        import types
        frame = nav.Frame(LAT0, LON0)
        rng = random.Random(11)
        t0 = _t.monotonic()

        def world():
            # fix drops to float from 4.0 s on
            la, lo, raw = antenna_reading(frame, 1.5, 2.5, 47.0, rng, 0.006)
            return la, lo, raw, (5 if _t.monotonic() - t0 > 4.0 else 4)

        keys = [(0.5, "m"), (4.6, "m"), (8.6, "q")]

        class Stdin:
            def fileno(self):
                return 0

            def read(self, n):
                return keys.pop(0)[1]

        def fake_select(r, w, x, timeout=0):
            ready = keys and _t.monotonic() - t0 >= keys[0][0]
            return ([r[0]] if ready else [], [], [])

        tmp = tempfile.mkdtemp()
        saved_state = rover_log.STATE_PATH
        # no forwarder: this test exercises the direct-ports fallback, even
        # on the rover Pi where the real service is running
        rover_log.STATE_PATH = os.path.join(tmp, "no_forwarder.json")
        saved = (rover_log.serial, rover_log.open_can, rover_log.termios,
                 rover_log.tty, rover_log.select, rover_log.find_port,
                 rover_log.OUT_DIR, rover_log.sys.stdin)
        rover_log.serial = types.SimpleNamespace(
            Serial=lambda port, baud, timeout=0:
                _FakeLG(world) if port == "LG" else _FakeRadio())
        rover_log.open_can = lambda: _FakeCan()
        rover_log.termios = types.SimpleNamespace(
            tcgetattr=lambda fd: None, tcsetattr=lambda *a: None, TCSADRAIN=1)
        rover_log.tty = types.SimpleNamespace(setcbreak=lambda fd: None)
        rover_log.select = types.SimpleNamespace(select=fake_select)
        rover_log.find_port = lambda frag: "LG" if "1a86" in frag else "RADIO"
        rover_log.OUT_DIR = tmp
        rover_log.sys.stdin = Stdin()
        out = io.StringIO()
        try:
            with redirect_stdout(out):
                rc = rover_log.cmd_log(argparse.Namespace(command="teach"))
        finally:
            (rover_log.serial, rover_log.open_can, rover_log.termios,
             rover_log.tty, rover_log.select, rover_log.find_port,
             rover_log.OUT_DIR, rover_log.sys.stdin) = saved
            rover_log.STATE_PATH = saved_state

        self.assertEqual(rc, 0)
        self.assertIn("via the ports directly", out.getvalue())
        js = [f for f in os.listdir(tmp) if f.endswith(".json")][0]
        with open(os.path.join(tmp, js)) as fh:
            summary = json.load(fh)
        self.assertEqual(summary["format"], 2)
        self.assertEqual(summary["marks_rejected"], 1)
        self.assertEqual([m["label"] for m in summary["marks"]], ["plot_001"])
        self.assertIn("REFUSED plot_002", out.getvalue())
        with redirect_stdout(io.StringIO()):
            m = nav.load_marks_json(os.path.join(tmp, js))[0]
        x, y = frame.to_xy(m["lat"], m["lon"])
        self.assertLess(math.hypot(x - 1.5, y - 2.5), 0.01)
        self.assertAlmostEqual(m["heading"], 47.0, delta=0.2)


# ============================================================================
#  6. rover_log reading through rtcm_forward - nothing to stop or start
# ============================================================================

class _StateWriter:
    """Plays rtcm_forward: rewrites a state file at 10 Hz GGA + 10 Hz TAR,
    i.e. 20 file writes a second, each GGA value appearing in two files."""

    def __init__(self, path, world):
        import threading
        self.path, self.world = path, world
        self.stop = threading.Event()
        self.t = threading.Thread(target=self.run, daemon=True)
        self.gga_n = self.tar_n = 0
        self.rtcm = 0

    def run(self):
        import time as _t
        g = None
        while not self.stop.is_set():
            lat, lon, raw, fix = self.world()
            self.gga_n += 1
            g = {"lat": lat, "lon": lon, "fix_q": str(fix), "nsat": "28",
                 "hdop": "0.6", "alt_m": "80.0", "corr_age_s": "1.5"}
            for step in ("gga", "tar"):
                if step == "tar":
                    self.tar_n += 1
                self.rtcm += 60
                st = {"t_wall": _t.time(), **g, "heading_deg": raw,
                      "pitch_deg": 0.5, "baseline_m": 1.3315, "head_q": "4",
                      "rtcm_bytes": self.rtcm, "gga_n": self.gga_n,
                      "tar_n": self.tar_n}
                tmp = self.path + ".tmp"
                with open(tmp, "w") as fh:
                    json.dump(st, fh)
                os.replace(tmp, self.path)
                _t.sleep(0.05)

    def __enter__(self):
        self.t.start()
        return self

    def __exit__(self, *a):
        self.stop.set()
        self.t.join()


class ForwarderMode(unittest.TestCase):

    def test_feed_counts_each_reading_once(self):
        import time as _t
        frame = nav.Frame(LAT0, LON0)
        tmp = tempfile.mkdtemp()
        saved = rover_log.STATE_PATH
        rover_log.STATE_PATH = os.path.join(tmp, "state.json")
        w = _StateWriter(rover_log.STATE_PATH,
                         lambda: (*antenna_reading(frame, 0, 0, 10.0), 4))
        try:
            with w:
                _t.sleep(0.2)
                feed, why = rover_log.open_feed()
                self.assertIsNone(why)
                self.assertEqual(feed.name, "rtcm_forward")
                n0 = w.gga_n
                got = 0
                end = _t.monotonic() + 2.0
                while _t.monotonic() < end:
                    got += sum(1 for k, _, _ in feed.poll(_t.monotonic())
                               if k == "gga")
                    _t.sleep(0.005)
                n1 = w.gga_n
        finally:
            rover_log.STATE_PATH = saved
        # each distinct GGA seen once: ~20 in 2 s, never ~40
        self.assertLessEqual(got, n1 - n0 + 1)
        self.assertGreaterEqual(got, n1 - n0 - 2)

    def test_teach_through_forwarder(self):
        import time as _t
        import types
        frame = nav.Frame(LAT0, LON0)
        rng = random.Random(5)
        t0 = _t.monotonic()

        def world():
            la, lo, raw = antenna_reading(frame, -0.7, 3.1, 200.0, rng, 0.006)
            return la, lo, raw, 4

        keys = [(0.6, "n"), (4.4, "q")]

        class Stdin:
            def fileno(self):
                return 0

            def read(self, n):
                return keys.pop(0)[1]

        def fake_select(r, w, x, timeout=0):
            ready = keys and _t.monotonic() - t0 >= keys[0][0]
            return ([r[0]] if ready else [], [], [])

        def no_ports(*a, **k):
            raise AssertionError("opened a serial port while the forwarder "
                                 "was running")

        tmp = tempfile.mkdtemp()
        saved = (rover_log.STATE_PATH, rover_log.serial, rover_log.open_can,
                 rover_log.termios, rover_log.tty, rover_log.select,
                 rover_log.OUT_DIR, rover_log.sys.stdin)
        rover_log.STATE_PATH = os.path.join(tmp, "state.json")
        rover_log.serial = types.SimpleNamespace(Serial=no_ports,
                                                 SerialException=OSError)
        rover_log.open_can = lambda: _FakeCan()
        rover_log.termios = types.SimpleNamespace(
            tcgetattr=lambda fd: None, tcsetattr=lambda *a: None, TCSADRAIN=1)
        rover_log.tty = types.SimpleNamespace(setcbreak=lambda fd: None)
        rover_log.select = types.SimpleNamespace(select=fake_select)
        rover_log.OUT_DIR = tmp
        rover_log.sys.stdin = Stdin()
        out = io.StringIO()
        try:
            with _StateWriter(rover_log.STATE_PATH, world):
                _t.sleep(0.2)
                with redirect_stdout(out):
                    rc = rover_log.cmd_log(argparse.Namespace(command="drive"))
        finally:
            (rover_log.STATE_PATH, rover_log.serial, rover_log.open_can,
             rover_log.termios, rover_log.tty, rover_log.select,
             rover_log.OUT_DIR, rover_log.sys.stdin) = saved

        self.assertEqual(rc, 0)
        self.assertIn("GNSS via rtcm_forward", out.getvalue())
        js = [f for f in os.listdir(tmp)
              if f.startswith("rover_") and f.endswith(".json")][0]
        with open(os.path.join(tmp, js)) as fh:
            summary = json.load(fh)
        self.assertEqual(summary["gnss_source"], "rtcm_forward")
        self.assertGreater(summary["rtcm_bytes"], 0)
        self.assertEqual(len(summary["marks"]), 1)
        m0 = summary["marks"][0]
        self.assertEqual(m0["label"], "plot_001")
        # 3 s at 10 Hz: about 30 distinct readings, not 60
        self.assertLessEqual(m0["n_samples"], 34)
        self.assertGreaterEqual(m0["n_samples"], 20)
        with redirect_stdout(io.StringIO()):
            m = nav.load_marks_json(os.path.join(tmp, js))[0]
        x, y = frame.to_xy(m["lat"], m["lon"])
        self.assertLess(math.hypot(x + 0.7, y - 3.1), 0.01)
        self.assertAlmostEqual(m["heading"], 200.0, delta=0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
