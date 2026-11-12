#!/usr/bin/env python3
"""
test_controller_tuning.py - the 2026-11-12 controller changes, against a
simulated Amiga that misbehaves the way the real one did that day.

    python3 test_controller_tuning.py

The simulator (from test_gps_handling.py) gains two field-measured faults:

  PIVOT SLIP   turning in place walks the machine BACKWARD ~0.28 m per
               radian. Fitted from the 2026-11-12 mission: plot_001 moved
               4.4 cm on a ~9 deg turn, mark_5 43 cm on ~88 deg.
  PIVOT FLOOR  in place, it does not rotate below 200 mrad/s.
  CRAB         rolling, it drifts sideways at 2% of forward speed (leftward,
               calibrated on the 12 Nov route replay; 27 Oct measured
               8 cm over 2 m) with the heading unchanged.
  TURN SCRUB   the wheels follow a turn command after ~0.3 s and ramp to
               it over ~1 s, but on turf only ~10-25% becomes rotation while
               rolling at ~300 mm/s, about the same left and right (three
               yaw-tests 2026-11-12, wheel rate from 0x18E vs GPS heading).
               Modelled as 18%. In place ~25% (mission turns). Jerky: the
               share varies 0.5-1.5x per half second (stick-slip).

and the scenario that went wrong: approaching the first plot of a row
slightly off the line and angled, as plot_001 did.

What it cannot tell you: how the real machine yaws on small commands while
ROLLING. The sim obeys any rate. That is what `amiga_nav.py yaw-test`
measures in the field.
"""

import io
import math
import os
import sys
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import amiga_nav as nav                              # noqa: E402
import test_gps_handling as T                        # noqa: E402

SLIP_M_PER_RAD = 0.28
CRAB = -0.02                    # leftward; calibrated on the 12 Nov route
WHEEL_DELAY_S = 0.1
WHEEL_RAMP_MRAD_S2 = 300.0
EFF_ROLL = {"left": 0.18, "right": 0.18}
EFF_PIVOT = {"left": 0.25, "right": 0.25}
STICK_SLIP_PERIOD_S = 0.5
STICK_RANGE = (0.5, 1.5)        # calibrated on the 12 Nov route replay


class FieldSim(T.SimRover):
    """SimRover plus pivot slip and the in-place rotation floor."""

    def __init__(self, *a, slip=SLIP_M_PER_RAD, pivot_floor=200, crab=CRAB,
                 scrub=True, **k):
        super().__init__(*a, **k)
        self.scrub = scrub
        self.cmd_hist = []            # (t, commanded mrad/s)
        self.wheel = 0.0              # what the wheels are doing, mrad/s
        self.stick = 1.0
        self.next_stick = 0.0
        self.crab = crab
        self.slip = slip
        self.pivot_floor = pivot_floor
        self.max_plot_turn = 0.0
        self.next_fix = 0.0
        self.fix_cache = None

    def poll(self, now):
        # The receiver gives a NEW position 10 times a second, not every
        # control tick - so noise is not averaged away by the loop rate.
        if now >= self.next_fix or self.fix_cache is None:
            super().poll(now)
            self.next_fix = now + 0.1
            self.fix_cache = (self.lat, self.lon, self.heading, now)
        else:
            self._integrate(now)
            self.lat, self.lon, self.heading, t = self.fix_cache
            self.t_pos = self.t_head = t
            self.t_can = now

    def _ground_yaw(self, now, st, lin, ang):
        """Commanded angular -> rotation that actually happens."""
        if not self.scrub:
            if lin == 0 and abs(ang) < self.pivot_floor:
                return 0
            return ang
        cmd = ang if st == nav.STATE_AUTO_ACTIVE else 0
        self.cmd_hist.append((now, cmd))
        target = 0.0
        while self.cmd_hist and self.cmd_hist[0][0] <= now - WHEEL_DELAY_S:
            target = self.cmd_hist[0][1]
            if len(self.cmd_hist) > 1 and \
                    self.cmd_hist[1][0] <= now - WHEEL_DELAY_S:
                self.cmd_hist.pop(0)
            else:
                break
        dt = 0.0 if self.t_last is None else now - self.t_last
        rolling = abs(lin) > 50 and st == nav.STATE_AUTO_ACTIVE
        if rolling:
            # rolling: the turn ramps in over ~1 s (yaw-test wheel rate)
            step = WHEEL_RAMP_MRAD_S2 * dt
            self.wheel += max(-step, min(step, target - self.wheel))
        else:
            # in place: short pulses DID turn it in the field (0.5-1 deg
            # per 150-450 ms pulse on 2026-11-12), so no slow ramp here
            self.wheel = target
        if now >= self.next_stick:
            self.stick = self.rng.uniform(*STICK_RANGE)
            self.next_stick = now + STICK_SLIP_PERIOD_S
        side = "left" if self.wheel > 0 else "right"
        if not rolling and abs(self.wheel) < self.pivot_floor:
            return 0
        eff = (EFF_ROLL if rolling else EFF_PIVOT)[side]
        return self.wheel * eff * self.stick

    def _integrate(self, now):
        st, lin, ang = self.cmd
        self.cmd = (st, lin, self._ground_yaw(now, st, lin, ang))
        h_before = self.h
        t_before = self.t_last
        super()._integrate(now)
        if t_before is not None and st == nav.STATE_AUTO_ACTIVE and lin:
            dt = now - t_before
            th = math.radians(self.h)
            side = self.crab * abs(lin) / 1000.0 * dt      # to the right
            self.cx += side * math.cos(th)
            self.cy -= side * math.sin(th)
        self.cmd = (st, lin, ang)
        dh = abs(T.wrap(self.h - h_before))
        if dh and lin == 0:
            th = math.radians(self.h)
            back = self.slip * math.radians(dh)
            self.cx -= back * math.sin(th)
            self.cy -= back * math.cos(th)


def goto(target_xy, final_heading, start_xy, start_hdg, seed=3, **simkw):
    """Run the real GotoController + drive_loop against FieldSim. Returns
    (true position error in cm along/cross of the final heading, sim, log)."""
    frame = nav.Frame(T.LAT0, T.LON0)
    clock = T.FakeTime()
    sim = FieldSim(clock, frame, start_xy, start_hdg, seed=seed, **simkw)
    saved = nav.time
    nav.time = clock
    out = io.StringIO()
    try:
        sim.open()
        tx, ty = target_xy
        ctl = nav.GotoController(frame, tx, ty, final_heading)
        with redirect_stdout(out):
            why = nav.drive_loop(sim, ctl.step, on_resume=ctl.on_resume)
    finally:
        nav.time = saved
    ex, ey = sim.cx - tx, sim.cy - ty
    along, left = nav.vehicle_frame(ex, ey, final_heading)
    return why, along * 100, left * 100, T.wrap(sim.h - final_heading), \
        ctl, out.getvalue()


class Patch:
    """Temporarily set amiga_nav constants."""

    def __init__(self, **kv):
        self.kv = kv

    def __enter__(self):
        self.old = {k: getattr(nav, k) for k in self.kv}
        for k, v in self.kv.items():
            setattr(nav, k, v)

    def __exit__(self, *a):
        for k, v in self.old.items():
            setattr(nav, k, v)


# The 2026-11-12 plot_001 situation: 5.5 m out, 6 cm off the row line,
# heading 8 deg off it. Row runs at 139 deg.
ROW = 139.0
PLOT = (0.0, 0.0)


def start_pose(back=5.5, off=0.06, angled=8.0):
    th = math.radians(ROW)
    ux, uy = math.sin(th), math.cos(th)            # along the row
    rx, ry = math.cos(th), -math.sin(th)           # to its right
    x = PLOT[0] - back * ux + off * rx
    y = PLOT[1] - back * uy + off * ry
    return (x, y), ROW + angled


OLD_WAY = dict(CROSS_CORRECT_M=0.03, POST_TURN_FIX_TRIES=0,
               ALONG_FILTER_N=1, STEER_MIN_SPEED_MM_S=0, STEER_YAW_GAIN=1.0,
               TURN_PER_SPEED_MAX=99.0, ROW_LINE_FOLLOW=False)

TURF = {"measured": {"left": 0.18, "right": 0.18},
        "weak": {"left": 0.12, "right": 0.12},
        "lopsided": {"left": 0.09, "right": 0.37}}


def first_plot_stats(patch=None, model="measured", seeds=range(1, 9), **pose):
    """Mean / max |along| and |cross| (cm) over seeds, first plot of a row."""
    saved = dict(EFF_ROLL)
    EFF_ROLL.update(TURF[model])
    al, cr = [], []
    try:
        for seed in seeds:
            xy, h = start_pose(**pose)
            with Patch(**(patch or {})):
                why, along, left, herr, ctl, log = goto(PLOT, ROW, xy, h,
                                                        seed=seed)
            assert why == "complete", log[-1500:]
            al.append(abs(along))
            cr.append(abs(left))
    finally:
        EFF_ROLL.update(saved)
    n = len(al)
    return sum(al) / n, max(al), sum(cr) / n, max(cr)


class FirstPlotOfRow(unittest.TestCase):
    """5.5 m out, 6 cm off the line, 8 deg angled - plot_001 on 12 Nov."""

    def test_new_beats_old_on_measured_turf(self):
        old = first_plot_stats(OLD_WAY)
        new = first_plot_stats()
        print(f"\n    old: along {old[0]:.1f}/{old[1]:.1f}  cross "
              f"{old[2]:.1f}/{old[3]:.1f} cm (mean/max)"
              f"\n    new: along {new[0]:.1f}/{new[1]:.1f}  cross "
              f"{new[2]:.1f}/{new[3]:.1f} cm")
        self.assertLess(new[0], old[0])
        self.assertLessEqual(new[2], old[2])
        self.assertLessEqual(new[1], 2.0)
        self.assertLessEqual(new[3], 2.5)

    def test_not_worse_if_turf_behaves_differently(self):
        for model in ("weak", "lopsided"):
            old = first_plot_stats(OLD_WAY, model)
            new = first_plot_stats(None, model)
            self.assertLessEqual(new[0], old[0] + 0.2, model)
            self.assertLessEqual(new[2], old[2] + 0.2, model)

    def test_harder_start(self):
        """30 cm off the line and 15 deg angled still converges."""
        a_mean, a_max, c_mean, c_max = first_plot_stats(
            None, back=5.5, off=0.30, angled=-15.0, seeds=range(1, 5))
        self.assertLessEqual(a_max, 2.0)
        self.assertLess(c_max, 4.0)

    def test_row_line_follow_is_off(self):
        self.assertFalse(nav.ROW_LINE_FOLLOW)


class PostTurnFix(unittest.TestCase):

    def test_turn_slip_is_crept_back(self):
        """Arrive with a forced 10 deg final turn: the pivot walks it back
        ~5 cm, the check sees it, creeps forward, ends within 1.5 cm."""
        xy, _ = start_pose(back=5.5, off=0.0, angled=0.0)
        # mark taught 10 deg off the row: the approach is in-row, but a
        # 10 deg final turn is unavoidable
        why, along, left, herr, ctl, log = goto(PLOT, ROW + 10.0, xy, ROW)
        self.assertEqual(why, "complete", log[-2000:])
        self.assertIn("fix     along", log)          # it crept to fix it
        m = [l for l in log.splitlines() if "squared up" in l][0]
        after = float(m.split("after turn ")[1].split(" cm")[0])
        print(f"\n    10 deg final turn: slipped {after:.1f} cm, "
              f"crept back to along {along:+.1f} cm")
        self.assertGreater(after, nav.POST_TURN_FIX_M * 100)   # slip was real
        self.assertLessEqual(abs(along), 2.0, log[-2000:])

    def test_large_headland_slip_reported_not_chased(self):
        """An 88 deg headland turn slips ~43 cm: beyond POST_TURN_FIX_MAX_M,
        so it is reported and the leg ends; nothing drives 40 cm blind."""
        th = math.radians(ROW)
        start = (-5.0 * math.sin(th), -5.0 * math.cos(th))
        why, along, left, herr, ctl, log = goto(PLOT, ROW - 88.0, start, ROW)
        self.assertEqual(why, "complete", log[-2000:])
        self.assertIn("too large to creep-fix", log)


class RollingFloor(unittest.TestCase):

    def _sent(self, lin, ang, **patch):
        r = nav.Rover()
        got = {}

        class S:
            def send(self, raw):
                cid, dlc, data = nav.CAN_FRAME.unpack(raw)
                got["v"] = nav.struct.unpack_from("<Bhh", data)

        r.sock = S()
        with Patch(**patch):
            r.send(nav.STATE_AUTO_ACTIVE, lin, ang)
        return got["v"][2]

    def test_default_unchanged(self):
        self.assertEqual(self._sent(300, 40), 220)
        self.assertEqual(self._sent(0, 40), 220)

    def test_rolling_floor_applies_only_when_rolling(self):
        kw = dict(ANGULAR_MIN_ROLLING_MRAD_S=60)
        self.assertEqual(self._sent(300, 40, **kw), 60)
        self.assertEqual(self._sent(300, -90, **kw), -90)
        self.assertEqual(self._sent(0, 40, **kw), 220)     # in place: 220
        self.assertEqual(self._sent(50, 40, **kw), 220)    # creep: 220


class YawTestPlumbing(unittest.TestCase):

    def test_fit_recovers_commanded_rate(self):
        """In the sim the machine obeys exactly, so yaw-test must report
        commanded ~= achieved, with the right signs."""
        frame = nav.Frame(T.LAT0, T.LON0)
        clock = T.FakeTime()
        sim = FieldSim(clock, frame, (0.0, 0.0), 90.0, hdg_noise=0.05,
                       scrub=False)
        sim.open()
        sim.angular_floor = False
        yt = nav.YawTest(300, [40, 80, 160], 2.0)
        saved = nav.time
        nav.time = clock
        try:
            with redirect_stdout(io.StringIO()):
                why = nav.drive_loop(sim, yt.step, on_resume=yt.on_resume)
                floor = nav.yaw_report(yt.results)
        finally:
            nav.time = saved
        self.assertEqual(why, "complete")
        self.assertEqual([c for c, _ in yt.results], [40, -80, 160])
        for cmd, got in yt.results:
            self.assertAlmostEqual(got / cmd, 1.0, delta=0.12)
        self.assertEqual(floor, 40)


class SerpentineWithSlip(unittest.TestCase):

    def test_two_rows(self):
        """2 rows x 4 plots with a headland turn, measured turf response."""
        pts, plots = T.serpentine(rows=2, plots=4)
        saved = T.SimRover
        T.SimRover = FieldSim
        try:
            rc, rows, done, sim, out = T.run_mission(pts, plots)
        finally:
            T.SimRover = saved
        self.assertEqual(rc, 0, out[-2000:])
        pl = [r for r in rows if r["label"].startswith("plot_")]
        wa = max(abs(float(r["along_cm"])) for r in pl)
        wc = max(abs(float(r["cross_cm"])) for r in pl)
        print(f"\n    serpentine: worst along {wa:.1f} cm, worst cross "
              f"{wc:.1f} cm (GPS-reported)")
        self.assertLessEqual(wa, 2.0)
        self.assertLess(wc, 4.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
