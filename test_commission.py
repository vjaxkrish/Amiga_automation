#!/usr/bin/env python3
"""
test_commission.py - commission.py against simulated machines whose true
properties are hidden in sim_amiga.py, and the machine profile it writes.

    python3 test_commission.py

Each test runs the real commissioning steps (driving through the real
drive_loop) on a simulated Amiga, then checks the script found what was
hidden: heading offset, antenna position forward and sideways, breakaway
speeds, turn efficiency, a curve radius it can hold - and that the profile
it writes is what vehicle.py / amiga_nav.py then run on.
"""

import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["AMIGA_PROFILE"] = "none"      # never a machine's profile in tests

import amiga_nav as nav                    # noqa: E402
import vehicle                             # noqa: E402

KEEP = ["HEADING_OFFSET_DEG", "ANTENNA_OFFSET_M", "ANTENNA_LATERAL_M"]
NAV_KEEP = ["CREEP_SPEED_MM_S", "ANGULAR_MIN_MRAD_S", "TURN_SLOW_MRAD_S",
            "TURN_IN_PLACE_MRAD_S", "ARC_YAW_EFFICIENCY",
            "ANGULAR_MIN_ROLLING_MRAD_S", "ANG_SLEW_MRAD_S2",
            "LIN_ACCEL_MM_S2", "ARC_RADIUS_M", "ARC_SPEED_MM_S",
            "HEADLAND_TURN"]


class Commissioning(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.saved_v = {k: getattr(vehicle, k) for k in KEEP}
        self.saved_n = {k: getattr(nav, k) for k in NAV_KEEP}
        self.saved_time = nav.time
        os.environ["TURF_NAV_LOGS"] = self.dir
        import commission
        commission.LOG_DIR = self.dir
        self.c = commission

    def tearDown(self):
        for k, v in self.saved_v.items():
            setattr(vehicle, k, v)
        for k, v in self.saved_n.items():
            setattr(nav, k, v)
        nav.time = self.saved_time
        os.environ.pop("TURF_NAV_LOGS", None)
        shutil.rmtree(self.dir, ignore_errors=True)

    def run_all(self, truth):
        res = os.path.join(self.dir, "results.json")
        prof = os.path.join(self.dir, "profile.json")
        out = io.StringIO()
        with redirect_stdout(out):
            rc = self.c.main(["--sim", "--yes", "--fresh", "--machine", "t",
                              "--results", res, "--profile", prof,
                              "--sim-truth", json.dumps(truth), "all"])
        self.assertEqual(rc, 0, out.getvalue()[-3000:])
        with open(res) as fh:
            results = json.load(fh)
        with open(prof) as fh:
            profile = json.load(fh)
        return results, profile, out.getvalue()

    def test_default_dallas_guess(self):
        truth = {}                              # sim_amiga.DEFAULT_TRUTH
        r, prof, out = self.run_all(truth)
        rec = r["recommend"]
        self.assertAlmostEqual(rec["HEADING_OFFSET_DEG"], 180.0, delta=0.5)
        self.assertAlmostEqual(rec["ANTENNA_OFFSET_M"], 0.62, delta=0.01)
        self.assertAlmostEqual(rec["ANTENNA_LATERAL_M"], 0.04, delta=0.01)
        self.assertAlmostEqual(r["steps"]["centre"]["walk_m_per_rad"], 0.10,
                               delta=0.02)
        self.assertAlmostEqual(rec["ARC_YAW_EFFICIENCY"], 0.55, delta=0.05)
        self.assertEqual(r["steps"]["breakaway"]["lin_floor_mm_s"], 30)
        self.assertEqual(r["steps"]["breakaway"]["rot_floor_mrad_s"], 160)
        self.assertEqual(rec["HEADLAND_TURN"], "arc")
        # the course: every fake plot within 3 cm by GPS
        plots = r["steps"]["course"]["plots"]
        self.assertEqual(len([p for p in plots if p["label"].startswith("R")]), 8)
        for p in plots:
            if p["label"].startswith("R"):
                self.assertLess(abs(float(p["along_cm"])), 3.0, p)
                self.assertLess(abs(float(p["cross_cm"])), 3.0, p)
        # profile layout
        self.assertEqual(prof["machine"], "t")
        self.assertAlmostEqual(prof["antenna_offset_m"], 0.62, delta=0.01)
        self.assertIn("ARC_RADIUS_M", prof["nav"])

    def test_left_right_mast_heavy_machine(self):
        truth = {"heading_offset_deg": 90.0, "antenna_fwd_m": 0.30,
                 "antenna_lat_m": -0.25, "roll_eff": 0.30, "pivot_eff": 0.45,
                 "pivot_floor_mrad_s": 240, "lin_floor_mm_s": 45,
                 "pivot_walk_m_per_rad": 0.2}
        r, prof, out = self.run_all(truth)
        rec = r["recommend"]
        self.assertAlmostEqual(rec["HEADING_OFFSET_DEG"], 90.0, delta=0.5)
        self.assertIn("LEFT-RIGHT", out)
        self.assertAlmostEqual(rec["ANTENNA_OFFSET_M"], 0.30, delta=0.01)
        self.assertAlmostEqual(rec["ANTENNA_LATERAL_M"], -0.25, delta=0.01)
        self.assertGreaterEqual(rec["CREEP_SPEED_MM_S"], 45)
        self.assertGreaterEqual(rec["ANGULAR_MIN_MRAD_S"], 240)
        self.assertAlmostEqual(rec["ARC_YAW_EFFICIENCY"], 0.30, delta=0.04)
        # turn command it would need stays inside the 600 mrad/s limit
        need = rec["ARC_SPEED_MM_S"] / rec["ARC_RADIUS_M"] / \
            rec["ARC_YAW_EFFICIENCY"]
        self.assertLess(need, nav.MAX_ANGULAR_MRAD_S)

    def test_turf_that_cannot_curve_falls_back_to_pivots(self):
        truth = {"roll_eff": 0.06}
        res = os.path.join(self.dir, "r.json")
        out = io.StringIO()
        with redirect_stdout(out):
            rc = self.c.main(["--sim", "--yes", "--fresh", "--machine", "t",
                              "--results", res, "--radii", "1.5",
                              "--sim-truth", json.dumps(truth), "arc"])
        self.assertEqual(rc, 0, out.getvalue()[-2000:])
        with open(res) as fh:
            r = json.load(fh)
        self.assertEqual(r["recommend"]["HEADLAND_TURN"], "pivot")

    def test_swing_ramp_found(self):
        res = os.path.join(self.dir, "r.json")
        out = io.StringIO()
        with redirect_stdout(out):
            rc = self.c.main(["--sim", "--yes", "--fresh", "--machine", "t",
                              "--results", res, "swing"])
        self.assertEqual(rc, 0, out.getvalue()[-2000:])
        with open(res) as fh:
            r = json.load(fh)
        turns = r["steps"]["swing"]["turns"]
        instant = [t for t in turns if t[0] is None][0]
        gentle = [t for t in turns if t[0] == 400.0][0]
        self.assertGreater(instant[1], 3 * gentle[1])     # ramps do help
        self.assertIsNotNone(r["recommend"]["ANG_SLEW_MRAD_S2"])

    def test_steps_accumulate_across_runs(self):
        res = os.path.join(self.dir, "r.json")
        with redirect_stdout(io.StringIO()):
            self.c.main(["--sim", "--yes", "--fresh", "--results", res,
                         "--machine", "t",
                         "--sim-truth", '{"heading_offset_deg": 170}',
                         "heading"])
            # a later run, a fresh process in the field: the measured
            # heading offset is used by the next step
            vehicle.HEADING_OFFSET_DEG = 180.0
            out = io.StringIO()
            with redirect_stdout(out):
                self.c.main(["--sim", "--yes", "--results", res,
                             "--machine", "t",
                             "--sim-truth", '{"heading_offset_deg": 170}',
                             "centre"])
        with open(res) as fh:
            r = json.load(fh)
        self.assertIn("heading", r["steps"])
        self.assertIn("centre", r["steps"])
        self.assertAlmostEqual(r["recommend"]["ANTENNA_OFFSET_M"], 0.62,
                               delta=0.01)


class Profile(unittest.TestCase):

    def setUp(self):
        self.saved_v = {k: getattr(vehicle, k) for k in KEEP}
        self.saved_n = {k: getattr(nav, k) for k in NAV_KEEP}

    def tearDown(self):
        for k, v in self.saved_v.items():
            setattr(vehicle, k, v)
        for k, v in self.saved_n.items():
            setattr(nav, k, v)
        vehicle.PROFILE, vehicle.NAV_OVERRIDES = {}, {}

    def test_profile_drives_both_modules(self):
        d = tempfile.mkdtemp()
        p = os.path.join(d, "v.json")
        with open(p, "w") as fh:
            json.dump({"machine": "dallas", "heading_offset_deg": 90.0,
                       "antenna_offset_m": 0.31, "antenna_lateral_m": -0.2,
                       "nav": {"ARC_RADIUS_M": 1.7, "NOT_A_SETTING": 1}}, fh)
        vehicle.load_profile(p)
        import contextlib
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            unknown = nav.apply_profile()
        self.assertEqual(unknown, ["NOT_A_SETTING"])
        self.assertIn("NOT_A_SETTING", err.getvalue())
        self.assertEqual(nav.ARC_RADIUS_M, 1.7)
        self.assertEqual(vehicle.true_heading(0.0), 90.0)
        self.assertIn("dallas", vehicle.describe())

    def test_teach_and_drive_agree_with_sideways_antenna(self):
        """A mark taught with the antenna off the centreline is the point
        the controller drives to - the lateral offset is applied once."""
        vehicle.ANTENNA_OFFSET_M, vehicle.ANTENNA_LATERAL_M = 0.3, -0.25
        frame = nav.Frame(32.0, -96.0)
        for hdg in (0.0, 37.0, 90.0, 200.0):
            lat, lon = frame.to_ll(3.0, 4.0)
            clat, clon = vehicle.rotation_centre(lat, lon, hdg)
            cx, cy = frame.to_xy(clat, clon)
            px, py = nav.control_point(frame, lat, lon, hdg)
            self.assertAlmostEqual(cx, px, places=4)
            self.assertAlmostEqual(cy, py, places=4)
            # and it is where a 0.3 fwd / 0.25 right antenna puts it
            f, left = nav.vehicle_frame(3.0 - px, 4.0 - py, hdg)
            self.assertAlmostEqual(f, 0.3, places=3)
            self.assertAlmostEqual(left, -0.25, places=3)

    def test_no_profile_means_built_in(self):
        old = os.environ.get("AMIGA_PROFILE")
        os.environ["AMIGA_PROFILE"] = "none"
        try:
            self.assertIsNone(vehicle.profile_path())
        finally:
            if old is not None:
                os.environ["AMIGA_PROFILE"] = old


if __name__ == "__main__":
    unittest.main(verbosity=2)
