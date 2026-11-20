#!/usr/bin/env python3
"""
test_teach.py - the teach logic shared by rover_log.py and teach_gui.py.

    python3 test_teach.py

No hardware. A fake GNSS feed and a fake CAN socket stand in for the
forwarder and the Amiga, on a fake clock. The last test opens the real
teach window under a virtual display if one is available (xvfb-run), and is
skipped otherwise.
"""

import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["AMIGA_PROFILE"] = "none"      # never a machine's profile in tests

import amiga_nav as nav                          # noqa: E402
import rover_log                                 # noqa: E402
from rover_log import TeachSession, MARK_AVG_S   # noqa: E402


class FakeFeed:
    name = "fake"

    def __init__(self):
        self.rtcm_bytes = 0
        self.lat, self.lon = 32.7767, -96.7970
        self.fix = "4"
        self.next_t = 0.0
        self.k = 0

    def poll(self, now):
        if now < self.next_t:
            return []
        self.next_t = now + 0.1
        self.k += 1
        jitter = ((self.k * 37) % 7 - 3) * 1e-8          # ~3 mm
        return [("gga", {"lat": self.lat + jitter, "lon": self.lon - jitter,
                         "fix_q": self.fix, "nsat": "28", "hdop": "0.6",
                         "alt_m": "150", "corr_age_s": "1.0"}, 0.0),
                ("tar", {"head_q": "4", "baseline_m": 1.33, "pitch_deg": 0.1,
                         "heading_deg": 270.0 + jitter * 1e6,
                         "head_nsat": "20"}, 0.0)]

    def close(self):
        pass


class FakeCan:
    def __init__(self):
        self.speed = 0
        self.ready = True

    def recv(self, n):
        if not self.ready:
            self.ready = True
            raise BlockingIOError
        self.ready = False
        data = struct.pack("<Bhh", 4, self.speed, 0) + b"\0\0" + bytes([80])
        return rover_log.CAN_FRAME.pack(rover_log.COBID_TPDO1, 8, data)

    def close(self):
        pass


class Teach(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.feed, self.can = FakeFeed(), FakeCan()
        self.s = TeachSession(self.feed, self.can, cols=3, start_row=1,
                              out_dir=self.dir, stamp="test")
        self.t = 100.0

    def tearDown(self):
        self.s.close()
        shutil.rmtree(self.dir, ignore_errors=True)

    def run_for(self, secs):
        ev = []
        end = self.t + secs
        while self.t < end:
            self.t += 0.02
            ev += self.s.poll(self.t)
        return ev

    def mark(self, kind):
        self.run_for(0.3)
        self.s.mark(kind, self.t)
        ev = self.run_for(MARK_AVG_S + 0.2)
        self.feed.lat += 1.22 / 111_320.0
        return [e for e in ev if e[0] in ("mark", "refused")]

    def saved(self):
        with open(self.s.json_path) as fh:
            return json.load(fh)

    def test_plot_named_by_map(self):
        ev = self.mark("plot")
        self.assertEqual(ev[0][0], "mark")
        m = ev[0][2]
        self.assertEqual((m["label"], m["kind"], m["row"], m["col"]),
                         ("R1_C1", "plot", 1, 1))
        self.assertEqual(self.s.next_plot(), "R1_C2")

    def test_saved_after_every_mark_and_mission_reads_it(self):
        self.mark("plot")
        self.mark("plot")
        self.assertEqual([m["label"] for m in self.saved()["marks"]],
                         ["R1_C1", "R1_C2"])
        marks = nav.load_marks_json(self.s.json_path)
        self.assertEqual([m["label"] for m in marks], ["R1_C1", "R1_C2"])

    def test_headland_does_not_use_a_plot_name(self):
        self.mark("plot")
        ev = self.mark("headland")
        self.assertEqual(ev[0][2]["label"], "mark_2")
        self.assertEqual(ev[0][2]["kind"], "headland")
        self.assertEqual(self.s.next_plot(), "R1_C2")

    def test_refused_float_keeps_the_name(self):
        self.feed.fix = "5"
        ev = self.mark("plot")
        self.assertEqual(ev[0][0], "refused")
        self.assertIn("not RTK fixed", ev[0][1])
        self.assertEqual(self.s.next_plot(), "R1_C1")
        self.assertEqual(self.s.marks, [])

    def test_refused_when_moving(self):
        self.can.speed = 120
        ev = self.mark("plot")
        self.assertEqual(ev[0][0], "refused")
        self.assertIn("moving", ev[0][1])

    def test_serpentine_names(self):
        names = []
        for _ in range(4):
            names.append(self.mark("plot")[0][2]["label"])
        self.assertEqual(names, ["R1_C1", "R1_C2", "R1_C3", "R2_C3"])

    def test_skip_then_undo_restores_names(self):
        self.mark("plot")                  # R1_C1
        self.assertEqual(self.s.skip(), "R1_C2")
        self.assertEqual(self.saved()["grid"]["skipped"], ["R1_C2"])
        self.mark("plot")                  # R1_C3
        self.assertEqual(self.s.undo(self.t), ("mark", "R1_C3"))
        self.assertEqual(self.s.next_plot(), "R1_C3")
        self.assertEqual(self.s.undo(self.t), ("skip", "R1_C2"))
        self.assertEqual(self.s.next_plot(), "R1_C2")
        self.assertEqual(self.s.undo(self.t), ("mark", "R1_C1"))
        self.assertEqual(self.s.next_plot(), "R1_C1")
        self.assertIsNone(self.s.undo(self.t))
        self.assertEqual(self.saved()["marks"], [])

    def test_undo_mark_after_skip_names_correctly(self):
        # The old terminal loop stepped the name back by one here and
        # handed out R1_C2 for a plot that was R1_C1.
        self.mark("plot")                  # R1_C1
        self.s.skip()                      # R1_C2
        self.s.undo(self.t)                # undoes the skip
        self.s.undo(self.t)                # undoes R1_C1
        self.assertEqual(self.s.next_plot(), "R1_C1")
        self.assertEqual(self.mark("plot")[0][2]["label"], "R1_C1")

    # -- coming back for a plot (click in the window) --------------------
    def test_pick_skipped_plot_then_order_carries_on(self):
        self.mark("plot")                          # R1_C1
        self.s.skip()                              # R1_C2 skipped
        self.mark("plot")                          # R1_C3
        self.assertIsNone(self.s.pick(1, 2))
        self.assertEqual(self.s.next_plot(), "R1_C2")
        m = self.mark("plot")[0][2]
        self.assertEqual((m["label"], m["row"], m["col"], m["out_of_order"]),
                         ("R1_C2", 1, 2, True))
        self.assertEqual(self.s.skipped, [])
        self.assertIsNone(self.s.picked)
        self.assertEqual(self.s.next_plot(), "R2_C3")      # back in order
        saved = self.saved()
        self.assertEqual([x["label"] for x in saved["marks"]],
                         ["R1_C1", "R1_C3", "R1_C2"])
        self.assertEqual(saved["grid"]["skipped"], [])

    def test_order_steps_over_a_plot_taught_early(self):
        self.mark("plot")                          # R1_C1
        self.s.pick(1, 3)
        self.mark("plot")                          # R1_C3, early
        self.assertEqual(self.s.next_plot(), "R1_C2")
        self.mark("plot")                          # R1_C2
        self.assertEqual(self.s.next_plot(), "R2_C3")      # not R1_C3 again

    def test_undo_picked_plot_goes_back_to_skipped(self):
        self.s.skip()                              # R1_C1 skipped
        self.mark("plot")                          # R1_C2
        self.s.pick(1, 1)
        self.mark("plot")                          # R1_C1, picked
        self.assertEqual(self.s.undo(self.t), ("mark", "R1_C1"))
        self.assertEqual(self.s.skipped, ["R1_C1"])
        self.assertEqual(self.s.next_plot(), "R1_C3")

    def test_pick_refusals_and_cancel(self):
        self.mark("plot")                          # R1_C1
        self.assertIn("already taught", self.s.pick(1, 1))
        self.assertIn("not in the map", self.s.pick(1, 9))
        self.assertIsNone(self.s.pick(1, 2))       # the normal next anyway
        self.assertIsNone(self.s.picked)
        self.s.pick(2, 2)
        self.assertIsNone(self.s.skip())           # s cancels the pick ...
        self.assertIsNone(self.s.picked)
        self.assertEqual(self.s.skipped, [])       # ... and skips nothing
        self.s.pick(2, 2)
        self.s.pick(None, None)
        self.assertEqual(self.s.next_plot(), "R1_C2")

    def test_refused_picked_mark_keeps_the_pick(self):
        self.s.pick(2, 1)
        self.feed.fix = "5"
        self.assertEqual(self.mark("plot")[0][0], "refused")
        self.assertEqual(self.s.next_plot(), "R2_C1")

    def test_mission_reads_out_of_order_file(self):
        self.mark("plot")
        self.s.skip()
        self.mark("plot")
        self.s.pick(1, 2)
        self.mark("plot")
        marks = nav.load_marks_json(self.s.json_path)
        self.assertEqual([m["label"] for m in marks], ["R1_C1", "R1_C3", "R1_C2"])

    def test_no_actions_while_averaging(self):
        self.run_for(0.3)
        self.s.mark("plot", self.t)
        self.assertIsNone(self.s.mark("plot", self.t))
        self.assertIsNone(self.s.skip())
        self.assertIsNone(self.s.undo(self.t))

    def test_plain_teach_without_grid_unchanged(self):
        s = TeachSession(FakeFeed(), FakeCan(), cols=None, out_dir=self.dir,
                         stamp="plain")
        t = 10.0
        s.mark("headland", t)            # 'm' in plain teach = a plot, as before
        ev = []
        while t < 10.0 + MARK_AVG_S + 0.3:
            t += 0.02
            ev += s.poll(t)
        s.close()
        self.assertEqual(ev[0][2]["label"], "plot_001")
        self.assertIsNone(s.summary()["grid"])


class Window(unittest.TestCase):
    """The real teach window, demo mode, under xvfb-run if installed."""

    def test_demo_window_marks_and_saves(self):
        if not shutil.which("xvfb-run"):
            self.skipTest("xvfb-run not installed")
        here = os.path.dirname(os.path.abspath(__file__))
        script = os.path.join(tempfile.mkdtemp(), "drive_gui.py")
        with open(script, "w") as fh:
            fh.write(DRIVE_GUI)
        r = subprocess.run(["xvfb-run", "-a", sys.executable, script, here],
                           capture_output=True, text=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        result = json.loads(r.stdout.strip().splitlines()[-1])
        # t t n(nothing) t m m t s t, then click the skipped R2_C2, t
        self.assertEqual(result["labels"],
                         ["R1_C1", "R1_C2", "R1_C3", "mark_4", "mark_5",
                          "R2_C3", "R2_C1", "R2_C2"])
        self.assertEqual(result["skipped_before_click"], ["R2_C2"])
        self.assertEqual(result["skipped"], [])
        self.assertEqual(result["turns"], {"1": 2})


DRIVE_GUI = r'''
import json, sys, time, tkinter as tk
sys.path.insert(0, sys.argv[1])
import teach_gui
from tkinter import messagebox
messagebox.showinfo = lambda *a, **k: None
args = teach_gui.argparse.Namespace(map=None, maps_file=None, start_row=1,
                                    rows=2, cols=3, demo=True)
root = tk.Tk()
w = teach_gui.TeachWindow(root, args)
keys = list("ttnt") + list("mmtst") + ["click R2_C2", "t"]
seen = {}
class Ev:
    def __init__(self, x, y): self.x, self.y = x, y
def step():
    if w.s.pending is not None:
        root.after(100, step); return
    if not keys:
        out = {"labels": [m["label"] for m in w.s.marks],
               "skipped": w.s.skipped,
               "skipped_before_click": seen.get("skipped"),
               "turns": {str(k): v for k, v in w.turn_counts().items()}}
        w.s.close(); root.destroy(); print(json.dumps(out)); return
    k = keys.pop(0)
    if k.startswith("click"):
        seen["skipped"] = list(w.s.skipped)
        x0, y0, cw, ch, nrows = w.geom
        w.on_click(Ev(x0 + 1.5 * cw, y0 + 1.5 * ch))       # row 2, col 2
    else:
        w.do(k)
    root.after(300, step)
root.after(500, step)
root.mainloop()
'''


if __name__ == "__main__":
    unittest.main(verbosity=2)
