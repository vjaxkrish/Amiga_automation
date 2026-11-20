#!/usr/bin/env python3
"""
test_capture_link.py - offline tests for the 2026-11-19 capture integration.

    python3 test_capture_link.py

No hardware, no camera app. The mission tests run the real cmd_mission
against the simulated Amiga from test_gps_handling.py, with a stand-in camera
app on a real Unix socket (the real CaptureServer, answering from a thread),
so the whole conversation - hello, status, capture, result, bye - goes over
the same wire as in the field.

What each group proves:
  Plot names     teach labels R{row}_C{col} in serpentine order, skip, undo
  Link           messages cross the socket; one mission at a time
  Mission        every plot is captured once, headlands never; a bad answer,
                 a slow answer or a missing camera app flags or stops the run
                 and always leaves the machine stopped
"""

import argparse
import io
import json
import os
import socket
import sys
import tempfile
import threading
import time as realtime
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["AMIGA_PROFILE"] = "none"      # never a machine's profile in tests

import amiga_nav as nav                          # noqa: E402
import capture_link                              # noqa: E402
import rover_log                                 # noqa: E402
import test_gps_handling as sim                  # noqa: E402


# ============================================================================
#  Plot names
# ============================================================================

class PlotNames(unittest.TestCase):

    def test_parse(self):
        self.assertEqual(capture_link.parse_plot_label("R3_C7"), (3, 7))
        self.assertEqual(capture_link.parse_plot_label("R12_C10"), (12, 10))
        for bad in ("mark_4", "plot_001", "R3C7", "r3_c7", "", None,
                    "R3_C7_x"):
            self.assertIsNone(capture_link.parse_plot_label(bad), bad)

    def test_serpentine(self):
        lab = rover_log.PlotLabeler(cols=3)
        seq = []
        for _ in range(9):
            seq.append(lab.current())
            lab.advance()
        self.assertEqual(seq, ["R1_C1", "R1_C2", "R1_C3",
                               "R2_C3", "R2_C2", "R2_C1",
                               "R3_C1", "R3_C2", "R3_C3"])

    def test_start_row_even_runs_backwards(self):
        lab = rover_log.PlotLabeler(cols=4, start_row=2)
        self.assertEqual(lab.current(), "R2_C4")

    def test_back_undoes_advance(self):
        lab = rover_log.PlotLabeler(cols=3)
        for _ in range(4):
            lab.advance()
        self.assertEqual(lab.current(), "R2_C2")
        lab.back()
        lab.back()
        self.assertEqual(lab.current(), "R1_C3")


# ============================================================================
#  The link itself
# ============================================================================

def temp_socket():
    d = tempfile.mkdtemp()
    return os.path.join(d, "cap.sock")


class Link(unittest.TestCase):

    def setUp(self):
        self.path = temp_socket()
        self.srv = capture_link.CaptureServer(self.path)
        self.srv.start()

    def tearDown(self):
        self.srv.stop()

    def get(self, timeout=2.0):
        return self.srv.inbox.get(timeout=timeout)

    def test_round_trip(self):
        c = capture_link.CaptureClient(self.path)
        c.connect()
        self.assertEqual(self.get()[1]["type"], "_connected")
        rid = c.request_capture("R1_C2", 1, 2, "auto", {"gps_ok": True})
        no, msg = self.get()
        self.assertEqual((msg["type"], msg["label"], msg["nav"]["gps_ok"]),
                         ("capture", "R1_C2", True))
        self.srv.reply(no, {"type": "result", "id": rid, "status": "ok"})
        got = []
        t_end = realtime.monotonic() + 2
        while not got and realtime.monotonic() < t_end:
            got = c.poll()
            realtime.sleep(0.01)
        self.assertEqual(got[0]["status"], "ok")
        c.close()
        self.assertEqual(self.get()[1]["type"], "_disconnected")

    def test_poll_never_blocks(self):
        c = capture_link.CaptureClient(self.path)
        c.connect()
        t0 = realtime.monotonic()
        self.assertEqual(c.poll(), [])
        self.assertLess(realtime.monotonic() - t0, 0.05)
        c.close()

    def test_second_mission_refused(self):
        a = capture_link.CaptureClient(self.path)
        a.connect()
        self.get()
        b = capture_link.CaptureClient(self.path)
        b.connect()
        msgs = []
        t_end = realtime.monotonic() + 2
        while realtime.monotonic() < t_end:
            try:
                msgs += b.poll()
            except capture_link.LinkClosed:
                break
            realtime.sleep(0.01)
        self.assertTrue(msgs and msgs[0]["ok"] is False)
        a.close()
        b.close()

    def test_app_gone_is_seen(self):
        c = capture_link.CaptureClient(self.path)
        c.connect()
        self.get()
        self.srv.stop()
        with self.assertRaises(capture_link.LinkClosed):
            t_end = realtime.monotonic() + 2
            while realtime.monotonic() < t_end:
                c.poll()
                realtime.sleep(0.01)


# ============================================================================
#  Missions against a stand-in camera app
# ============================================================================

class PacedTime(sim.FakeTime):
    """The fake clock, but each control tick also lets a little real time
    pass, so the app thread on the other end of the socket gets to answer."""

    def sleep(self, dt):
        super().sleep(dt)
        realtime.sleep(0.0001)


class FakeApp:
    """Answers like the camera app. `answer(label)` returns a status, or
    None for never answering; `refuse` makes hello fail."""

    def __init__(self, path, answer=lambda label: "ok", refuse=None,
                 die_at=None):
        self.srv = capture_link.CaptureServer(path)
        self.answer, self.refuse, self.die_at = answer, refuse, die_at
        self.captures, self.statuses, self.hello = [], [], None
        self.stop = threading.Event()
        self.srv.start()
        self.t = threading.Thread(target=self.run, daemon=True)
        self.t.start()

    def run(self):
        import queue
        while not self.stop.is_set():
            try:
                no, msg = self.srv.inbox.get(timeout=0.05)
            except queue.Empty:
                continue
            kind = msg.get("type")
            if kind == "hello":
                self.hello = msg
                self.srv.reply(no, {"type": "hello_ok", "ok": not self.refuse,
                                    "invalid": self.refuse or [],
                                    "map": "test", "warmup_remaining_s": 0,
                                    "detail": "" if not self.refuse
                                    else "not in map"})
            elif kind == "status":
                self.statuses.append(msg)
            elif kind == "capture":
                self.captures.append(msg)
                if msg["label"] == self.die_at:
                    self.srv.stop()
                    return
                status = self.answer(msg["label"])
                if status is not None:
                    self.srv.reply(no, {"type": "result", "id": msg["id"],
                                        "status": status, "detail": "",
                                        "folder": msg["label"]})

    def close(self):
        self.stop.set()
        self.srv.stop()


def grid_route(rows=2, cols=3):
    """Serpentine with R/C labels and two headland points, like a teach
    with --cols."""
    pts, plot_xy = [], []
    for r in range(rows):
        x = r * sim.PITCH
        north = r % 2 == 0
        cs = list(range(1, cols + 1)) if north else list(range(cols, 0, -1))
        h = 0.0 if north else 180.0
        ys = []
        for i, c in enumerate(cs):
            y = 2.0 + (i if north else cols - 1 - i) * sim.PITCH
            pts.append((f"R{r+1}_C{c}", x, y, h))
            plot_xy.append((x, y))
            ys.append(y)
        if r < rows - 1:
            yend = ys[-1] + 5.0 if north else ys[-1] - 5.0
            pts.append((f"mark_{len(pts)+1}", x, yend, h))
            pts.append((f"mark_{len(pts)+1}", x + sim.PITCH, yend,
                        (h + 180.0) % 360.0))
    return pts, plot_xy


def run_capture_mission(app_kwargs=None, mode="auto", timeout=30.0,
                        no_app=False, rows=2, cols=3):
    pts, plot_xy = grid_route(rows, cols)
    tmp = tempfile.mkdtemp()
    frame = nav.Frame(sim.LAT0, sim.LON0)
    teach = os.path.join(tmp, "teach.json")
    sim.teach_file(teach, frame, pts)
    sock = os.path.join(tmp, "cap.sock")
    app = None if no_app else FakeApp(sock, **(app_kwargs or {}))

    clock = PacedTime()
    rover = sim.SimRover(clock, frame, (0.0, 0.0), 0.0)
    rover.plots = plot_xy
    saved = nav.time, nav.Rover
    nav.time, nav.Rover = clock, (lambda: rover)
    out = io.StringIO()
    try:
        args = argparse.Namespace(path=teach, capture=True, capture_mode=mode,
                                  capture_socket=sock, settle=1.0,
                                  capture_timeout=timeout)
        with redirect_stdout(out):
            rc = nav.cmd_mission(args)
    finally:
        nav.time, nav.Rover = saved
        if app:
            app.close()
    runs = [f for f in os.listdir(tmp) if "_run_" in f]
    rows_out = []
    if runs:
        import csv
        with open(os.path.join(tmp, runs[0])) as fh:
            rows_out = list(csv.DictReader(fh))
    return rc, rows_out, app, rover, out.getvalue()


class Mission(unittest.TestCase):

    def test_every_plot_captured_once_headlands_never(self):
        rc, rows, app, rover, out = run_capture_mission()
        self.assertEqual(rc, 0, out[-2000:])
        labels = [c["label"] for c in app.captures]
        self.assertEqual(labels, ["R1_C1", "R1_C2", "R1_C3",
                                  "R2_C3", "R2_C2", "R2_C1"])
        self.assertEqual(app.hello["labels"], labels)
        for c in app.captures:
            nv = c["nav"]
            self.assertTrue(nv["gps_ok"])
            self.assertLess(nv["stop_err_cm"], 5.0)
            self.assertEqual(nv["mark_label"], c["label"])
            self.assertEqual(nv["trigger"], "auto")
            self.assertEqual((c["row"], c["col"]),
                             capture_link.parse_plot_label(c["label"]))
        plots = [r for r in rows if r["label"].startswith("R")]
        heads = [r for r in rows if r["label"].startswith("mark_")]
        self.assertTrue(all(r["capture"] == "ok" and r["result"] == "ok"
                            for r in plots), plots)
        self.assertTrue(all(r["capture"] == "" for r in heads))
        states = [s["state"] for s in app.statuses]
        self.assertIn("arrived", states)
        self.assertEqual(states[-1], "complete")

    def test_quarantine_is_flagged_and_run_continues(self):
        rc, rows, app, _, out = run_capture_mission(
            {"answer": lambda l: "quarantined" if l == "R1_C2" else "ok"})
        self.assertEqual(rc, 0, out[-2000:])
        by = {r["label"]: r for r in rows}
        self.assertEqual(by["R1_C2"]["result"], "flag_capture")
        self.assertEqual(by["R1_C3"]["result"], "ok")
        self.assertEqual(len(app.captures), 6)

    def test_skip_is_recorded(self):
        rc, rows, _, _, out = run_capture_mission(
            {"answer": lambda l: "skipped" if l == "R2_C2" else "ok"})
        self.assertEqual(rc, 0)
        by = {r["label"]: r for r in rows}
        self.assertEqual(by["R2_C2"]["result"], "skipped")

    def test_no_answer_times_out_and_continues(self):
        rc, rows, app, _, out = run_capture_mission(
            {"answer": lambda l: None if l == "R1_C1" else "ok"}, timeout=5.0)
        self.assertEqual(rc, 0, out[-2000:])
        by = {r["label"]: r for r in rows}
        self.assertEqual(by["R1_C1"]["capture"], "failed")
        self.assertIn("no answer", by["R1_C1"]["capture_detail"])
        self.assertEqual(by["R1_C2"]["capture"], "ok")

    def test_wait_mode_waits_past_the_auto_timeout(self):
        # In wait mode the robot waits for the operator however long it
        # takes; the answer here arrives long after the 0.5 s auto timeout.
        def slow(label):
            realtime.sleep(0.25)        # ~ many fake seconds at PacedTime
            return "ok"
        rc, rows, app, _, out = run_capture_mission(
            {"answer": slow}, mode="wait", timeout=0.5, rows=1, cols=2)
        self.assertEqual(rc, 0, out[-2000:])
        self.assertTrue(all(r["capture"] == "ok" for r in rows))
        self.assertTrue(all(c["mode"] == "wait" for c in app.captures))
        self.assertTrue(all(c["nav"]["trigger"] == "wait"
                            for c in app.captures))

    def test_refused_plot_map_never_moves(self):
        rc, rows, _, rover, out = run_capture_mission(
            {"refuse": ["R2_C3"]})
        self.assertEqual(rc, 1)
        self.assertIn("refused", out)
        self.assertEqual(rover.cmd[1:], (0, 0))
        self.assertEqual((rover.cx, rover.cy), (0.0, 0.0))

    def test_no_camera_app_never_moves(self):
        rc, rows, _, rover, out = run_capture_mission(no_app=True)
        self.assertEqual(rc, 1)
        self.assertIn("not reachable", out)
        self.assertEqual((rover.cx, rover.cy), (0.0, 0.0))

    def test_app_dies_mid_run_stops_machine(self):
        rc, rows, app, rover, out = run_capture_mission({"die_at": "R1_C3"})
        self.assertEqual(rc, 1)
        self.assertIn("camera app gone", out)
        self.assertEqual(rover.cmd, (nav.STATE_AUTO_READY, 0, 0))
        self.assertEqual([r["label"] for r in rows][:2], ["R1_C1", "R1_C2"])

    def test_resume_skips_completed(self):
        pts, _ = grid_route(1, 3)
        # covered by cmd_mission's completed-file logic; here just check
        # hello only lists what is left
        tmp = tempfile.mkdtemp()
        frame = nav.Frame(sim.LAT0, sim.LON0)
        teach = os.path.join(tmp, "teach.json")
        sim.teach_file(teach, frame, pts)
        with open(os.path.join(tmp, "teach_completed.txt"), "w") as fh:
            fh.write("R1_C1\n")
        sock = os.path.join(tmp, "cap.sock")
        app = FakeApp(sock)
        clock = PacedTime()
        rover = sim.SimRover(clock, frame, (0.0, 0.0), 0.0)
        saved = nav.time, nav.Rover
        nav.time, nav.Rover = clock, (lambda: rover)
        try:
            args = argparse.Namespace(path=teach, capture=True, resume=True,
                                      capture_socket=sock)
            with redirect_stdout(io.StringIO()):
                rc = nav.cmd_mission(args)
        finally:
            nav.time, nav.Rover = saved
            app.close()
        self.assertEqual(rc, 0)
        self.assertEqual(app.hello["labels"], ["R1_C2", "R1_C3"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
