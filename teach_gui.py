#!/usr/bin/env python3
"""
teach_gui.py - teach plots with the plot map on screen.

The same teach as `rover_log.py teach --cols N`, in a window. It writes the
same teach file (~/logs/rover_<time>.json) through the same code
(rover_log.TeachSession), so the mission and the camera app need nothing
new. What it adds is the picture:

    red     plot not taught yet
    yellow  averaging now (hold still)
    green   taught
    grey    skipped (s)
    blue    border = the plot the next 'n' will mark

and, at the end of each row on the side where the turn is, how many
headland points ('m') have been taught for that turn. A turn needs two:
one about 5 m past the row's last plot, and one lined up on the next row
(the lead-in). With fewer, the robot cuts the corner into the next row.

    python3 teach_gui.py                       pick the map in the window
    python3 teach_gui.py --map "2023 SA NTEP" --start-row 3
    python3 teach_gui.py --rows 7 --cols 9     no plot map file needed
    python3 teach_gui.py --demo                fake GPS - try it on a laptop

Keys (click the window first so it has the keyboard):
    n   mark the next plot              m   mark a headland point
    s   skip the next plot (missing)    u   undo the last mark or skip
    q   save and quit                   (the file is also saved after
                                         every mark, so a crash loses nothing)

The plot map comes from the camera app's plot_maps_config.py: --maps-file,
else $TURF_CAMERA_DIR, else a few usual places (see find_maps_file).
"""

import argparse
import glob
import importlib.util
import os
import random
import struct
import sys
import tempfile
import time
import tkinter as tk
from tkinter import messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rover_log                                            # noqa: E402
from rover_log import TeachSession, MARK_AVG_S              # noqa: E402

# -- colours -----------------------------------------------------------------
C_TODO = "#E05A4E"
C_DONE = "#2E9E4F"
C_AVG = "#F2C230"
C_SKIP = "#A8A8A8"
C_OUT = "#E9E9E9"
C_NEXT = "#1F5FD1"
C_OK_TURN = "#2E9E4F"
C_PART_TURN = "#E08A00"
C_BAD_TURN = "#C62828"
TURN_POINTS_WANTED = 2


# ============================================================================
#  Plot maps
# ============================================================================

def find_maps_file(explicit=None):
    cands = []
    if explicit:
        cands.append(explicit)
    env = os.environ.get("TURF_CAMERA_DIR")
    if env:
        cands.append(os.path.join(env, "plot_maps_config.py"))
    home = os.path.expanduser("~")
    for pat in ("camera_app*/plot_maps_config.py",
                "Turf_phenotype/*/plot_maps_config.py",
                "Turf_phenotype/plot_maps_config.py",
                "*/plot_maps_config.py",
                "Downloads/TAMU/Turf_phenotype/Amiga_automation/camera_app*/"
                "plot_maps_config.py"):
        cands += sorted(glob.glob(os.path.join(home, pat)), reverse=True)
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def load_maps(path):
    spec = importlib.util.spec_from_file_location("plot_maps_config", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return dict(mod.PLOT_MAPS)


# ============================================================================
#  Demo sources (no hardware)
# ============================================================================

class DemoFeed:
    """RTK-fixed readings that sit still with ~5 mm noise; the spot moves a
    little after each mark, as if you drove to the next plot. 'f' in the
    window drops the fix to float, to see a refusal."""

    name = "demo"

    def __init__(self):
        self.rtcm_bytes = 0
        self.lat, self.lon = 32.7767, -96.7970
        self.hdg_raw = 270.0
        self.fixed = True
        self.next_t = 0.0
        self.rng = random.Random(1)

    def move(self, metres_north=1.22):
        self.lat += metres_north / 111_320.0

    def poll(self, now):
        if now < self.next_t:
            return []
        self.next_t = now + 0.1
        self.rtcm_bytes += 120
        j = lambda: self.rng.gauss(0, 0.005) / 111_320.0
        gga = {"lat": self.lat + j(), "lon": self.lon + j(),
               "fix_q": "4" if self.fixed else "5", "nsat": "28",
               "hdop": "0.6", "alt_m": "150.0",
               "corr_age_s": "1.0" if self.fixed else "4.0"}
        tar = {"head_q": "4", "baseline_m": 1.33, "pitch_deg": 0.2,
               "heading_deg": self.hdg_raw + self.rng.gauss(0, 0.05),
               "head_nsat": "20"}
        return [("gga", gga, 0.0), ("tar", tar, 0.0)]

    def close(self):
        pass


class DemoCan:
    """A stationary Amiga: TPDO1 at 20 Hz, speed 0."""

    def __init__(self):
        self.next_t = 0.0

    def recv(self, n):
        now = time.monotonic()
        if now < self.next_t:
            raise BlockingIOError
        self.next_t = now + 0.05
        data = struct.pack("<Bhh", 4, 0, 0) + b"\x00\x00" + bytes([87])
        return rover_log.CAN_FRAME.pack(rover_log.COBID_TPDO1, 8, data)

    def close(self):
        pass


# ============================================================================
#  The window
# ============================================================================

class TeachWindow:

    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.s = None
        self.grid = None            # list of rows of genotype names
        self.map_name = None
        root.title("Teach plots")
        root.geometry("1400x900")
        root.configure(bg="white")
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.log_lines = []
        self.flash = None           # (row, col, until) - a refused plot
        self.maps_file = find_maps_file(args.maps_file)
        self.maps = load_maps(self.maps_file) if self.maps_file else {}
        if args.rows and args.cols:
            self.grid = [[""] * args.cols for _ in range(args.rows)]
            self.map_name = f"{args.rows} x {args.cols} grid"
            self.start(args.start_row)
        elif args.map:
            if args.map not in self.maps:
                messagebox.showerror("Unknown map",
                                     f"'{args.map}' is not in "
                                     f"{self.maps_file or 'any plot map file'}")
                self.picker()
            else:
                self.use_map(args.map)
                self.start(args.start_row)
        else:
            self.picker()

    # ---------------------------------------------------------------- picker
    def picker(self):
        f = self.pick_frame = tk.Frame(self.root, bg="white")
        f.pack(expand=True)
        tk.Label(f, text="Teach plots", font=("Arial", 26, "bold"),
                 bg="white").pack(pady=10)
        if not self.maps:
            tk.Label(f, bg="white", fg=C_BAD_TURN, font=("Arial", 12),
                     text="No plot_maps_config.py found. Start with "
                          "--maps-file PATH, or --rows R --cols C.").pack()
            return
        tk.Label(f, text=f"maps from {self.maps_file}", fg="gray",
                 bg="white").pack()
        row = tk.Frame(f, bg="white")
        row.pack(pady=12)
        tk.Label(row, text="Plot map:", font=("Arial", 14),
                 bg="white").pack(side=tk.LEFT)
        self.map_var = tk.StringVar(value=next(iter(self.maps)))
        ttk.Combobox(row, textvariable=self.map_var, values=list(self.maps),
                     width=40, state="readonly",
                     font=("Arial", 13)).pack(side=tk.LEFT, padx=6)
        row2 = tk.Frame(f, bg="white")
        row2.pack(pady=6)
        tk.Label(row2, text="This block starts on map row:",
                 font=("Arial", 14), bg="white").pack(side=tk.LEFT)
        self.start_var = tk.Spinbox(row2, from_=1, to=200, width=4,
                                    font=("Arial", 14))
        self.start_var.pack(side=tk.LEFT, padx=6)
        tk.Label(f, bg="white", fg="#404040", font=("Arial", 11),
                 justify=tk.LEFT,
                 text="Use the same map as the camera app. Row 1 runs C1 -> "
                      "last column, row 2 comes back, and so on.").pack(pady=6)
        tk.Button(f, text="START TEACHING", bg=C_DONE, fg="white",
                  font=("Arial", 16, "bold"), width=18,
                  command=self._picked).pack(pady=14)

    def _picked(self):
        try:
            start_row = int(self.start_var.get())
        except ValueError:
            start_row = 1
        name = self.map_var.get()
        rows = self.maps[name]
        if not 1 <= start_row <= len(rows):
            messagebox.showerror("Start row", f"'{name}' has rows 1-{len(rows)}")
            return
        self.pick_frame.destroy()
        self.use_map(name)
        self.start(start_row)

    def use_map(self, name):
        self.map_name = name
        self.grid = [list(r) for r in self.maps[name]]

    # ---------------------------------------------------------------- start
    def start(self, start_row):
        cols = max(len(r) for r in self.grid)
        ragged = any(len(r) != cols for r in self.grid)
        if self.args.demo:
            self.demo_feed = DemoFeed()
            feed, sock = self.demo_feed, DemoCan()
            out_dir = tempfile.mkdtemp(prefix="teach_demo_")
        else:
            feed, sock, why = rover_log.open_sources()
            if feed is None:
                messagebox.showerror("No GPS / CAN", why)
                self.root.destroy()
                return
            out_dir = None
        self.s = TeachSession(feed, sock, cols=cols, start_row=start_row,
                              teach=True, out_dir=out_dir)
        self.build()
        self.log(f"Teaching '{self.map_name}' from row {start_row}. "
                 f"File: {self.s.json_path}")
        if feed.name != "rtcm_forward" and not self.args.demo:
            self.log("GNSS straight from the ports - rtcm_forward is not "
                     "running.", "warn")
        if self.args.demo:
            self.log("DEMO: fake GPS. f = toggle RTK fixed/float. The file "
                     "goes to a temp folder, not ~/logs.", "warn")
        if ragged:
            self.log("This map has rows of different lengths; plot names "
                     f"use {cols} columns.", "warn")
        for k in ("n", "m", "s", "u", "q", "f"):
            self.root.bind(f"<KeyPress-{k}>", self.on_key)
            self.root.bind(f"<KeyPress-{k.upper()}>", self.on_key)
        self.root.focus_force()
        self.root.after(5, self.tick)
        self.root.after(200, self.refresh_status)

    # ---------------------------------------------------------------- layout
    def build(self):
        top = tk.Frame(self.root, bg="white")
        top.pack(fill=tk.X, padx=14, pady=(10, 4))
        self.lbl_next = tk.Label(top, text="", font=("Arial", 26, "bold"),
                                 bg="white", anchor="w")
        self.lbl_next.pack(side=tk.LEFT)
        self.lbl_fix = tk.Label(top, text="", font=("Arial", 16, "bold"),
                                fg="white", bg="gray", padx=12, pady=4)
        self.lbl_fix.pack(side=tk.RIGHT)
        self.lbl_gps = tk.Label(top, text="", font=("Arial", 12),
                                bg="white", fg="#303030")
        self.lbl_gps.pack(side=tk.RIGHT, padx=12)

        self.lbl_counts = tk.Label(self.root, text="", font=("Arial", 13),
                                   bg="white", anchor="w")
        self.lbl_counts.pack(fill=tk.X, padx=14)

        self.canvas = tk.Canvas(self.root, bg="white", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.canvas.bind("<Configure>", lambda e: self.draw())

        bar = tk.Frame(self.root, bg="white")
        bar.pack(fill=tk.X, padx=14, pady=4)
        spec = (("N  mark plot", "n", C_DONE), ("M  headland point", "m", "#6A4FB3"),
                ("S  skip plot", "s", "#707070"), ("U  undo", "u", "#B35C00"),
                ("Q  save & quit", "q", "#303030"))
        self.buttons = {}
        for text, key, colour in spec:
            b = tk.Button(bar, text=text, bg=colour, fg="white",
                          font=("Arial", 14, "bold"), width=16,
                          command=lambda k=key: self.do(k))
            b.pack(side=tk.LEFT, padx=4)
            self.buttons[key] = b
        legend = tk.Frame(bar, bg="white")
        legend.pack(side=tk.RIGHT)
        for colour, text in ((C_TODO, "to teach"), (C_AVG, "averaging"),
                             (C_DONE, "taught"), (C_SKIP, "skipped")):
            tk.Label(legend, text=f" {text} ", bg=colour, fg="white" if colour
                     != C_AVG else "black",
                     font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=2)

        self.txt = tk.Text(self.root, height=5, font=("DejaVu Sans Mono", 10),
                           bg="#F7F7F7", relief=tk.FLAT, state=tk.DISABLED)
        self.txt.pack(fill=tk.X, padx=14, pady=(2, 10))
        self.txt.tag_config("warn", foreground="#B35C00")
        self.txt.tag_config("bad", foreground=C_BAD_TURN)
        self.txt.tag_config("good", foreground=C_DONE)

    def log(self, text, tag=None):
        self.txt.config(state=tk.NORMAL)
        self.txt.insert(tk.END, time.strftime("%H:%M:%S  ") + text + "\n",
                        tag or ())
        self.txt.see(tk.END)
        self.txt.config(state=tk.DISABLED)

    # ---------------------------------------------------------------- state
    def plot_states(self):
        """{(row, col): 'done'|'skip'} from the session, plus the pending."""
        st = {}
        for m in self.s.marks:
            if m.get("kind") == "plot" and "row" in m:
                st[(m["row"], m["col"])] = "done"
        for name in self.s.skipped:
            rc = parse_rc(name)
            if rc:
                st[rc] = "skip"
        return st

    def turn_counts(self):
        """Headland points per turn: key 'start' (before the first plot) or
        the row the turn follows. A headland point belongs to the turn after
        the last plot marked before it."""
        counts = {}
        last_row = None
        for m in self.s.marks:
            if m.get("kind") == "plot":
                last_row = m.get("row")
            else:
                key = "start" if last_row is None else last_row
                counts[key] = counts.get(key, 0) + 1
        return counts

    # ---------------------------------------------------------------- drawing
    def draw(self):
        if self.s is None:
            return
        cv = self.canvas
        cv.delete("all")
        W, H = cv.winfo_width(), cv.winfo_height()
        if W < 50 or H < 50:
            return
        nrows = len(self.grid)
        ncols = max(len(r) for r in self.grid)
        side = 110                          # turn badges left and right
        lab_w = 60                          # "R3 ->" labels
        gw = W - 2 * side - lab_w
        cw = gw / ncols
        ch = min(H / (nrows + 0.6), cw * 0.9)
        y0 = (H - ch * nrows) / 2
        x0 = side + lab_w
        start = self.s.labeler.start_row
        states = self.plot_states()
        nxt = self.s.next_rc()
        pend = self.s.pending
        pend_rc = parse_rc(pend["label"]) if pend and pend["numbered"] else None
        fs = max(7, min(12, int(cw / 9)))
        flash = self.flash if self.flash and self.flash[2] > time.monotonic() else None

        for r in range(1, nrows + 1):
            y = y0 + (r - 1) * ch
            arrow = "->" if r % 2 == 1 else "<-"
            cv.create_text(side + lab_w - 8, y + ch / 2, anchor="e",
                           text=f"R{r} {arrow}", font=("Arial", 12, "bold"),
                           fill="#404040" if r >= start else "#B0B0B0")
            for c in range(1, len(self.grid[r - 1]) + 1):
                x = x0 + (c - 1) * cw
                stt = states.get((r, c))
                if (r, c) == pend_rc:
                    fill, fg = C_AVG, "black"
                elif stt == "done":
                    fill, fg = C_DONE, "white"
                elif stt == "skip":
                    fill, fg = C_SKIP, "white"
                elif r < start:
                    fill, fg = C_OUT, "#A0A0A0"
                else:
                    fill, fg = C_TODO, "white"
                cv.create_rectangle(x + 2, y + 2, x + cw - 2, y + ch - 2,
                                    fill=fill, outline="white", width=1)
                geno = self.grid[r - 1][c - 1]
                cv.create_text(x + cw / 2, y + ch / 2 - (fs * 0.7 if geno else 0),
                               text=f"R{r}_C{c}", fill=fg,
                               font=("Arial", fs, "bold"))
                if geno and ch > 34:
                    cv.create_text(x + cw / 2, y + ch / 2 + fs * 0.8,
                                   text=geno[:14], fill=fg, font=("Arial", fs - 1))
                if nxt and (r, c) == nxt and (r, c) != pend_rc:
                    cv.create_rectangle(x + 1, y + 1, x + cw - 1, y + ch - 1,
                                        outline=C_NEXT, width=5)
                if flash and (r, c) == flash[:2]:
                    cv.create_rectangle(x + 1, y + 1, x + cw - 1, y + ch - 1,
                                        outline=C_BAD_TURN, width=5)

        # turns: on the side each row ends, between it and the next row
        counts = self.turn_counts()
        rows_reached = {m["row"] for m in self.s.marks
                        if m.get("kind") == "plot" and "row" in m}
        keys = ["start"] + list(range(start, nrows + 1))
        for key in keys:
            n = counts.get(key, 0)
            if key == "start":
                if n == 0:
                    continue
                yc = y0 + (start - 1) * ch + ch / 2
                right = start % 2 == 0          # the side row `start` begins
                caption, colour, txt = "lead-in", C_OK_TURN, f"{n} m"
            else:
                r = key
                if n == 0 and r not in rows_reached:
                    continue
                last = r == nrows
                yc = y0 + r * ch if not last else y0 + (r - 0.5) * ch
                right = r % 2 == 1               # odd rows end on the right
                caption = f"turn R{r} > R{r+1}" if not last else "end"
                if last:
                    if n == 0:
                        continue
                    colour, txt = C_OK_TURN, f"{n} m"
                elif n >= TURN_POINTS_WANTED:
                    colour, txt = C_OK_TURN, f"{n} m  OK"
                elif n == 0 and (r + 1) in rows_reached:
                    colour, txt = C_BAD_TURN, "0 m  !"
                elif n == 0:
                    colour, txt = "#BDBDBD", f"0 of {TURN_POINTS_WANTED} m"
                elif (r + 1) in rows_reached:
                    colour, txt = C_BAD_TURN, f"{n} of {TURN_POINTS_WANTED} m !"
                else:
                    colour, txt = C_PART_TURN, f"{n} of {TURN_POINTS_WANTED} m"
            bx = (x0 + ncols * cw + 8) if right else 6
            cv.create_rectangle(bx, yc - 22, bx + side - 14, yc + 22,
                                fill=colour, outline="")
            cv.create_text(bx + (side - 14) / 2, yc - 9, text=caption,
                           fill="white", font=("Arial", 9))
            cv.create_text(bx + (side - 14) / 2, yc + 8, text=txt,
                           fill="white", font=("Arial", 12, "bold"))

    # ---------------------------------------------------------------- status
    def refresh_status(self):
        if self.s is None:
            return
        st = self.s.status(time.monotonic())
        pend = st["averaging"]
        if pend:
            self.lbl_next.config(text=f"HOLD STILL  {pend[0]}  "
                                      f"{max(0, pend[1]):.1f} s", fg="#9A7000")
        else:
            nxt = self.s.next_plot()
            rc = self.s.next_rc()
            geno = ""
            if rc and rc[0] <= len(self.grid) and rc[1] <= len(self.grid[rc[0] - 1]):
                geno = self.grid[rc[0] - 1][rc[1] - 1]
            beyond = rc and rc[0] > len(self.grid)
            self.lbl_next.config(
                text=("all plots in the map are done - m for headland, q to save"
                      if beyond else f"NEXT:  {nxt}" + (f"   {geno}" if geno else "")),
                fg=C_NEXT)
        fixed = st["fix_q"] == "4" and not st["stale"]
        self.lbl_fix.config(text=("RTK FIXED" if fixed else
                                  (st["fix"].upper() if not st["stale"] else
                                   "NO DATA: " + ",".join(st["stale"]))),
                            bg=C_DONE if fixed else C_BAD_TURN)
        hd = f"{st['heading']:.1f} deg" if st["heading"] is not None else "-"
        sp = st["speed"] if st["speed"] is not None else "-"
        moving = isinstance(st["speed"], int) and abs(st["speed"]) > 10
        self.lbl_gps.config(
            text=f"sats {st['nsat'] or '-'}   corr {st['corr_age_s'] or '-'} s   "
                 f"heading {hd}   speed {sp} mm/s" + ("  MOVING" if moving else ""),
            fg=C_BAD_TURN if moving else "#303030")
        states = self.plot_states()
        done = sum(1 for v in states.values() if v == "done")
        skipped = sum(1 for v in states.values() if v == "skip")
        start = self.s.labeler.start_row
        total = sum(len(r) for r in self.grid[start - 1:])
        heads = sum(1 for m in self.s.marks if m.get("kind") == "headland")
        self.lbl_counts.config(
            text=f"{self.map_name}  |  plots taught {done}   skipped {skipped}   "
                 f"left {total - done - skipped}   |  headland points {heads}"
                 f"   |  refused {self.s.rejected}   |  saved after every mark")
        for k in ("n", "m", "s", "u"):
            self.buttons[k].config(state=tk.DISABLED if pend else tk.NORMAL)
        if pend or (self.flash and self.flash[2] > time.monotonic() - 0.3):
            self.draw()
        self.root.after(200, self.refresh_status)

    # ---------------------------------------------------------------- loop
    def tick(self):
        if self.s is None or self.s.closed:
            return
        now = time.monotonic()
        try:
            events = self.s.poll(now)
        except OSError as e:
            self.log(f"read error: {e}", "bad")
            events = []
        for kind, text, data in events:
            if kind == "mark":
                self.log(text, "good")
                self.check_turn_after(data)
                if self.args.demo:
                    self.demo_feed.move()
            elif kind == "refused":
                self.log(text, "bad")
                rc = parse_rc(data["label"])
                if rc:
                    self.flash = (rc[0], rc[1], time.monotonic() + 3.0)
                self.root.bell()
            self.draw()
        self.root.after(10, self.tick)

    def check_turn_after(self, mark):
        """Warn when a new row is started without two turn points."""
        if mark.get("kind") != "plot":
            return
        plots = [m for m in self.s.marks if m.get("kind") == "plot"]
        if len(plots) < 2:
            return
        prev = plots[-2]
        if prev.get("row") is None or prev["row"] == mark.get("row"):
            return
        n = self.turn_counts().get(prev["row"], 0)
        if n < TURN_POINTS_WANTED:
            self.log(f"Row {mark['row']} started with only {n} headland "
                     f"point(s) after row {prev['row']} - the robot would cut "
                     f"the corner. Press u, then m for the missing turn "
                     f"point(s), then n again.", "bad")
            self.root.bell()

    # ---------------------------------------------------------------- keys
    def on_key(self, ev):
        self.do(ev.keysym.lower())

    def do(self, key):
        s = self.s
        if s is None:
            return
        now = time.monotonic()
        if key == "q":
            self.quit()
            return
        if key == "f" and self.args.demo:
            self.demo_feed.fixed = not self.demo_feed.fixed
            self.log(f"DEMO: fix now {'RTK fixed' if self.demo_feed.fixed else 'FLOAT'}",
                     "warn")
            return
        if s.pending is not None:
            return
        if key == "n":
            rc = s.next_rc()
            if rc and rc[0] > len(self.grid):
                self.log("Every plot in the map is already taught or skipped.",
                         "warn")
                return
            label = s.mark("plot", now)
            self.log(f"{label}: hold still, averaging {MARK_AVG_S:.0f} s ...")
        elif key == "m":
            label = s.mark("headland", now)
            self.log(f"{label} (headland): hold still, averaging "
                     f"{MARK_AVG_S:.0f} s ...")
        elif key == "s":
            name = s.skip()
            if name:
                self.log(f"skipped {name} (not marked, will not be driven)",
                         "warn")
        elif key == "u":
            r = s.undo(now)
            self.log("nothing to undo" if r is None
                     else f"UNDONE {r[0]} {r[1]}", "warn")
        self.draw()

    def quit(self):
        if self.s is not None and not self.s.closed:
            if self.s.pending is not None:
                if not messagebox.askyesno("Averaging",
                                           "A mark is still averaging and will "
                                           "be lost. Quit anyway?"):
                    return
            self.s.pending = None
            self.s.close()
            n = len(self.s.marks)
            messagebox.showinfo("Saved",
                                f"{n} marks saved to\n{self.s.json_path}\n\n"
                                f"Check it:  python3 amiga_nav.py plan "
                                f"{self.s.json_path}")
            print(f"{n} marks -> {self.s.json_path}")
        self.root.destroy()


def parse_rc(label):
    if not label or not label.startswith("R") or "_C" not in label:
        return None
    try:
        r, c = label[1:].split("_C")
        return int(r), int(c)
    except ValueError:
        return None


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map", help="plot map name, as in the camera app")
    p.add_argument("--maps-file", help="path to plot_maps_config.py")
    p.add_argument("--start-row", type=int, default=1,
                   help="map row this block starts on (default 1)")
    p.add_argument("--rows", type=int, help="no map file: rows")
    p.add_argument("--cols", type=int, help="no map file: columns")
    p.add_argument("--demo", action="store_true",
                   help="fake GPS and CAN; nothing is written to ~/logs")
    args = p.parse_args()
    if bool(args.rows) != bool(args.cols):
        p.error("--rows and --cols go together")
    root = tk.Tk()
    TeachWindow(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
