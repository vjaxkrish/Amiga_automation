#!/usr/bin/env python3
"""
commission.py - measure a machine and write its profile.

Run this whenever the GNSS kit goes onto a different Amiga (College Station
-> Dallas), or the mast is moved. It drives short test patterns on OPEN
GROUND (never on plots), measures, and writes the numbers the navigation
needs to ~/amiga_vehicle.json, which vehicle.py and amiga_nav.py then use.
No code is edited.

    python3 commission.py all              every step in order (~25 min)
    python3 commission.py <step>           one step; results accumulate
    python3 commission.py report           what has been measured so far
    python3 commission.py save             write the profile from it
    python3 commission.py --sim all        the same against a simulated
                                           machine - try it on the laptop

Steps, in the order `all` runs them:

    check      no motion. GPS, RTK, heading, CAN, Dashboard state, profile
    heading    drives 5 m straight forward and back. Receiver heading
               offset (180 on the College Station mast; ~+-90 if the mast
               is mounted left-right)
    centre     turns in place one way, then the other (~300 deg each). Where
               the antenna sits relative to the turning centre, forward AND
               sideways, and how far the machine walks while pivoting
    breakaway  the slowest forward speed and the slowest in-place turn rate
               that actually move this machine (it is heavier than the
               College Station one)
    yaw        rolling turns at several rates over ~7 m: how much of a
               commanded turn becomes rotation, and the smallest rolling turn
               that works
    swing      45 deg turns and short starts with different ramps: how hard
               it can be turned and started before the GPS mast swings
    arc        laps of rolling curves at several radii: the tightest
               headland curve the machine holds
    course     a real mini mission on fake plots from where it stands:
               2 rows x 4 plots, straight rows, curved headland turn. Tape it

Every motion step says how much clear ground it needs and asks before it
moves. Keep the pendant in hand: X is the real stop. q stops a step.

Results go to ~/logs/commission_<machine>.json, so steps can be run on
different days and re-run singly; later steps use what earlier ones
measured. Nothing changes the profile until `save` (or `all` at the end).
"""

import argparse
import json
import math
import os
import sys
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import amiga_nav as nav                    # noqa: E402
import vehicle                             # noqa: E402
from vehicle import circular_mean_deg      # noqa: E402

SWING_OK_DEG = 0.2      # heading swing below this is lost in the noise
SWING_OK_CM = 0.5       # antenna swing below this, likewise
LOG_DIR = os.path.expanduser(os.environ.get("TURF_NAV_LOGS", "~/logs"))
PITCH_M = 1.22


# ============================================================================
#  Plumbing
# ============================================================================

def now_utc():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class Ctx:
    def __init__(self, args):
        self.args = args
        self.rover = None
        self.frame = None
        self.results_path = args.results or os.path.join(
            LOG_DIR, f"commission_{args.machine}.json")
        self.data = {"machine": args.machine, "steps": {}, "recommend": {}}
        if os.path.isfile(self.results_path) and not args.fresh:
            with open(self.results_path) as fh:
                self.data = json.load(fh)
            self.data.setdefault("steps", {})
            self.data.setdefault("recommend", {})

    # -- results -----------------------------------------------------------
    def record(self, step, result, recommend):
        result = dict(result, time=now_utc(), sim=bool(self.args.sim))
        self.data["steps"][step] = result
        self.data["recommend"].update(recommend)
        apply_recommend(recommend)
        os.makedirs(os.path.dirname(self.results_path) or ".", exist_ok=True)
        with open(self.results_path, "w") as fh:
            json.dump(self.data, fh, indent=2)
        say(f"\n  saved to {self.results_path}")
        for k, v in recommend.items():
            say(f"    {k:<28} {BUILT_IN.get(k, current_value(k))!s:>10}  ->  {v}")

    def confirm(self, text):
        if self.args.yes:
            say(f"\n{text}  [yes - --yes given]")
            return True
        try:
            ans = input(f"\n{text}  Type y to go: ").strip().lower()
        except EOFError:
            ans = ""
        return ans in ("y", "yes")


def say(s=""):
    print(s, flush=True)


PROFILE_TOP = {"HEADING_OFFSET_DEG": "heading_offset_deg",
               "ANTENNA_OFFSET_M": "antenna_offset_m",
               "ANTENNA_LATERAL_M": "antenna_lateral_m"}


def current_value(k):
    if k in PROFILE_TOP:
        return getattr(vehicle, k)
    return getattr(nav, k, None)


MEASURED_KEYS = ["HEADING_OFFSET_DEG", "ANTENNA_OFFSET_M", "ANTENNA_LATERAL_M",
                 "CREEP_SPEED_MM_S", "ANGULAR_MIN_MRAD_S", "TURN_SLOW_MRAD_S",
                 "TURN_IN_PLACE_MRAD_S", "ARC_YAW_EFFICIENCY",
                 "ANGULAR_MIN_ROLLING_MRAD_S", "ANG_SLEW_MRAD_S2",
                 "LIN_ACCEL_MM_S2", "ARC_RADIUS_M", "ARC_SPEED_MM_S",
                 "HEADLAND_TURN"]
BUILT_IN = {}           # each value as the machine profile had it at start


def snapshot_built_in():
    for k in MEASURED_KEYS:
        BUILT_IN[k] = current_value(k)


def apply_recommend(rec):
    """Use measured values from here on (this run), before any save."""
    for k, v in rec.items():
        if k in PROFILE_TOP:
            setattr(vehicle, k, v)
        elif hasattr(nav, k):
            setattr(nav, k, v)


def open_machine(ctx, need_ready=True):
    args = ctx.args
    if ctx.rover is not None:
        return ctx.rover
    if args.sim:
        import sim_amiga
        clock = sim_amiga.FakeTime()
        nav.time = clock
        truth = json.loads(args.sim_truth) if args.sim_truth else None
        r = sim_amiga.SimAmiga(clock, truth, seed=args.seed)
    else:
        r = nav.Rover()
    r.open()
    ctx.rover = r
    if need_ready and not nav.wait_ready(r, timeout=args.ready_timeout):
        raise SystemExit("not ready - see the line above")
    t_end = nav.time.monotonic() + 10.0
    while r.lat is None and nav.time.monotonic() < t_end:
        r.poll(nav.time.monotonic())
        nav.time.sleep(0.05)
    if r.lat is None:
        raise SystemExit("no position from the GNSS - is rtcm_forward running?")
    ctx.frame = nav.Frame(r.lat, r.lon)
    return r


class Rec:
    """Each NEW GNSS reading: t, antenna x/y, true and raw heading, pitch,
    wheel speed."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.rows = []
        self.last = None

    def add(self, rover, tag=""):
        if rover.t_pos is None or rover.t_pos == self.last:
            return
        self.last = rover.t_pos
        x, y = self.ctx.frame.to_xy(rover.lat, rover.lon)
        self.rows.append({"t": rover.t_pos, "x": x, "y": y,
                          "h": rover.heading, "raw": rover.heading_raw,
                          "pitch": rover.pitch, "speed": rover.speed,
                          "tag": tag})

    def tagged(self, tag):
        return [r for r in self.rows if r["tag"] == tag]


def drive(ctx, step_fn, label, rec=None, tag=""):
    """Run a step function through the real drive_loop (GPS holds, hard
    faults, q to stop), recording every reading. Returns the stop reason."""
    rover = ctx.rover

    def wrapped(now, r):
        if rec is not None:
            rec.add(r, tag)
        return step_fn(now, r)
    why = nav.drive_loop(rover, wrapped, label=f"{label}  ")
    return why


def hold(ctx, secs, rec=None, tag=""):
    """Stand still, AUTO_ACTIVE and zero, recording."""
    rover = ctx.rover
    t_end = nav.time.monotonic() + secs
    while True:
        t = nav.time.monotonic()
        rover.poll(t)
        if rec is not None:
            rec.add(rover, tag)
        why = rover.hard_fault(t)
        if why:
            raise SystemExit(f"stopped: {why}")
        rover.send(nav.STATE_AUTO_ACTIVE, 0, 0)
        if t >= t_end:
            return
        nav.time.sleep(1.0 / nav.CONTROL_HZ)


def unwrap(hs):
    out, acc, prev = [], 0.0, None
    for h in hs:
        if prev is not None:
            acc += nav.wrap180(h - prev)
        out.append(acc)
        prev = h
    return out


def solve(A, b):
    """Least squares via normal equations, Gaussian elimination. Small."""
    n = len(A[0])
    M = [[sum(r[i] * r[j] for r in A) for j in range(n)] +
         [sum(r[i] * bb for r, bb in zip(A, b))] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            raise ValueError("singular")
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [a - f * bb for a, bb in zip(M[r], M[c])]
    return [M[i][n] / M[i][i] for i in range(n)]


def median(v):
    v = sorted(v)
    n = len(v)
    return None if not n else (v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2)


def round_to(v, step):
    return round(round(v / step) * step, 6)


def ceil_to(v, step):
    return math.ceil(v / step - 1e-9) * step


# ============================================================================
#  Steps
# ============================================================================

def step_check(ctx):
    say("== check: no motion")
    r = open_machine(ctx, need_ready=False)
    t_end = nav.time.monotonic() + 3.0
    while nav.time.monotonic() < t_end:
        r.poll(nav.time.monotonic())
        nav.time.sleep(0.05)
    t = nav.time.monotonic()
    say(f"  {vehicle.describe()}")
    say(f"  GNSS from {'rtcm_forward' if r.from_state else 'the ports'}"
        if not ctx.args.sim else "  GNSS from the simulator")
    say(f"  fix {r.fix_q} ({'RTK fixed' if r.fix_q == '4' else 'NOT fixed'}), "
        f"corrections {r.corr_age} s old")
    say(f"  heading raw {r.heading_raw}  -> true {r.heading}  "
        f"(offset {vehicle.HEADING_OFFSET_DEG})")
    st = nav.read_state() if not ctx.args.sim else None
    if st:
        say(f"  dual-antenna baseline {st.get('baseline_m')} m (tape it: "
            f"the receiver's number should match within ~2 cm)")
    say(f"  CAN: Dashboard state {r.state} "
        f"({'auto' if r.state in (nav.STATE_AUTO_READY, nav.STATE_AUTO_ACTIVE) else 'NOT in auto - arm it on the Dashboard tab'})")
    hf, gf = r.hard_fault(t), r.gps_fault(t)
    say(f"  ready to drive: {'yes' if not (hf or gf) else (hf or gf)}")
    ctx.record("check", {"fix_q": r.fix_q, "state": r.state,
                         "ok": not (hf or gf)}, {})
    return 0


def step_heading(ctx):
    a = ctx.args
    say(f"== heading: {a.dist:.0f} m straight forward at 300 mm/s, then back")
    if not ctx.confirm(f"Needs {a.dist + 2:.0f} m of clear, flat ground "
                       f"straight ahead, off the plots."):
        return 1
    r = open_machine(ctx)
    rec = Rec(ctx)
    x0, y0 = ctx.frame.to_xy(r.lat, r.lon)

    def leg(sign, tag):
        sx, sy = ctx.frame.to_xy(r.lat, r.lon)

        def fn(now, rv):
            x, y = ctx.frame.to_xy(rv.lat, rv.lon)
            d = math.hypot(x - sx, y - sy)
            if d >= a.dist:
                return 0, 0, True, f"{d:.2f} m"
            return sign * (300 if sign > 0 else 250), 0, False, \
                f"{'forward' if sign > 0 else 'back'} {d:4.2f} m"
        return drive(ctx, fn, "heading", rec, tag)

    for sign, tag in ((1, "fwd"), (-1, "back")):
        why = leg(sign, tag)
        if why != "complete":
            say(f"stopped: {why}")
            return 1
        hold(ctx, 1.0)

    ests = []
    for tag, sign in (("fwd", 1), ("back", -1)):
        rows = rec.tagged(tag)
        # the steady part: skip the first metre and the last 0.3 m
        sx, sy = rows[0]["x"], rows[0]["y"]
        steady = [q for q in rows
                  if 1.0 <= math.hypot(q["x"] - sx, q["y"] - sy) <= a.dist - 0.3]
        if len(steady) < 10:
            say(f"  too few readings on the {tag} leg ({len(steady)})")
            return 1
        n = max(3, len(steady) // 5)
        ax = sum(q["x"] for q in steady[:n]) / n
        ay = sum(q["y"] for q in steady[:n]) / n
        bx = sum(q["x"] for q in steady[-n:]) / n
        by = sum(q["y"] for q in steady[-n:]) / n
        course = nav.bearing_to(bx - ax, by - ay)
        if sign < 0:
            course = (course + 180.0) % 360.0        # it was reversing
        raw, raw_sd = circular_mean_deg([q["raw"] for q in steady])
        ests.append(((course - raw) % 360.0, raw_sd, tag))
    off, _ = circular_mean_deg([e[0] for e in ests])
    spread = abs(nav.wrap180(ests[0][0] - ests[1][0]))
    say(f"\n  forward leg: offset {ests[0][0]:.1f} deg (heading sd "
        f"{ests[0][1]:.2f}),  back leg: {ests[1][0]:.1f} deg")
    say(f"  HEADING_OFFSET_DEG = {off:.1f}   (the two legs differ by "
        f"{spread:.1f} deg{'' if spread < 2 else ' - drive again, flatter ground'})")
    near90 = round(off / 90.0) * 90 % 360
    if near90 in (90, 270):
        say("  The mast is mounted LEFT-RIGHT: the antenna is probably off the "
            "centreline. The centre step measures how far.")
    ctx.record("heading", {"estimates": ests, "spread_deg": spread},
               {"HEADING_OFFSET_DEG": round(off % 360.0, 1)})
    return 0


def _spin(ctx, sign, degrees, rate, rec, tag):
    turned = {"a": 0.0, "prev": None, "stop_at": None, "t0": None}

    def fn(now, rv):
        if turned["t0"] is None:
            turned["t0"] = now
        if turned["prev"] is not None:
            turned["a"] += abs(nav.wrap180(rv.heading - turned["prev"]))
        turned["prev"] = rv.heading
        if turned["stop_at"] is not None:
            if now >= turned["stop_at"]:
                return 0, 0, True, f"turned {turned['a']:.0f} deg"
            return 0, 0, False, "stopping"
        if turned["a"] >= degrees:
            turned["stop_at"] = now + 0.8
            return 0, 0, False, "stopping"
        if now - turned["t0"] > 5.0 and turned["a"] < 3.0:
            return 0, 0, True, "STOPPED: not turning - raise --rate"
        return 0, sign * rate, False, f"spin {turned['a']:5.0f} deg"
    return drive(ctx, fn, "spin", rec, tag)


def fit_centre(rows):
    """antenna = C + f * forward(h) + l * left(h), least squares."""
    A, b = [], []
    for q in rows:
        th = math.radians(q["h"])
        A.append([1, 0, math.sin(th), -math.cos(th)])
        b.append(q["x"])
        A.append([0, 1, math.cos(th), math.sin(th)])
        b.append(q["y"])
    cx, cy, f, l_ = solve(A, b)
    res = 0.0
    for q in rows:
        th = math.radians(q["h"])
        px = cx + f * math.sin(th) - l_ * math.cos(th)
        py = cy + f * math.cos(th) + l_ * math.sin(th)
        res += (px - q["x"]) ** 2 + (py - q["y"]) ** 2
    return f, l_, math.sqrt(res / max(1, len(rows)))


def step_centre(ctx):
    a = ctx.args
    say(f"== centre: turn in place {a.deg:.0f} deg left, then {a.deg:.0f} deg "
        f"right")
    if "heading" not in ctx.data["steps"]:
        say("  NOTE: heading offset not measured yet - run `heading` first, "
            "or this uses the profile's value.")
    if not ctx.confirm("Needs a clear 5 m circle on open ground, OFF the "
                       "plots: pivoting scuffs turf. Motors warm up - "
                       "do not repeat it many times in a row."):
        return 1
    rate = a.rate or max(nav.TURN_IN_PLACE_MRAD_S, 300)
    open_machine(ctx)
    rec = Rec(ctx)
    fits = {}
    for sign, tag in ((1, "left"), (-1, "right")):
        why = _spin(ctx, sign, a.deg, rate, rec, tag)
        if why != "complete":
            say(f"stopped: {why}")
            return 1
        hold(ctx, 1.0)
        rows = rec.tagged(tag)
        span = abs(unwrap([q["h"] for q in rows])[-1]) if rows else 0
        if len(rows) < 20 or span < 120:
            say(f"  {tag}: only {span:.0f} deg of readings - too little to fit")
            return 1
        fits[tag] = fit_centre(rows)
        say(f"  {tag:<5} forward {fits[tag][0]:.3f} m, sideways "
            f"{fits[tag][1]:+.3f} m (apparent), fit rms "
            f"{fits[tag][2]*100:.1f} cm")
    fL, lL, _ = fits["left"]
    fR, lR, _ = fits["right"]
    fwd = (fL + fR) / 2.0
    lat = (lL + lR) / 2.0
    walk = (lL - lR) / 2.0
    say(f"\n  ANTENNA_OFFSET_M  = {fwd:.3f}   (left/right agree to "
        f"{abs(fL - fR)*100:.1f} cm)")
    say(f"  ANTENNA_LATERAL_M = {lat:+.3f}   (+ = left of the centreline)")
    say(f"  walks {abs(walk)*100:.0f} cm {'backward' if walk > 0 else 'forward'} "
        f"per radian while pivoting (College Station turf: 28 cm)")
    if abs(fL - fR) > 0.05:
        say("  The two directions disagree by more than 5 cm - repeat on "
            "firmer, flatter ground before saving.")
    ctx.record("centre", {"fits": fits, "walk_m_per_rad": walk,
                          "rate": rate},
               {"ANTENNA_OFFSET_M": round(fwd, 3),
                "ANTENNA_LATERAL_M": round(lat, 3)})
    return 0


def _const_cmd(ctx, lin, ang, secs, rec, tag):
    t = {"t0": None}

    def fn(now, rv):
        if t["t0"] is None:
            t["t0"] = now
        if now - t["t0"] >= secs:
            return 0, 0, True, "done"
        return lin, ang, False, f"{lin} mm/s {ang} mrad/s"
    return drive(ctx, fn, "step", rec, tag)


def step_breakaway(ctx):
    say("== breakaway: slowest speed and slowest in-place turn that move it")
    if not ctx.confirm("Needs 2 m clear ahead and room to turn in place, off "
                       "the plots."):
        return 1
    r = open_machine(ctx)
    r.ramp = False
    r.angular_floor = False
    rec = Rec(ctx)
    lin_floor = None
    say("\n  forward speed:")
    for v in ctx.args.speeds:
        tag = f"v{v}"
        why = _const_cmd(ctx, v, 0, 2.5, rec, tag)
        if why != "complete":
            say(f"stopped: {why}")
            return 1
        rows = [q for q in rec.tagged(tag) if q["t"] >= rec.tagged(tag)[0]["t"] + 1.0]
        moved = math.hypot(rows[-1]["x"] - rows[0]["x"],
                           rows[-1]["y"] - rows[0]["y"]) if len(rows) > 2 else 0
        dt = rows[-1]["t"] - rows[0]["t"] if len(rows) > 2 else 1
        got = moved / dt * 1000.0 if dt > 0 else 0
        ok = got >= 0.5 * v
        say(f"    {v:4d} mm/s asked: {got:5.0f} mm/s by GPS  "
            f"{'MOVES' if ok else 'stuck'}")
        hold(ctx, 0.8)
        if ok and lin_floor is None:
            lin_floor = v
            break
    say("\n  turning in place:")
    rot_floor = None
    for k, w in enumerate(ctx.args.rates):
        sign = 1 if k % 2 == 0 else -1
        tag = f"w{w}"
        why = _const_cmd(ctx, 0, sign * w, 1.8, rec, tag)
        if why != "complete":
            say(f"stopped: {why}")
            return 1
        rows = rec.tagged(tag)
        rows = [q for q in rows if q["t"] >= rows[0]["t"] + 0.8]
        if len(rows) > 2:
            hs = unwrap([q["h"] for q in rows])
            got = abs(math.radians(hs[-1] - hs[0]) /
                      (rows[-1]["t"] - rows[0]["t"])) * 1000
        else:
            got = 0
        ok = got >= max(25.0, 0.15 * w)
        say(f"    {w:4d} mrad/s asked: {got:5.0f} mrad/s achieved  "
            f"{'TURNS' if ok else 'stuck'}")
        hold(ctx, 0.8)
        if ok and rot_floor is None:
            rot_floor = w
            break
    r.ramp = True
    r.angular_floor = True
    rec_vals = {}
    if lin_floor is None:
        say("\n  never moved forward - raise --speeds")
    else:
        creep = int(max(20, ceil_to(lin_floor * 1.25, 5)))
        rec_vals["CREEP_SPEED_MM_S"] = creep
        say(f"\n  moves from {lin_floor} mm/s -> CREEP_SPEED_MM_S = {creep} "
            f"(College Station: 40)")
    if rot_floor is None:
        say("  never turned in place - raise --rates")
    else:
        amin = int(ceil_to(rot_floor * 1.1, 10))
        rec_vals.update({"ANGULAR_MIN_MRAD_S": amin,
                         "TURN_SLOW_MRAD_S": int(ceil_to(amin * 1.2, 10)),
                         "TURN_IN_PLACE_MRAD_S": int(max(300, ceil_to(amin * 1.6, 10)))})
        say(f"  turns in place from {rot_floor} mrad/s -> ANGULAR_MIN_MRAD_S "
            f"= {amin} (College Station: 220)")
    ctx.record("breakaway", {"lin_floor_mm_s": lin_floor,
                             "rot_floor_mrad_s": rot_floor}, rec_vals)
    return 0


def step_yaw(ctx):
    a = ctx.args
    rates = a.yaw_rates
    per = nav.YawTest.SETTLE_S + a.secs + nav.YawTest.TAIL_S
    dist = 0.3 * per * len(rates)
    say(f"== yaw: rolling turns {rates} mrad/s at 300 mm/s, alternating sides")
    if not ctx.confirm(f"Needs about {dist + 2:.0f} m of clear ground ahead, "
                       f"2 m wide, off the plots."):
        return 1
    r = open_machine(ctx)
    r.ramp = False
    r.angular_floor = False
    yt = nav.YawTest(300, rates, a.secs)
    why = nav.drive_loop(r, yt.step, label="yaw  ", on_resume=yt.on_resume)
    r.ramp = True
    r.angular_floor = True
    if why != "complete":
        say(f"stopped: {why}")
        return 1
    ratios = [(abs(c), (g / c) if (g is not None and c) else 0.0)
              for c, g in yt.results]
    say("\n  commanded  achieved ratio")
    for (c, g), (_, rr) in zip(yt.results, ratios):
        say(f"    {c:+6d}  {('-' if g is None else f'{g:+6.0f}'):>8}  {rr:5.2f}")
    strong = [rr for c, rr in ratios if c >= sorted(rates)[len(rates) // 2]]
    eff = median(strong) or 0.0
    eff = max(0.1, min(1.0, eff))
    floor = next((c for c, rr in sorted(ratios) if rr >= 0.6 * eff), None)
    rec = {"ARC_YAW_EFFICIENCY": round(eff, 2)}
    say(f"\n  rolling turns give {eff*100:.0f}% of what is asked "
        f"(College Station turf: ~18%) -> ARC_YAW_EFFICIENCY = {eff:.2f}")
    if floor is not None:
        rec["ANGULAR_MIN_ROLLING_MRAD_S"] = int(floor)
        say(f"  smallest rolling turn that works: {floor} mrad/s -> "
            f"ANGULAR_MIN_ROLLING_MRAD_S")
    ctx.record("yaw", {"results": yt.results, "efficiency": eff}, rec)
    return 0


def _turn_by(ctx, sign, degrees, rate, rec, tag, slew):
    """Turn in place, then ramp the turn rate down at `slew` and stand still
    for settle_secs, all inside one drive_loop - ending the loop sends an
    instant stop, which would whip the mast whatever the ramp. Returns
    (stop reason, time the turn command went to zero)."""
    st = {"a": 0.0, "prev": None, "zero_at": None}
    ramp_s = (rate / slew) if slew else 0.0
    hold_s = ramp_s + ctx.args.settle_secs

    def fn(now, rv):
        if st["prev"] is not None:
            st["a"] += abs(nav.wrap180(rv.heading - st["prev"]))
        st["prev"] = rv.heading
        if st["zero_at"] is not None:
            if now >= st["zero_at"] + hold_s:
                return 0, 0, True, "settled"
            return 0, 0, False, "standing"
        if st["a"] >= degrees:
            st["zero_at"] = now
            return 0, 0, False, "standing"
        return 0, sign * rate, False, f"turn {st['a']:4.1f} deg"
    why = drive(ctx, fn, "swing", rec, tag)
    return why, st["zero_at"], ramp_s


def swing_after(rows, t_from, secs=2.0):
    """Swing left once the machine has stopped turning: RMS of heading and
    antenna position about where they end up, over `secs`, minus the GNSS
    noise measured at the end (when the swing is over)."""
    win = [q for q in rows if t_from <= q["t"] < t_from + secs]
    tail = [q for q in rows if q["t"] >= rows[-1]["t"] - 1.0]
    if len(win) < 8 or len(tail) < 6:
        return None
    hf = circular_mean_deg([q["h"] for q in tail])[0]
    xf = sum(q["x"] for q in tail) / len(tail)
    yf = sum(q["y"] for q in tail) / len(tail)

    def rms(qs):
        h = (sum(nav.wrap180(q["h"] - hf) ** 2 for q in qs) / len(qs)) ** 0.5
        p = (sum((q["x"] - xf) ** 2 + (q["y"] - yf) ** 2 for q in qs)
             / len(qs)) ** 0.5
        return h, p
    hw, pw = rms(win)
    hn, pn = rms(tail)
    return (max(0.0, hw * hw - hn * hn) ** 0.5,
            max(0.0, pw * pw - pn * pn) ** 0.5 * 100)


def step_swing(ctx):
    saved = (nav.ANG_SLEW_MRAD_S2, nav.LIN_ACCEL_MM_S2)
    rc = 1
    try:
        rc = _step_swing(ctx)
        return rc
    finally:
        if rc != 0:        # trial values must not outlive an unfinished step
            nav.ANG_SLEW_MRAD_S2, nav.LIN_ACCEL_MM_S2 = saved


def _step_swing(ctx):
    a = ctx.args
    say("== swing: 45 deg turns and short starts at different ramps; how "
        "long the GPS takes to stop swinging after each")
    if not ctx.confirm("Needs room to turn in place and 2 m ahead, off the "
                       "plots. Watch the mast: you will see which is gentle."):
        return 1
    open_machine(ctx)
    rate = nav.TURN_IN_PLACE_MRAD_S
    rec = Rec(ctx)
    turn_res = []
    say("\n  turns (ANG_SLEW_MRAD_S2, None = instant), each one left and "
        "one right; swing = what is left after the turn, above GPS noise:")
    k = 0
    for slew in a.slews:
        nav.ANG_SLEW_MRAD_S2 = slew
        hs, ps, durs = [], [], []
        for sign in (1, -1):
            tag = f"turn{k}"
            k += 1
            why, t_zero, ramp_s = _turn_by(ctx, sign, 45.0, rate, rec, tag,
                                           slew)
            if why != "complete":
                say(f"stopped: {why}")
                return 1
            rows = rec.tagged(tag)
            sw = swing_after(rows, t_zero + ramp_s)
            if sw is None:
                say("  too few readings - is the GNSS at 10 Hz?")
                return 1
            hs.append(sw[0])
            ps.append(sw[1])
            durs.append(t_zero + ramp_s - rows[0]["t"])
        h, pcm, dur = sum(hs) / 2, sum(ps) / 2, sum(durs) / 2
        turn_res.append((slew, round(h, 3), round(pcm, 2), round(dur, 1)))
        say(f"    {str(slew):>6}: turn {dur:4.1f} s, swing heading "
            f"{h:.2f} deg, antenna {pcm:.1f} cm")
    # Quickest ramp whose leftover swing is below what GNSS noise hides
    # anyway (or close to the best seen, if none gets that low).
    hmin = min(t[1] for t in turn_res)
    pmin = min(t[2] for t in turn_res)
    ok = [t[0] for t in turn_res
          if t[1] <= max(SWING_OK_DEG, hmin * 1.3)
          and t[2] <= max(SWING_OK_CM, pmin * 1.3)]
    chosen = max(ok, key=lambda s: float("inf") if s is None else s)
    say(f"  -> ANG_SLEW_MRAD_S2 = {chosen} (the quickest ramp whose swing is "
        f"under {SWING_OK_DEG} deg / {SWING_OK_CM} cm, or near the best)")

    say("\n  starts (LIN_ACCEL_MM_S2):")
    start_res = []
    for k, acc in enumerate(a.accels):
        nav.LIN_ACCEL_MM_S2 = acc
        nav.ANG_SLEW_MRAD_S2 = chosen
        tag = f"go{k}"
        sign = 1 if k % 2 == 0 else -1
        why = _const_cmd(ctx, sign * 400, 0, 2.2, rec, tag)
        if why != "complete":
            say(f"stopped: {why}")
            return 1
        hold(ctx, 1.5, rec, tag + "_rest")
        rows = rec.tagged(tag)
        # the first 2.5 s of the start, minus the steady lean once rolling
        ps = [q["pitch"] for q in rows[:25] if q["pitch"] is not None]
        if len(ps) > 4:
            ps = [sum(ps[max(0, i - 1):i + 2]) / len(ps[max(0, i - 1):i + 2])
                  for i in range(len(ps))]
        pp = (max(ps) - min(ps)) if ps else 0.0
        start_res.append((acc, round(pp, 3)))
        say(f"    {str(acc):>6}: mast pitch swung {pp:.2f} deg peak-to-peak "
            f"while starting")
    bestp = min(p for _, p in start_res)
    okp = [acc for acc, p in start_res if p <= bestp + 0.15]
    chosen_lin = max(okp, key=lambda s: float("inf") if s is None else s)
    say(f"  -> LIN_ACCEL_MM_S2 = {chosen_lin}")
    ctx.record("swing", {"turns": turn_res, "starts": start_res},
               {"ANG_SLEW_MRAD_S2": chosen, "LIN_ACCEL_MM_S2": chosen_lin})
    return 0


def lap_path(frame, x, y, h, R, side, straight=1.0):
    """Stadium lap from the pose and back: straight, half circle, straight
    back alongside, half circle home. side +1 = curving left."""
    fx, fy = math.sin(math.radians(h)), math.cos(math.radians(h))
    lx, ly = -fy, fx                                   # left unit
    off = 2.0 * R * side
    p1 = (x + straight * fx, y + straight * fy, h)
    p2 = (p1[0] + off * lx, p1[1] + off * ly, (h + 180) % 360)
    p3 = (x + off * lx, y + off * ly, (h + 180) % 360)
    pts = nav.line_path(x, y, p1[0], p1[1])
    for a_, b_ in ((p1, p2),):
        pts += nav.dubins_path(a_, b_, R)[2][1:]
    pts += nav.line_path(p2[0], p2[1], p3[0], p3[1])[1:]
    pts += nav.dubins_path(p3, (x, y, h), R)[2][1:]
    return pts


def step_arc(ctx):
    a = ctx.args
    eff = nav.ARC_YAW_EFFICIENCY
    say(f"== arc: laps of rolling curves at radii {a.radii} m "
        f"(turn efficiency {eff:.2f}"
        f"{'' if 'yaw' in ctx.data['steps'] else ' - NOT measured, run yaw first'})")
    big = max(a.radii)
    if not ctx.confirm(f"Needs open ground about {2*big + 2:.0f} m wide "
                       f"and {2*big + 3:.0f} m long - the laps curve to the "
                       f"LEFT. No plots, no people."):
        return 1
    r = open_machine(ctx)
    res = []
    for R in sorted(a.radii, reverse=True):
        # as fast as 300 mm/s, but slow enough that the turn this radius
        # needs stays well inside the 600 mrad/s command limit
        speed = int(min(300, max(100, round_to(
            0.7 * nav.MAX_ANGULAR_MRAD_S * eff * R, 10))))
        x, y = nav.control_point(ctx.frame, r.lat, r.lon, r.heading)
        pts = lap_path(ctx.frame, x, y, r.heading, R, +1)
        saved = (nav.ARC_RADIUS_M, nav.ARC_SPEED_MM_S)
        nav.ARC_RADIUS_M, nav.ARC_SPEED_MM_S = R, speed
        pf = nav.PathFollower(ctx.frame, pts, r.heading, label=f"R{R:g}")
        t0 = nav.time.monotonic()
        why = nav.drive_loop(r, pf.step, label=f"arc R{R:g}  ",
                             on_resume=pf.on_resume)
        nav.ARC_RADIUS_M, nav.ARC_SPEED_MM_S = saved
        secs = nav.time.monotonic() - t0
        ok = why == "complete" and pf.max_dev <= a.max_dev
        res.append({"R": R, "speed": speed, "done": why == "complete",
                    "max_dev_m": round(pf.max_dev, 3), "secs": round(secs, 1)})
        say(f"\n    R {R:4.1f} m at {speed} mm/s: "
            f"{'held' if ok else 'NOT held'} - max {pf.max_dev*100:.0f} cm off "
            f"the curve, {secs:.0f} s")
        if why != "complete":
            say(f"    ({why}) - stopping the arc step here")
            break
        hold(ctx, 1.0)
    held = [q for q in res if q["done"] and q["max_dev_m"] <= a.max_dev]
    rec = {}
    if held:
        best = min(held, key=lambda q: q["R"])
        rec = {"ARC_RADIUS_M": best["R"], "ARC_SPEED_MM_S": best["speed"],
               "HEADLAND_TURN": "arc"}
        say(f"\n  tightest curve held within {a.max_dev*100:.0f} cm: "
            f"R {best['R']} m at {best['speed']} mm/s -> ARC_RADIUS_M")
        fwd = nav.turn_extent(nav.dubins_path((0, 0, 0), (PITCH_M, 0, 180),
                                              best["R"])[2], 0)
        say(f"  a turn into the next row then needs {fwd[0]:.1f} m of "
            f"headland past the run-out point, {fwd[2]:.1f} m to the side")
    else:
        rec = {"HEADLAND_TURN": "pivot"}
        say("\n  no radius was held - headland turns stay as pivots "
            "(HEADLAND_TURN = pivot). Try larger --radii, or lower "
            "ARC_SPEED_MM_S.")
    ctx.record("arc", {"laps": res}, rec)
    return 0


def step_course(ctx):
    a = ctx.args
    side = 1 if a.side == "left" else -1
    say(f"== course: {a.rows} rows x {a.plots} fake plots, {PITCH_M} m apart, "
        f"rows to the {a.side}; straight rows, curved headland turn")
    depth = 2.0 + PITCH_M * (a.plots - 1) + a.runout
    R = nav.ARC_RADIUS_M
    if not ctx.confirm(f"Needs open ground about {depth + 2*R + 2:.0f} m "
                       f"ahead and {a.rows * PITCH_M + 2*R:.0f} m to the "
                       f"{a.side}. Chalk the fake plot spots if you want to "
                       f"tape them."):
        return 1
    r = open_machine(ctx)
    x0, y0 = nav.control_point(ctx.frame, r.lat, r.lon, r.heading)
    h0 = r.heading
    fx, fy = math.sin(math.radians(h0)), math.cos(math.radians(h0))
    lx, ly = -fy, fx
    marks = []

    def add(label, along, lateral, hdg, kind):
        x = x0 + along * fx + lateral * lx
        y = y0 + along * fy + lateral * ly
        lat, lon = ctx.frame.to_ll(x, y)
        marks.append({"label": label, "lat": lat, "lon": lon,
                      "heading_deg": hdg % 360.0, "fix_q": "4", "kind": kind})

    n_h = 0
    for row in range(a.rows):
        lat_off = side * row * PITCH_M
        away = row % 2 == 0
        hdg = h0 if away else h0 + 180.0
        alongs = [2.0 + k * PITCH_M for k in range(a.plots)]
        if not away:
            alongs = alongs[::-1]
        for k, al in enumerate(alongs):
            col = k + 1 if away else a.plots - k
            add(f"R{row+1}_C{col}", al, lat_off, hdg, "plot")
        if row < a.rows - 1:
            end = (alongs[-1] + a.runout) if away else (alongs[-1] - a.runout)
            n_h += 1
            add(f"mark_{len(marks)+1}", end, lat_off, hdg, "headland")
            add(f"mark_{len(marks)+1}", end, lat_off + side * PITCH_M,
                hdg + 180.0, "headland")
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, f"commission_course_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    with open(path, "w") as fh:
        json.dump({"format": 2, "heading_ref": "true", "marks": marks,
                   "note": "synthetic course from commission.py"}, fh, indent=2)
    say(f"  course written to {path}")
    say("  it is an ordinary teach file: `amiga_nav.py plan` / `mission` "
        "work on it too")
    # the mission opens its own Rover - give it ours
    saved_rover = nav.Rover
    nav.Rover = lambda: _Borrowed(r)
    try:
        rc = nav.cmd_mission(argparse.Namespace(
            path=path, dwell=a.dwell, resume=False, heading=None,
            capture=False))
    finally:
        nav.Rover = saved_rover
    runs = sorted(f for f in os.listdir(LOG_DIR)
                  if f.startswith(os.path.basename(path)[:-5] + "_run_"))
    summary = []
    if runs:
        import csv
        with open(os.path.join(LOG_DIR, runs[-1])) as fh:
            for row in csv.DictReader(fh):
                summary.append({k: row[k] for k in
                                ("label", "result", "along_cm", "cross_cm",
                                 "heading_err_deg")})
    say("\n  plot     along   cross   heading   (GPS's own estimate - tape "
        "them, that is the real number)")
    for s_ in summary:
        if s_["label"].startswith("R"):
            say(f"  {s_['label']:<8} {s_['along_cm']:>6}  {s_['cross_cm']:>6}  "
                f"{s_['heading_err_deg']:>7}")
    ctx.record("course", {"rc": rc, "teach_file": path, "plots": summary}, {})
    return rc


class _Borrowed:
    """The commissioning Rover, lent to cmd_mission without reopening."""

    def __init__(self, r):
        self.__dict__["_r"] = r

    def open(self):
        pass

    def close(self):
        pass

    def __getattr__(self, k):
        return getattr(self._r, k)

    def __setattr__(self, k, v):
        setattr(self._r, k, v)


def step_report(ctx):
    d = ctx.data
    say(f"== measured for '{d.get('machine')}' ({ctx.results_path})")
    for name in STEP_ORDER:
        st = d["steps"].get(name)
        say(f"  {name:<10} {'-' if not st else st['time']}"
            f"{'  (simulated)' if st and st.get('sim') else ''}")
    say("\n  value                       before     measured")
    for k, v in d["recommend"].items():
        say(f"  {k:<28} {BUILT_IN.get(k, current_value(k))!s:>10}  {v!s:>10}")
    return 0


def step_save(ctx):
    rec = ctx.data["recommend"]
    if not rec:
        say("nothing measured yet")
        return 1
    path = ctx.args.profile or vehicle.profile_path() or vehicle.DEFAULT_PROFILE
    prof = {}
    if os.path.isfile(path):
        with open(path) as fh:
            prof = json.load(fh)
    prof["machine"] = ctx.args.machine
    prof["measured"] = datetime.now().strftime("%Y-%m-%d")
    prof.setdefault("nav", {})
    for k, v in rec.items():
        if k in PROFILE_TOP:
            prof[PROFILE_TOP[k]] = v
        else:
            prof["nav"][k] = v
    prof["source"] = ctx.results_path
    say(json.dumps(prof, indent=2))
    if not ctx.confirm(f"Write this to {path}? Every program then uses it."):
        return 1
    with open(path, "w") as fh:
        json.dump(prof, fh, indent=2)
    say(f"  written: {path}")
    say("  check it:  python3 amiga_nav.py plan <a teach file>  (first line "
        "names the machine)")
    return 0


STEP_ORDER = ["check", "heading", "centre", "breakaway", "yaw", "swing",
              "arc", "course"]
STEPS = {"check": step_check, "heading": step_heading, "centre": step_centre,
         "breakaway": step_breakaway, "yaw": step_yaw, "swing": step_swing,
         "arc": step_arc, "course": step_course, "report": step_report,
         "save": step_save}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("step", choices=list(STEPS) + ["all"])
    p.add_argument("--machine", default=None,
                   help="name for the results / profile (default: the "
                        "profile's, else 'dallas')")
    p.add_argument("--yes", action="store_true", help="do not ask before moving")
    p.add_argument("--fresh", action="store_true",
                   help="ignore earlier results for this machine")
    p.add_argument("--results", help="results file (default ~/logs/commission_<machine>.json)")
    p.add_argument("--profile", help="profile to write (default ~/amiga_vehicle.json)")
    p.add_argument("--sim", action="store_true", help="simulated machine")
    p.add_argument("--sim-truth", help="JSON overriding sim_amiga.DEFAULT_TRUTH")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--ready-timeout", type=float, default=600.0)
    p.add_argument("--dist", type=float, default=5.0, help="heading: metres")
    p.add_argument("--deg", type=float, default=300.0, help="centre: degrees per spin")
    p.add_argument("--rate", type=int, default=None, help="centre: mrad/s")
    p.add_argument("--speeds", default="15,20,25,30,35,40,50,60,80")
    p.add_argument("--rates", default="80,120,160,200,240,280,320,380,450")
    p.add_argument("--yaw-rates", default="120,200,280,360,450")
    p.add_argument("--secs", type=float, default=3.0, help="yaw: seconds per turn")
    p.add_argument("--slews", default="none,3000,1500,800,400")
    p.add_argument("--accels", default="none,1000,500,250")
    p.add_argument("--settle-secs", type=float, default=4.0)
    p.add_argument("--radii", default="3.0,2.5,2.0,1.5")
    p.add_argument("--max-dev", type=float, default=0.15,
                   help="arc: most a held curve may wander, m")
    p.add_argument("--rows", type=int, default=2)
    p.add_argument("--plots", type=int, default=4)
    p.add_argument("--side", choices=("left", "right"), default="right")
    p.add_argument("--runout", type=float, default=2.0,
                   help="course: run-out past the last plot, m")
    p.add_argument("--dwell", type=float, default=2.0)
    args = p.parse_args(argv)

    def ints(s):
        return [int(v) for v in s.split(",") if v.strip()]

    def opt(s):
        return [None if v.strip().lower() == "none" else float(v)
                for v in s.split(",") if v.strip()]
    args.speeds, args.rates = ints(args.speeds), ints(args.rates)
    args.yaw_rates = ints(args.yaw_rates)
    args.slews, args.accels = opt(args.slews), opt(args.accels)
    args.radii = [float(v) for v in args.radii.split(",") if v.strip()]
    if args.machine is None:
        args.machine = vehicle.PROFILE.get("machine", "dallas")
    if args.sim and not args.results:
        import tempfile
        args.results = os.path.join(tempfile.gettempdir(),
                                    f"commission_sim_{args.machine}.json")
        if not args.profile:
            args.profile = os.path.join(tempfile.gettempdir(),
                                        "amiga_vehicle_sim.json")

    snapshot_built_in()
    ctx = Ctx(args)
    if ctx.data["recommend"] and args.step not in ("report", "save"):
        apply_recommend(ctx.data["recommend"])
        say(f"(using values measured earlier for '{args.machine}': "
            f"{', '.join(ctx.data['recommend'])})")
    steps = STEP_ORDER if args.step == "all" else [args.step]
    rc = 0
    try:
        for name in steps:
            rc = STEPS[name](ctx)
            if rc:
                say(f"\n{name} did not finish - fix that, then run "
                    f"`commission.py {name}` again")
                break
        if args.step == "all" and rc == 0:
            step_report(ctx)
            rc = step_save(ctx)
    except KeyboardInterrupt:
        say("\nstopped")
        rc = 130
    finally:
        if ctx.rover is not None:
            for _ in range(5):
                try:
                    ctx.rover.send(nav.STATE_AUTO_READY, 0, 0)
                except Exception:
                    break
            ctx.rover.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
