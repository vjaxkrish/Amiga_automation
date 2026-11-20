"""
sim_amiga.py - a simulated Amiga with HIDDEN true properties, for testing
commission.py (and trying it on a laptop) without the machine.

    python3 commission.py --sim all          the whole commissioning run
    python3 commission.py --sim --sim-truth '{"heading_offset_deg": 90}' heading

The truth is what commission.py has to find: the receiver's heading offset,
where the antenna sits relative to the turning centre (forward AND
sideways), how much of a commanded turn the ground gives (rolling and in
place), the slowest speed and turn rate that move it at all, how far it
walks backward while pivoting, and a mast that swings when the machine is
jerked. Defaults are a guess at the 4 ft Dallas machine, NOT measurements.

The controller code runs unchanged against it: it is an amiga_nav.Rover
whose poll() reads the simulated receiver and CAN instead of the hardware,
and whose heading goes through vehicle.true_heading() like the real one.
"""

import math
import random
import struct

import amiga_nav as nav
import vehicle

DEFAULT_TRUTH = {
    "heading_offset_deg": 180.0,   # raw receiver heading = true - this
    "antenna_fwd_m": 0.62,         # antenna ahead of the turning centre
    "antenna_lat_m": 0.04,         # ... and to the LEFT of the centreline
    "roll_eff": 0.55,              # rotation achieved / commanded, rolling
    "pivot_eff": 0.6,              # ... turning in place
    "pivot_floor_mrad_s": 160,     # in-place turn below this: no rotation
    "lin_floor_mm_s": 28,          # forward command below this: no motion
    "pivot_walk_m_per_rad": 0.10,  # centre walks backward while pivoting
    "crab": 0.0,                   # sideways drift per metre, + = right
    "swing_hz": 1.6,               # mast natural frequency
    "swing_zeta": 0.08,            # mast damping ratio (light)
    "swing_yaw_m_per_rad_s2": 0.015,   # antenna lateral swing per yaw accel
    "swing_yaw_deg_per_rad_s2": 0.6,   # heading swing per yaw accel
    "swing_lin_m_per_m_s2": 0.03,      # antenna fore-aft swing per accel
    "pos_noise_m": 0.005,
    "hdg_noise_deg": 0.08,
    "lag_s": 0.25,                 # wheels follow commands with this lag
}


class FakeTime:
    """Stands in for the time module: sleep() advances the clock at once,
    so a 20 Hz control loop runs as fast as Python can go."""

    def __init__(self, t=1000.0, limit_s=20000.0):
        self.t0 = t
        self.t = t
        self.limit = limit_s

    def monotonic(self):
        return self.t

    def time(self):
        return 1.795e9 + self.t

    def sleep(self, dt):
        self.t += max(dt, 0.001)
        if self.t - self.t0 > self.limit:
            raise RuntimeError("simulation ran too long - something is stuck")


class _Osc:
    """Damped spring, driven by an acceleration. `drive` is the deflection a
    steady acceleration would hold it at; a sudden one overshoots ~2x and
    rings at hz, decaying with zeta."""

    def __init__(self, hz, zeta):
        self.w = 2 * math.pi * hz
        self.z = zeta
        self.x = 0.0
        self.v = 0.0

    def step(self, dt, drive):
        n = max(1, int(dt / 0.005))
        h = dt / n
        for _ in range(n):
            a = -self.w * self.w * (self.x + drive) - 2 * self.z * self.w * self.v
            self.v += a * h
            self.x += self.v * h
        return self.x


class SimAmiga(nav.Rover):

    def __init__(self, clock, truth=None, lat0=32.7767, lon0=-96.7970,
                 seed=1, start_heading=0.0):
        super().__init__()
        self.truth = dict(DEFAULT_TRUTH)
        self.truth.update(truth or {})
        self.clock = clock
        self.frame = nav.Frame(lat0, lon0)
        self.rng = random.Random(seed)
        self.cx = self.cy = 0.0            # TRUE turning centre, metres
        self.h = start_heading             # TRUE heading, deg clockwise
        self.v = 0.0                       # actual forward speed, mm/s
        self.w = 0.0                       # actual yaw rate, mrad/s (+ left)
        self.cmd = (nav.STATE_AUTO_READY, 0, 0)
        self.t_last = None
        self.next_fix = 0.0
        self.osc_lat = _Osc(self.truth["swing_hz"], self.truth["swing_zeta"])
        self.osc_hdg = _Osc(self.truth["swing_hz"], self.truth["swing_zeta"])
        self.osc_lon = _Osc(self.truth["swing_hz"], self.truth["swing_zeta"])
        self.state = nav.STATE_AUTO_READY
        self.from_state = False

    # -- the hardware side ------------------------------------------------
    def open(self):
        outer = self

        class Sock:
            def send(self, raw):
                _, _, data = nav.CAN_FRAME.unpack(raw)
                outer.cmd = struct.unpack_from("<Bhh", data)

            def close(self):
                pass
        self.sock = Sock()
        print("SIMULATED Amiga (sim_amiga.py) - nothing real moves")

    def close(self):
        pass

    # -- physics ------------------------------------------------------------
    def _integrate(self, now):
        if self.t_last is None:
            self.t_last = now
            return
        dt = now - self.t_last
        if dt <= 0:
            return
        self.t_last = now
        T = self.truth
        st, lin, ang = self.cmd
        active = st == nav.STATE_AUTO_ACTIVE
        self.state = nav.STATE_AUTO_ACTIVE if active else nav.STATE_AUTO_READY
        lin_t = lin if active and abs(lin) >= T["lin_floor_mm_s"] else 0.0
        k = min(1.0, dt / T["lag_s"])
        v_old, w_old = self.v, self.w
        self.v += (lin_t - self.v) * k
        rolling = abs(self.v) > 50
        if not active:
            ang_t = 0.0
        elif rolling:
            ang_t = ang * T["roll_eff"]
        elif abs(ang) < T["pivot_floor_mrad_s"]:
            ang_t = 0.0
        else:
            ang_t = ang * T["pivot_eff"]
        self.w += (ang_t - self.w) * k
        lin_acc = (self.v - v_old) / 1000.0 / dt          # m/s^2
        yaw_acc = (self.w - w_old) / 1000.0 / dt          # rad/s^2
        dh = -math.degrees(self.w / 1000.0 * dt)          # + left = - heading
        self.h = (self.h + dh) % 360.0
        th = math.radians(self.h)
        d = self.v / 1000.0 * dt
        self.cx += d * math.sin(th) + T["crab"] * abs(d) * math.cos(th)
        self.cy += d * math.cos(th) - T["crab"] * abs(d) * math.sin(th)
        if not rolling and dh:
            back = T["pivot_walk_m_per_rad"] * abs(math.radians(dh))
            self.cx -= back * math.sin(th)
            self.cy -= back * math.cos(th)
        self.osc_lat.step(dt, T["swing_yaw_m_per_rad_s2"] * yaw_acc)
        self.osc_hdg.step(dt, T["swing_yaw_deg_per_rad_s2"] * yaw_acc)
        self.osc_lon.step(dt, T["swing_lin_m_per_m_s2"] * lin_acc)
        self.speed = int(round(self.v))

    def antenna_xy(self):
        """True antenna position: rigid offset plus the mast swing."""
        T = self.truth
        th = math.radians(self.h)
        fx, fy = math.sin(th), math.cos(th)          # forward
        lx, ly = -math.cos(th), math.sin(th)         # left
        f = T["antenna_fwd_m"] + self.osc_lon.x
        lft = T["antenna_lat_m"] + self.osc_lat.x
        return self.cx + f * fx + lft * lx, self.cy + f * fy + lft * ly

    def poll(self, now):
        self._integrate(now)
        self.t_can = now
        if now < self.next_fix and self.lat is not None:
            return
        self.next_fix = now + 0.1                    # receiver at 10 Hz
        T = self.truth
        ax, ay = self.antenna_xy()
        ax += self.rng.gauss(0, T["pos_noise_m"])
        ay += self.rng.gauss(0, T["pos_noise_m"])
        self.lat, self.lon = self.frame.to_ll(ax, ay)
        raw = (self.h + self.osc_hdg.x - T["heading_offset_deg"]
               + self.rng.gauss(0, T["hdg_noise_deg"])) % 360.0
        self.heading_raw = raw
        self.heading = vehicle.true_heading(raw)
        self.pitch = 0.4 + 15.0 * self.osc_lon.x      # deg; mast rocks fore-aft
        self.fix_q = "4"
        self.corr_age = 1.0
        self.t_pos = self.t_head = now
