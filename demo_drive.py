#!/usr/bin/env python3
"""
demo_drive.py - drive the Amiga from the keyboard. Hold a key to move,
let go to stop.

    w / s      forward / backward          (hold)
    a / d      turn left / right in place  (hold)
    q / e      forward arc left / right    (hold)
    v / c      faster / slower             (tap)
    space      stop now
    x          quit  (Ctrl-C also quits; both stop the machine first)

    python3 demo_drive.py              drive the Amiga
    python3 demo_drive.py --dry-run    practise the keys, no robot, no CAN

ARM AUTO ON THE DASHBOARD SCREEN (the auto tab, not the start button) after
starting this. The pendant X button is the real abort and always works.

Why hold, not tap: a terminal reports key PRESSES only - there is no "key
released" event. Holding a key makes the keyboard auto-repeat it, so this
treats "still receiving repeats" as "still held" and stops once they have
been missing for HOLD_TIMEOUT_S. See the notes at the bottom of this file.

Do not spin in place on turf for long: three spins took the motors to 220 F.
Use q / e arcs to turn while moving.
"""

import argparse
import select
import socket
import struct
import sys
import termios
import time
import tty

# ---------------------------------------------------------------------------
#  Settings
# ---------------------------------------------------------------------------

SPEED_LEVELS_MM_S = [100, 200, 300, 500, 700, 1000]
START_LEVEL = 1                  # 200 mm/s
TURN_MRAD_S = 400                # turn in place. Below ~220 it does not turn.
ARC_RATIO = 0.6                  # q / e: angular (rad/s) per linear (m/s)
ARC_MIN_MRAD_S = 250             # ...but never below the turning breakaway
HOLD_TIMEOUT_S = 0.6             # stop this long after the last key repeat.
                                 # Must exceed the keyboard's repeat delay
                                 # (typically 0.25 - 0.5 s) or the first
                                 # press stutters.

RATE_HZ = 20.0                   # RPDO1 rate. The Dashboard's watchdog trips
                                 # ~550 ms after the last frame, and then
                                 # auto has to be re-armed by hand.

IFACE = "can0"
COBID_RPDO1 = 0x20E
COBID_TPDO1 = 0x18E
CAN_FRAME = struct.Struct("<IB3x8s")
AUTO_READY, AUTO_ACTIVE = 4, 5
STATE_NAMES = {0: "BOOT", 1: "MANUAL_READY", 2: "MANUAL_ACTIVE",
               3: "CC_ACTIVE", 4: "AUTO_READY", 5: "AUTO_ACTIVE",
               6: "ESTOPPED"}

# key -> (linear sign, angular kind). Positive mrad/s turns LEFT.
MOTIONS = {
    "w": (+1, None), "s": (-1, None),
    "a": (0, "left"), "d": (0, "right"),
    "q": (+1, "arc_left"), "e": (+1, "arc_right"),
}


# ---------------------------------------------------------------------------
#  Driving logic - no hardware, no terminal
# ---------------------------------------------------------------------------

class Driver:
    def __init__(self):
        self.level = START_LEVEL
        self.motion = None           # key currently held, or None
        self.last_key_t = 0.0
        self.quit = False

    @property
    def speed(self):
        return SPEED_LEVELS_MM_S[self.level]

    def key(self, c, now):
        c = c.lower()
        if c in MOTIONS:
            self.motion = c
            self.last_key_t = now
        elif c == "v":
            self.level = min(self.level + 1, len(SPEED_LEVELS_MM_S) - 1)
        elif c == "c":
            self.level = max(self.level - 1, 0)
        elif c == " ":
            self.motion = None
        elif c == "x":
            self.motion = None
            self.quit = True

    def command(self, now):
        """(linear mm/s, angular mrad/s) to send right now."""
        if self.motion and now - self.last_key_t > HOLD_TIMEOUT_S:
            self.motion = None                     # key let go
        if self.motion is None:
            return 0, 0
        sign, kind = MOTIONS[self.motion]
        lin = sign * self.speed
        if kind is None:
            ang = 0
        elif kind in ("left", "right"):
            ang = TURN_MRAD_S
        else:
            ang = max(ARC_MIN_MRAD_S, int(ARC_RATIO * self.speed))
        if kind in ("right", "arc_right"):
            ang = -ang
        return lin, ang


# ---------------------------------------------------------------------------
#  CAN
# ---------------------------------------------------------------------------

def rpdo1(state, lin, ang):
    return CAN_FRAME.pack(COBID_RPDO1, 8,
                          struct.pack("<Bhh3x", state, int(lin), int(ang)))


class Can:
    def __init__(self):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW,
                                  socket.CAN_RAW)
        self.sock.bind((IFACE,))
        self.sock.setblocking(False)

    def send(self, state, lin, ang):
        self.sock.send(rpdo1(state, lin, ang))

    def feedback(self):
        """Latest (state, speed mm/s, angular mrad/s) from 0x18E, or None."""
        latest = None
        while True:
            try:
                raw = self.sock.recv(16)
            except (BlockingIOError, OSError):
                return latest
            cid, dlc, data = CAN_FRAME.unpack(raw)
            if (cid & 0x1FFFFFFF) == COBID_TPDO1 and dlc >= 8:
                latest = struct.unpack_from("<Bhh", data)

    def close(self):
        self.sock.close()


class FakeCan:
    """--dry-run: echoes the command back as if the machine obeyed."""

    def __init__(self):
        self.last = (AUTO_ACTIVE, 0, 0)

    def send(self, state, lin, ang):
        self.last = (state, lin, ang)

    def feedback(self):
        return self.last

    def close(self):
        pass


# ---------------------------------------------------------------------------
#  Main loop
# ---------------------------------------------------------------------------

HELP = """
  hold  w / s   forward / back        hold  q / e   arc left / right
  hold  a / d   turn in place         tap   v / c   faster / slower
        space   stop                        x       quit

  [speed]  cmd = what is sent,  got = what the Amiga reports
  (mm/s forward, mrad/s turning; positive turn = left)
"""


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dry-run", action="store_true",
                   help="no CAN, no robot - practise the keys")
    args = p.parse_args()

    if not sys.stdin.isatty():
        print("run this in a terminal - it reads single keypresses")
        return 1

    try:
        can = FakeCan() if args.dry_run else Can()
    except OSError as e:
        print(f"cannot open {IFACE}: {e}\n"
              f"check: ip -details link show {IFACE}")
        return 1

    print(HELP)
    if args.dry_run:
        print("  DRY RUN - nothing is sent to the robot\n")
    else:
        print("  Now arm AUTO on the Dashboard screen. Pendant X = abort.\n")

    drv = Driver()
    period = 1.0 / RATE_HZ
    next_tx = time.monotonic()
    meas = None

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)          # remember the terminal's settings
    try:
        tty.setcbreak(fd)                  # keys arrive one at a time, unechoed
        while not drv.quit:
            now = time.monotonic()

            # every key waiting right now - never blocks
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if not ch:
                    break
                drv.key(ch, now)

            lin, ang = drv.command(now)

            if now >= next_tx:             # 20 Hz, moving or not
                try:
                    can.send(AUTO_ACTIVE, lin, ang)
                except OSError as e:
                    print(f"\r\nCAN send failed ({e.errno}) - e-stop pressed? "
                          f"Release it, power-cycle the Amiga, check can0.")
                    break
                next_tx += period
                if next_tx < now:
                    next_tx = now + period

            fb = can.feedback()
            if fb is not None:
                meas = fb
            # one short line (< 80 columns), redrawn in place with \r
            if meas is None:
                status = "no feedback yet"
            else:
                name = STATE_NAMES.get(meas[0], "?")
                arm = "" if meas[0] == AUTO_ACTIVE else " <-ARM AUTO"
                status = f"{name:<13} got {meas[1]:+5} {meas[2]:+5}{arm}"
            sys.stdout.write(
                f"\r  [{drv.speed:>4}]  cmd {lin:+5} {ang:+5}  {status}   ")
            sys.stdout.flush()
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)   # terminal back
        print("\n\nstopping")
        for _ in range(10):                # zero, then hand back AUTO_READY
            try:
                can.send(AUTO_READY, 0, 0)
            except OSError:
                break
            time.sleep(period)
        can.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())


# ---------------------------------------------------------------------------
#  How the keyboard reading works
# ---------------------------------------------------------------------------
#
# 1. Normally the terminal is in "canonical" mode: it collects what you type
#    into a line, shows it on screen, and only hands it to the program when
#    you press Enter. Useless for driving.
#
# 2. tty.setcbreak(fd) switches that off. Each keypress is handed over the
#    moment it happens, and is not echoed. Ctrl-C still works.
#
# 3. sys.stdin.read(1) would WAIT for a key - and while it waits, nothing is
#    sent on CAN, the Dashboard's watchdog trips after ~550 ms, and auto has
#    to be re-armed by hand. So the loop first asks select.select(..., 0):
#    "is a key waiting?", with a timeout of zero, so it never waits. Only if
#    the answer is yes does it read(1), which then returns at once.
#
# 4. The loop runs ~200 times a second: read any waiting keys, work out the
#    command, send a CAN frame every 50 ms whether or not anything changed.
#
# 5. There is no key-release event in a terminal. Holding w makes the
#    keyboard send w, then (after a ~0.25-0.5 s delay) w w w w at ~30/s.
#    So "held" means "a repeat arrived recently", and the machine stops
#    HOLD_TIMEOUT_S after the last one. That is also why it keeps moving for
#    about half a second after you let go - plus the Dashboard's own braking.
#
# 6. Over ssh it is the same: your laptop's keyboard repeats, ssh carries the
#    characters to the Pi, and the Pi's terminal hands them to this program.
#    A laggy network can make the motion stutter; a longer HOLD_TIMEOUT_S
#    smooths that at the cost of a longer stop.
#
# 7. The terminal's settings are saved before and restored in `finally`, so
#    they come back however the program ends. If a crash ever leaves your
#    terminal not echoing what you type, type `reset` and press Enter.
