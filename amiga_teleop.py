#!/usr/bin/env python3
"""
amiga_teleop.py - WASD teleop over raw SocketCAN. Phase 0, step 3.

Purpose beyond driving: exercise the two parts of AmigaRpdo1 that have never
been on the wire - angular rate (bytes 3-4) and negative linear speed.

    w / s      linear  +/- 25 mm/s per press
    a / d      angular +/- 50 mrad/s per press
    space      zero both, immediately
    x          zero and drop to AUTO_READY
    q          quit cleanly (requests AUTO_READY first)

Setpoints latch - press and they hold, so a steady turn can be observed on
0x18E. Use --deadman on the ground, where latching is the wrong behaviour.

ARM AUTO ON THE DASHBOARD SCREEN FIRST. Wheels off the ground for the first
angular test - a commanded turn spins the sides in opposite directions.
"""

import argparse
import os
import select
import socket
import struct
import sys
import termios
import time
import tty

IFACE = "can0"
COBID_RPDO1 = 0x20E
COBID_TPDO1 = 0x18E
COBID_HEARTBEAT = 0x70E

RATE_HZ = 20.0
LIN_STEP = 5           # mm/s per keypress - small, for the breakaway test
ANG_STEP = 50          # mrad/s per keypress
LIN_MAX = 1000         # mm/s clamp
ANG_MAX = 1000         # mrad/s clamp
DEADMAN_S = 1.0        # with --deadman: zero out after this long with no key

STATE_NAMES = {0: "BOOT", 1: "MANUAL_READY", 2: "MANUAL_ACTIVE", 3: "CC_ACTIVE",
               4: "AUTO_READY", 5: "AUTO_ACTIVE", 6: "ESTOPPED"}

CAN_FRAME = struct.Struct("<IB3x8s")


def build_rpdo1(state, lin, ang):
    lin = max(-LIN_MAX, min(LIN_MAX, int(lin)))
    ang = max(-ANG_MAX, min(ANG_MAX, int(ang)))
    return CAN_FRAME.pack(COBID_RPDO1, 8, struct.pack("<Bhh3x", state, lin, ang))


def read_frame(sock):
    try:
        raw = sock.recv(16)
    except (BlockingIOError, OSError):
        return None
    can_id, dlc, data = CAN_FRAME.unpack(raw)
    return can_id & 0x1FFFFFFF, data[:dlc]


def wait_operational(sock, timeout=20.0):
    print(f"waiting for dashboard NMT 0x05 on 0x{COBID_HEARTBEAT:03X} ...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        f = read_frame(sock)
        if f and f[0] == COBID_HEARTBEAT and f[1][0] == 0x05:
            print("dashboard OPERATIONAL\n")
            return True
        time.sleep(0.005)
    print("TIMEOUT - is the Amiga powered?")
    return False


LEGEND = """\
  w / s   linear  +/- {lin} mm/s        a / d   angular +/- {ang} mrad/s
  space   zero both                  x       zero + AUTO_READY
  q       quit

  Arm auto on the Dashboard screen if state is not 5.
""".format(lin=LIN_STEP, ang=ANG_STEP)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--deadman", action="store_true",
                   help=f"zero velocity after {DEADMAN_S}s with no keypress")
    args = p.parse_args()

    print("=" * 70)
    print("  WHEELS OFF THE GROUND for the first angular test.")
    print("  A commanded turn drives the two sides in opposite directions.")
    print("=" * 70)
    print(LEGEND)

    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((IFACE,))
        sock.setblocking(False)
    except OSError as e:
        print(f"cannot open {IFACE}: {e}")
        return 1

    if not wait_operational(sock):
        return 1

    lin = ang = 0
    state_req = 5
    meas = (None, 0, 0)
    last_key = time.monotonic()
    period = 1.0 / RATE_HZ
    next_tx = time.monotonic()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            now = time.monotonic()

            # --- keyboard
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1).lower()
                last_key = now
                if c == "w":
                    lin = min(LIN_MAX, lin + LIN_STEP)
                elif c == "s":
                    lin = max(-LIN_MAX, lin - LIN_STEP)
                elif c == "a":
                    ang = min(ANG_MAX, ang + ANG_STEP)
                elif c == "d":
                    ang = max(-ANG_MAX, ang - ANG_STEP)
                elif c == " ":
                    lin = ang = 0
                elif c == "x":
                    lin = ang = 0
                    state_req = 4
                elif c == "q":
                    break

            if args.deadman and (now - last_key) > DEADMAN_S and (lin or ang):
                lin = ang = 0

            # --- transmit
            if now >= next_tx:
                try:
                    sock.send(build_rpdo1(state_req, lin, ang))
                except OSError as e:
                    sys.stdout.write("\r\n")
                    print(f"TX failed ({e.errno}) - bus silent? e-stop? stopping.")
                    break
                next_tx += period
                if next_tx < now:
                    next_tx = now + period

            # --- receive
            while True:
                f = read_frame(sock)
                if f is None:
                    break
                if f[0] == COBID_TPDO1 and len(f[1]) >= 8:
                    s, ms, ma = struct.unpack_from("<Bhh", f[1])
                    meas = (s, ms, ma)

            name = STATE_NAMES.get(meas[0], "?")
            sys.stdout.write(
                f"\r  cmd {lin:>5} mm/s {ang:>5} mrad/s  |  "
                f"state {meas[0]} {name:<13} meas {meas[1]:>5} mm/s "
                f"{meas[2]:>5} mrad/s   ")
            sys.stdout.flush()
            time.sleep(0.002)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n\nstopping - requesting AUTO_READY")
        for _ in range(10):
            try:
                sock.send(build_rpdo1(4, 0, 0))
            except OSError:
                break
            time.sleep(period)
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
