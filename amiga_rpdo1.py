#!/usr/bin/env python3
"""
amiga_rpdo1.py - stream AmigaRpdo1 to the farm-ng Amiga dashboard over raw SocketCAN.

Phase 0, step 2. Stdlib only. No python-can, no farm-ng packages.

    ready       request AUTO_READY  (state 4), zero velocity, forever   [phase A]
    active      request AUTO_ACTIVE (state 5), zero velocity, forever   [phase A]
    drive       state 5, 50 mm/s for 2 s, ramp down, back to READY      [phase B]
    kill-test   state 5, 50 mm/s, forever - SIGKILL this from elsewhere [phase C]

Frame layout, 0x20E, 8 bytes (firmware past v0.1.9), mirroring the
validated AmigaTpdo1 decode at 0x18E:

    byte 0     requested control state (uint8)
    bytes 1-2  linear speed, int16 LE, mm/s
    bytes 3-4  angular rate, int16 LE, mrad/s
    bytes 5-7  PTO / H-bridge / reserved - sent as zero

WHEELS OFF THE GROUND. This commands motion.
"""

import os
import socket
import struct
import sys
import time

# --- configuration ----------------------------------------------------------

import devices

IFACE = devices.CAN_IFACE

COBID_RPDO1_DASH = 0x20E   # dashboard's RPDO1 inbox - nothing else talks here
COBID_TPDO1_DASH = 0x18E   # dashboard's TPDO1 broadcast
COBID_HEARTBEAT_DASH = 0x70E

RATE_HZ = 20.0             # dashboard watchdog reverts below ~5 Hz
MAX_SPEED_MM_S = 100       # hard clamp, phase 2 safety
DRIVE_SPEED_MM_S = 50
DRIVE_SECONDS = 2.0

STATE_NAMES = {
    0: "BOOT", 1: "MANUAL_READY", 2: "MANUAL_ACTIVE", 3: "CC_ACTIVE",
    4: "AUTO_READY", 5: "AUTO_ACTIVE", 6: "ESTOPPED",
}

CAN_FRAME = struct.Struct("<IB3x8s")   # can_id, can_dlc, pad, data

# --- frame helpers ----------------------------------------------------------


def build_rpdo1(state, speed_mm_s=0, angular_mrad_s=0):
    speed = max(-MAX_SPEED_MM_S, min(MAX_SPEED_MM_S, int(speed_mm_s)))
    angular = int(angular_mrad_s)
    data = struct.pack("<Bhh3x", state & 0xFF, speed, angular)
    return CAN_FRAME.pack(COBID_RPDO1_DASH, 8, data)


def decode_tpdo1(data):
    state, speed, angular = struct.unpack_from("<Bhh", data)
    soc = data[7]
    return state, speed, angular, soc


def open_socket():
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((IFACE,))
    s.setblocking(False)
    return s


# --- bus state --------------------------------------------------------------


def wait_for_operational(sock, timeout=20.0):
    """Gate on the dashboard heartbeat reading NMT OPERATIONAL (0x05)."""
    print(f"waiting for dashboard NMT 0x05 on 0x{COBID_HEARTBEAT_DASH:03X} ...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = read_frame(sock)
        if frame and frame[0] == COBID_HEARTBEAT_DASH:
            nmt = frame[1][0]
            if nmt == 0x05:
                print("dashboard OPERATIONAL")
                return True
            print(f"  dashboard NMT 0x{nmt:02X}")
        time.sleep(0.005)
    print("TIMEOUT - dashboard never reached OPERATIONAL. Is the Amiga powered?")
    return False


def safe_send(sock, frame):
    """Send, or report a dead bus and say so.

    On e-stop the dashboard, motor controllers and pendant all go silent and
    our adapter is the only powered node left. A lone node gets no ACK, the
    TX queue (qlen 10) fills, and send raises ENOBUFS after ten frames. That
    is correct behaviour, not a fault - but there is nothing left to command,
    so break rather than retry. See docs/watchdog-and-estop.md.
    """
    try:
        sock.send(frame)
        return True
    except OSError as e:
        if e.errno == 105:       # ENOBUFS
            print("\nCAN TX queue full - no other node is ACKing.")
            print("Almost certainly an e-stop: the whole bus is down.")
        else:
            print(f"\nCAN send failed: {e}")
        return False


def read_frame(sock):
    try:
        raw = sock.recv(16)
    except (BlockingIOError, OSError):
        return None
    can_id, dlc, data = CAN_FRAME.unpack(raw)
    return can_id & 0x1FFFFFFF, data[:dlc]


# --- main loop --------------------------------------------------------------


def stream(sock, state_fn, duration=None, label=""):
    """Send RPDO1 at RATE_HZ. state_fn(t) -> (state, speed, angular)."""
    period = 1.0 / RATE_HZ
    t0 = time.monotonic()
    next_tx = t0
    last_report = None
    sent = 0

    print(f"\n--- {label} --- streaming 0x{COBID_RPDO1_DASH:03X} "
          f"at {RATE_HZ:.0f} Hz. Ctrl-C to stop.\n")

    while True:
        now = time.monotonic()
        t = now - t0

        if duration is not None and t >= duration:
            break

        if now >= next_tx:
            state, speed, angular = state_fn(t)
            if not safe_send(sock, build_rpdo1(state, speed, angular)):
                print(f"stopping after {sent} frames.")
                break
            sent += 1
            next_tx += period
            if next_tx < now:          # we fell behind; resync
                next_tx = now + period

        # drain RX, report dashboard state changes
        while True:
            frame = read_frame(sock)
            if frame is None:
                break
            if frame[0] == COBID_TPDO1_DASH and len(frame[1]) >= 8:
                meas = decode_tpdo1(frame[1])
                report = (meas[0], meas[1], meas[2])
                if report != last_report:
                    name = STATE_NAMES.get(meas[0], "?")
                    print(f"{t:7.2f}  state {meas[0]} {name:<13} "
                          f"speed {meas[1]:>5} mm/s  ang {meas[2]:>5} mrad/s  "
                          f"soc {meas[3]}%")
                    last_report = report

        time.sleep(0.002)

    return sent


def send_ready_then_stop(sock, n=10):
    """Clean exit: request AUTO_READY a few times, then stop talking."""
    for _ in range(n):
        if not safe_send(sock, build_rpdo1(4, 0, 0)):
            return
        time.sleep(1.0 / RATE_HZ)
    print("\nrequested AUTO_READY and stopped streaming.")


# --- commands ---------------------------------------------------------------


def cmd_ready(sock):
    stream(sock, lambda t: (4, 0, 0), label="PHASE A - request AUTO_READY, zero velocity")


def cmd_active(sock):
    stream(sock, lambda t: (5, 0, 0), label="PHASE A - request AUTO_ACTIVE, zero velocity")


def cmd_drive(sock):
    ramp = 0.5

    def profile(t):
        if t < ramp:
            v = DRIVE_SPEED_MM_S * (t / ramp)
        elif t < ramp + DRIVE_SECONDS:
            v = DRIVE_SPEED_MM_S
        elif t < ramp + DRIVE_SECONDS + ramp:
            v = DRIVE_SPEED_MM_S * (1 - (t - ramp - DRIVE_SECONDS) / ramp)
        else:
            v = 0
        return (5, v, 0)

    total = ramp + DRIVE_SECONDS + ramp + 1.0
    stream(sock, profile, duration=total,
           label=f"PHASE B - {DRIVE_SPEED_MM_S} mm/s for {DRIVE_SECONDS} s")
    send_ready_then_stop(sock)


def cmd_kill_test(sock):
    print("\nPHASE C - from another terminal:  kill -9 $(pgrep -f amiga_rpdo1)")
    print("Then confirm the wheels stop and 0x18E reverts to AUTO_READY.\n")
    stream(sock, lambda t: (5, DRIVE_SPEED_MM_S, 0),
           label=f"PHASE C - holding {DRIVE_SPEED_MM_S} mm/s, awaiting SIGKILL")


COMMANDS = {
    "ready": cmd_ready,
    "active": cmd_active,
    "drive": cmd_drive,
    "kill-test": cmd_kill_test,
}


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in COMMANDS:
        print(__doc__)
        print("usage: amiga_rpdo1.py {" + "|".join(COMMANDS) + "}")
        return 2

    cmd = sys.argv[1]
    print("=" * 68)
    print("  WHEELS OFF THE GROUND. Pendant in hand. Circle button disengages.")
    print("=" * 68)

    try:
        sock = open_socket()
    except OSError as e:
        print(f"cannot open {IFACE}: {e}")
        return 1

    if not wait_for_operational(sock):
        return 1

    try:
        COMMANDS[cmd](sock)
    except KeyboardInterrupt:
        if cmd in ("active", "kill-test"):
            send_ready_then_stop(sock)
        else:
            print("\nstopped.")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
