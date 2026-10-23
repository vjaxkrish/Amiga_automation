#!/usr/bin/env python3
"""
lg580p_cmd.py - send Quectel PQTM commands to the LG580P and show replies.

Checksums are computed automatically: give the command body without the
leading '$' and without the '*XX'.

    python3 lg580p_cmd.py PQTMVERNO
    python3 lg580p_cmd.py PQTMCFGNAVMODE,R
    python3 lg580p_cmd.py PQTMCFGMSGRATE,W,PQTMTAR,1,1
    python3 lg580p_cmd.py --listen 10          # just watch the stream

Useful sequence for dual-antenna heading:

    PQTMVERNO                        firmware version
    PQTMCFGNAVMODE,R                 current nav mode  (want dual-antenna)
    PQTMCFGMSGRATE,W,PQTMTAR,1,1     enable attitude output
    PQTMCFGMSGRATE,W,THS,1           enable NMEA true heading
    PQTMSAVEPAR                      save to NVM
    PQTMSRR                          reboot
"""

import argparse
import sys
import time

import serial

import devices

PORT = devices.LG580P_PORT
BAUD = devices.LG580P_BAUD


def nmea(body):
    ck = 0
    for c in body:
        ck ^= ord(c)
    return f"${body}*{ck:02X}\r\n"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("command", nargs="?", help="PQTM body, no $ and no *XX")
    p.add_argument("--listen", type=float, metavar="SEC",
                   help="just read the stream for SEC seconds")
    p.add_argument("--wait", type=float, default=2.0,
                   help="seconds to collect replies (default 2)")
    p.add_argument("--filter", default="",
                   help="only print lines containing this substring")
    args = p.parse_args()

    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        if args.listen:
            deadline = time.time() + args.listen
            buf = b""
            while time.time() < deadline:
                buf += ser.read(512)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("ascii", "ignore").strip()
                    if s and (not args.filter or args.filter in s):
                        print(s)
            return 0

        if not args.command:
            p.print_help()
            return 2

        out = nmea(args.command)
        print(f">>> {out.strip()}")
        ser.reset_input_buffer()
        ser.write(out.encode("ascii"))

        deadline = time.time() + args.wait
        buf = b""
        seen = False
        while time.time() < deadline:
            buf += ser.read(512)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", "ignore").strip()
                if s.startswith("$PQTM") or s.startswith("$PAIR"):
                    print(f"<<< {s}")
                    seen = True
        if not seen:
            print("(no PQTM reply - command may be unsupported on this firmware)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
