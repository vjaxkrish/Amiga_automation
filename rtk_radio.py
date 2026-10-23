#!/usr/bin/env python3
"""
rtk_radio.py - RTCM3 from the F9P over the SiK radio pair into the LG580P.

Phase 0, step 4, stage 2. Open question 3: actual SiK range across the plot.

    F9P (USB) -> this script -> radio A (USB) ))) air ))) radio B (USB)
                                                       -> this script -> LG580P (USB)

Both radios are on the laptop for this test, so one process sees both ends.
In the field, radio A sits at the base with the F9P and no computer.

    bridge      run the link, report RTCM out, bytes in, and rover fix
    airtest     send a known pattern A -> B and report loss, no GNSS involved
"""

import argparse
import sys
import time

import serial
from pyubx2 import UBXReader

import devices

F9P_PORT = devices.F9P_PORT
LG580P_PORT = devices.LG580P_PORT
RADIO_A = devices.RADIO_A            # base side
RADIO_B = devices.RADIO_B            # rover side

F9P_BAUD = devices.F9P_BAUD
LG580P_BAUD = devices.LG580P_BAUD
RADIO_BAUD = devices.RADIO_BAUD


def cmd_bridge(args):
    print("F9P -> radio A ))) radio B -> LG580P\n")
    counts = {}
    tx_bytes = 0
    rx_bytes = 0
    gga = None
    last = time.time()

    with serial.Serial(F9P_PORT, F9P_BAUD, timeout=0.05) as f9p, \
         serial.Serial(RADIO_A, RADIO_BAUD, timeout=0.05) as ra, \
         serial.Serial(RADIO_B, RADIO_BAUD, timeout=0.05) as rb, \
         serial.Serial(LG580P_PORT, LG580P_BAUD, timeout=0.05) as lg:

        ubr = UBXReader(f9p, protfilter=6)
        lg_buf = b""

        while True:
            # F9P -> radio A
            try:
                raw, parsed = ubr.read()
            except Exception:
                raw, parsed = None, None
            if raw and parsed is not None:
                ident = str(parsed.identity)
                if ident.isdigit():
                    ra.write(raw)
                    tx_bytes += len(raw)
                    counts[ident] = counts.get(ident, 0) + 1

            # radio B -> LG580P
            air = rb.read(1024)
            if air:
                lg.write(air)
                rx_bytes += len(air)

            # rover status
            lg_buf += lg.read(1024)
            while b"\n" in lg_buf:
                line, lg_buf = lg_buf.split(b"\n", 1)
                s = line.decode("ascii", "ignore").strip()
                if s.startswith("$GNGGA"):
                    f = s.split(",")
                    if len(f) > 13:
                        gga = (f[6], f[7], f[8], f[13])

            if time.time() - last >= 2.0:
                last = time.time()
                loss = 100.0 * (1 - rx_bytes / tx_bytes) if tx_bytes else 0.0
                if gga:
                    q, nsat, hdop, age = gga
                    label = {"0": "NO FIX", "1": "standalone", "2": "DGPS/SBAS",
                             "4": "RTK FIXED", "5": "RTK FLOAT"}.get(q, f"q={q}")
                    print(f"  {label:<12} sats {nsat:>2}  hdop {hdop:>5}  "
                          f"age {age or '-':>5}  |  air tx {tx_bytes:>7} "
                          f"rx {rx_bytes:>7}  loss {loss:5.1f}%")
                else:
                    print(f"  no GGA yet  |  air tx {tx_bytes} rx {rx_bytes}")


def cmd_airtest(args):
    """Pure link test. No GNSS. Walk radio B away and watch loss climb."""
    print(f"air test: {args.rate} packets/s of 64 bytes, Ctrl-C to stop\n")
    payload = bytes(range(64))
    sent = recv = 0
    last = time.time()

    with serial.Serial(RADIO_A, RADIO_BAUD, timeout=0.05) as ra, \
         serial.Serial(RADIO_B, RADIO_BAUD, timeout=0.05) as rb:
        period = 1.0 / args.rate
        nxt = time.time()
        while True:
            now = time.time()
            if now >= nxt:
                ra.write(payload)
                sent += len(payload)
                nxt += period
            recv += len(rb.read(1024))
            if now - last >= 2.0:
                last = now
                loss = 100.0 * (1 - recv / sent) if sent else 0.0
                bar = "#" * int(max(0, 40 - loss * 0.4))
                print(f"  tx {sent:>8}  rx {recv:>8}  loss {loss:5.1f}%  {bar}")
            time.sleep(0.002)


COMMANDS = {"bridge": cmd_bridge, "airtest": cmd_airtest}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=COMMANDS)
    p.add_argument("--rate", type=float, default=5.0,
                   help="airtest: packets per second (default 5)")
    args = p.parse_args()
    try:
        return COMMANDS[args.command](args)
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
