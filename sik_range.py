#!/usr/bin/env python3
"""
sik_range.py - two-ended SiK link test. Open question 3: range across the plot.

Run ONE of these at each end. They do not need to talk to each other except
over the air.

  BASE (laptop, with radio A):
      python3 sik_range.py send

  ROVER (Pi, with radio B, walking the plot):
      python3 sik_range.py recv

Each packet carries a sequence number and a timestamp, so the receiver reports
loss, gaps and latency without needing a return path. Walk away from the base
and watch the numbers degrade; the distance where loss starts climbing is the
answer.

Also here, for the real rover job once range is known:

      python3 sik_range.py rover      radio B -> LG580P, report fix + age
"""

import argparse
import struct
import sys
import time

import serial

import devices

RADIO_A = devices.RADIO_A
RADIO_B = devices.RADIO_B
LG580P = devices.LG580P_PORT

RADIO_BAUD = devices.RADIO_BAUD
LG580P_BAUD = devices.LG580P_BAUD

MAGIC = b"\xAB\xCD"
PKT = struct.Struct("<2sId32s")      # magic, seq, tx_time, filler
FILLER = bytes(range(32))


def cmd_send(args):
    print(f"sending {args.rate}/s on radio A. Ctrl-C to stop.\n")
    seq = 0
    period = 1.0 / args.rate
    nxt = time.time()
    last_report = time.time()
    with serial.Serial(RADIO_A, RADIO_BAUD, timeout=0.05) as ra:
        while True:
            now = time.time()
            if now >= nxt:
                ra.write(PKT.pack(MAGIC, seq, now, FILLER))
                seq += 1
                nxt += period
                if nxt < now:
                    nxt = now + period
            if now - last_report >= 5.0:
                last_report = now
                print(f"  sent {seq} packets")
            time.sleep(0.002)


def cmd_recv(args):
    print("listening on radio B. Walk away from the base.\n")
    print(f"{'recv':>6} {'lost':>6} {'loss%':>7} {'gap_max':>8} "
          f"{'latency_ms':>11}   link")
    buf = b""
    first_seq = None
    last_seq = None
    received = 0
    lost = 0
    gap_max = 0
    lat_sum = 0.0
    lat_n = 0
    last_report = time.time()
    last_rx = time.time()

    with serial.Serial(RADIO_B, RADIO_BAUD, timeout=0.1) as rb:
        while True:
            buf += rb.read(512)

            while True:
                i = buf.find(MAGIC)
                if i < 0 or len(buf) - i < PKT.size:
                    if i > 0:
                        buf = buf[i:]
                    break
                chunk = buf[i:i + PKT.size]
                buf = buf[i + PKT.size:]
                try:
                    _, seq, txt, _ = PKT.unpack(chunk)
                except struct.error:
                    continue

                now = time.time()
                received += 1
                last_rx = now
                lat = (now - txt) * 1000.0
                if -1000 < lat < 10000:       # clocks are not synced; sanity only
                    lat_sum += lat
                    lat_n += 1

                if first_seq is None:
                    first_seq = seq
                elif last_seq is not None and seq > last_seq + 1:
                    gap = seq - last_seq - 1
                    lost += gap
                    gap_max = max(gap_max, gap)
                last_seq = seq

            if time.time() - last_report >= 2.0:
                last_report = time.time()
                expected = (last_seq - first_seq + 1) if first_seq is not None else 0
                pct = (100.0 * lost / expected) if expected else 0.0
                lat = (lat_sum / lat_n) if lat_n else 0.0
                silence = time.time() - last_rx
                if silence > 3:
                    link = f"!! NO PACKETS FOR {silence:.0f} s"
                elif pct < 1:
                    link = "solid"
                elif pct < 10:
                    link = "degrading"
                else:
                    link = "BAD"
                print(f"{received:>6} {lost:>6} {pct:>6.1f}% {gap_max:>8} "
                      f"{lat:>10.0f}   {link}")


def cmd_rover(args):
    """Radio B -> LG580P. Report fix quality and correction age."""
    print("radio B -> LG580P\n")
    gga = None
    rx = 0
    last = time.time()
    with serial.Serial(RADIO_B, RADIO_BAUD, timeout=0.05) as rb, \
         serial.Serial(LG580P, LG580P_BAUD, timeout=0.05) as lg:
        buf = b""
        while True:
            air = rb.read(1024)
            if air:
                lg.write(air)
                rx += len(air)
            buf += lg.read(1024)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", "ignore").strip()
                if s.startswith("$GNGGA"):
                    f = s.split(",")
                    if len(f) > 13:
                        gga = (f[6], f[7], f[8], f[13])
            if time.time() - last >= 2.0:
                last = time.time()
                if gga:
                    q, nsat, hdop, age = gga
                    label = {"0": "NO FIX", "1": "standalone", "2": "DGPS/SBAS",
                             "4": "RTK FIXED", "5": "RTK FLOAT"}.get(q, f"q={q}")
                    warn = ""
                    try:
                        if age and float(age) > 3.0:
                            warn = "   <-- corrections stale, link struggling"
                    except ValueError:
                        pass
                    print(f"  {label:<12} sats {nsat:>2}  hdop {hdop:>5}  "
                          f"age {age or '-':>5}  rtcm rx {rx:>8}{warn}")
                else:
                    print(f"  no GGA yet  (rtcm rx {rx})")


COMMANDS = {"send": cmd_send, "recv": cmd_recv, "rover": cmd_rover}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=COMMANDS)
    p.add_argument("--rate", type=float, default=10.0,
                   help="send: packets per second (default 10)")
    args = p.parse_args()
    try:
        return COMMANDS[args.command](args)
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
