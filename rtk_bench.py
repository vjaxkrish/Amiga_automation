#!/usr/bin/env python3
"""
rtk_bench.py - F9P base + LG580P rover, both on USB, no radio.

Phase 0, step 4, stage 1. Answers open question 4: does the LG580P ingest
F9P RTCM3 MSM4 cleanly?

    config     put the F9P into survey-in base mode, RTCM3 out on USB
    svin       watch survey-in converge (UBX-NAV-SVIN)
    bridge     pipe RTCM3 from the F9P into the LG580P, report both ends
    status     read GGA from the LG580P only, no bridging

Config is written to RAM only, so a power cycle reverts the F9P to normal.
Deliberate: nothing permanent until the setup is proven.
"""

import argparse
import sys
import time

import serial
from pyubx2 import UBXMessage, UBXReader

# --- device paths -----------------------------------------------------------

import devices

F9P_PORT = devices.F9P_PORT
LG580P_PORT = devices.LG580P_PORT

F9P_BAUD = devices.F9P_BAUD          # ignored over USB CDC, pyserial wants a number
LG580P_BAUD = devices.LG580P_BAUD

# --- survey-in parameters ---------------------------------------------------

SVIN_MIN_DUR_S = 60          # minimum seconds before survey-in can finish
SVIN_ACC_LIMIT_MM = 2000     # 2 m target; tighten for a permanent base

LAYER_RAM = 1
LAYER_BBR = 2
LAYER_FLASH = 4

# RTCM3 messages for an MSM4 base.
#   1005  station reference position (ARP)
#   1074  GPS MSM4
#   1084  GLONASS MSM4
#   1094  Galileo MSM4
#   1124  BeiDou MSM4
#   1230  GLONASS code-phase biases
RTCM_MSGS = ["1005", "1074", "1084", "1094", "1124", "1230"]


def cmd_config(args):
    cfg = []

    # survey-in base mode
    cfg.append(("CFG_TMODE_MODE", 1))
    cfg.append(("CFG_TMODE_SVIN_MIN_DUR", SVIN_MIN_DUR_S))
    cfg.append(("CFG_TMODE_SVIN_ACC_LIMIT", SVIN_ACC_LIMIT_MM * 10))  # 0.1 mm units

    # RTCM3 out on USB at 1 Hz, except 1005/1230 which are slower by convention
    for msg in RTCM_MSGS:
        rate = 1 if msg not in ("1005", "1230") else 1
        cfg.append((f"CFG_MSGOUT_RTCM_3X_TYPE{msg}_USB", rate))

    # survey-in status so we can watch convergence
    cfg.append(("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1))

    # protocols on USB: UBX in/out, RTCM3 out, NMEA off so the stream is clean
    cfg.append(("CFG_USBOUTPROT_NMEA", 0))
    cfg.append(("CFG_USBOUTPROT_UBX", 1))
    cfg.append(("CFG_USBOUTPROT_RTCM3X", 1))

    layers = LAYER_RAM
    if args.persist:
        layers = LAYER_RAM + LAYER_BBR + LAYER_FLASH
        print("writing to RAM + BBR + FLASH (persists across power cycles)")
    else:
        print("writing to RAM only (reverts on power cycle)")

    msg = UBXMessage.config_set(layers, 0, cfg)

    with serial.Serial(F9P_PORT, F9P_BAUD, timeout=2) as ser:
        ser.write(msg.serialize())
        print(f"sent VALSET with {len(cfg)} keys, waiting for ACK ...")

        ubr = UBXReader(ser, protfilter=2)
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                _, parsed = ubr.read()
            except Exception:
                continue
            if parsed is None:
                continue
            if parsed.identity == "ACK-ACK":
                print("ACK-ACK - configuration accepted")
                return 0
            if parsed.identity == "ACK-NAK":
                print("ACK-NAK - REJECTED. Check firmware supports these keys.")
                return 1
        print("no ACK seen within 3 s (may still have applied; run 'svin')")
    return 0


def cmd_svin(args):
    print(f"watching survey-in: need {SVIN_MIN_DUR_S} s and "
          f"{SVIN_ACC_LIMIT_MM/1000:.1f} m accuracy\n")
    with serial.Serial(F9P_PORT, F9P_BAUD, timeout=2) as ser:
        ubr = UBXReader(ser, protfilter=2)
        while True:
            try:
                _, parsed = ubr.read()
            except Exception:
                continue
            if parsed is None or parsed.identity != "NAV-SVIN":
                continue
            acc = parsed.meanAcc / 10000.0   # 0.1 mm -> m
            print(f"  dur {parsed.dur:5d} s   obs {parsed.obs:6d}   "
                  f"meanAcc {acc:7.3f} m   valid {parsed.valid}   "
                  f"active {parsed.active}")
            if parsed.valid:
                print("\nSURVEY-IN COMPLETE. Base position fixed. "
                      "Run 'bridge' next.")
                return 0


def cmd_bridge(args):
    print("bridging RTCM3: F9P -> LG580P\n")
    counts = {}
    last_report = time.time()
    gga_fix = None

    with serial.Serial(F9P_PORT, F9P_BAUD, timeout=0.1) as f9p, \
         serial.Serial(LG580P_PORT, LG580P_BAUD, timeout=0.1) as lg:

        ubr = UBXReader(f9p, protfilter=6)   # UBX + RTCM3
        lg_buf = b""

        while True:
            # --- F9P -> LG580P
            try:
                raw, parsed = ubr.read()
            except Exception:
                raw, parsed = None, None

            if raw and parsed is not None:
                ident = str(parsed.identity)
                if ident.isdigit() or ident.startswith("1"):
                    lg.write(raw)
                    counts[ident] = counts.get(ident, 0) + 1

            # --- LG580P status
            lg_buf += lg.read(512)
            while b"\n" in lg_buf:
                line, lg_buf = lg_buf.split(b"\n", 1)
                s = line.decode("ascii", "ignore").strip()
                if s.startswith("$GNGGA") or s.startswith("$GPGGA"):
                    f = s.split(",")
                    if len(f) > 8:
                        gga_fix = (f[6], f[7], f[8], f[13] if len(f) > 13 else "")

            if time.time() - last_report >= 2.0:
                last_report = time.time()
                sent = " ".join(f"{k}:{v}" for k, v in sorted(counts.items()))
                if gga_fix:
                    q, nsat, hdop, age = gga_fix
                    label = {"0": "NO FIX", "1": "standalone", "2": "DGPS/SBAS",
                             "4": "RTK FIXED", "5": "RTK FLOAT"}.get(q, f"q={q}")
                    print(f"  rover: {label:<12} sats {nsat:>2}  hdop {hdop:>5}  "
                          f"corr age {age or '-':>5}   | rtcm sent  {sent}")
                else:
                    print(f"  rover: no GGA yet   | rtcm sent  {sent}")


def cmd_status(args):
    with serial.Serial(LG580P_PORT, LG580P_BAUD, timeout=1) as lg:
        buf = b""
        while True:
            buf += lg.read(512)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", "ignore").strip()
                if s.startswith("$GNGGA"):
                    print(s)


COMMANDS = {"config": cmd_config, "svin": cmd_svin,
            "bridge": cmd_bridge, "status": cmd_status}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=COMMANDS)
    p.add_argument("--persist", action="store_true",
                   help="config: also write to BBR and FLASH")
    args = p.parse_args()
    try:
        return COMMANDS[args.command](args)
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
