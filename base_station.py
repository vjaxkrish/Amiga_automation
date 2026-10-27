#!/usr/bin/env python3
"""
base_station.py - unattended RTK base. Pi 3 + F9P + SiK radio + power bank.

Configures the F9P on startup, runs survey-in (or loads fixed coordinates),
then pipes RTCM3 into the radio continuously and logs status to disk.

    python3 base_station.py run                 survey-in each boot
    python3 base_station.py run --fixed         use coordinates from the config file
    python3 base_station.py survey --hours 4    long survey, writes base_position.json
    python3 base_station.py check               one-shot health report, no radio

Designed to be started by systemd on boot with nobody logged in. Exits
non-zero on unrecoverable errors so systemd restarts it.

Why log: when a rover session produces bad data, the first question is whether
the base was healthy. Without a log you cannot answer it.
"""

import argparse
import json
import os
import signal
import sys
import time
from datetime import datetime, timezone

import serial
from pyubx2 import UBXMessage, UBXReader

# --- device resolution ------------------------------------------------------
# by-id paths are stable across reboots and USB port changes. The F9P pattern
# matches any u-blox receiver; the radio is matched by its FTDI serial so the
# two SiK units never get confused.

F9P_GLOB = "usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
RADIO_SERIAL = "DU0E44NK"          # radio A, base side

BYID = "/dev/serial/by-id"
F9P_BAUD = 38400                   # ignored over USB CDC
RADIO_BAUD = 57600

CONFIG_DIR = os.path.expanduser("~/base")
POSITION_FILE = os.path.join(CONFIG_DIR, "base_position.json")
LOG_DIR = os.path.join(CONFIG_DIR, "logs")

# --- survey-in --------------------------------------------------------------

SVIN_MIN_DUR_S = 300               # 5 min on boot; --hours overrides for a survey
SVIN_ACC_LIMIT_MM = 2000

LAYER_RAM = 1
LAYER_BBR = 2
LAYER_FLASH = 4

RTCM_MSGS = ["1005", "1074", "1084", "1094", "1124", "1230"]

STOP = False


def _sigterm(signum, frame):
    global STOP
    STOP = True


def find_port(fragment):
    try:
        for name in os.listdir(BYID):
            if fragment in name:
                return os.path.join(BYID, name)
    except FileNotFoundError:
        pass
    return None


def log(msg, fh=None):
    line = f"{datetime.now(timezone.utc).isoformat(timespec='seconds')}  {msg}"
    print(line, flush=True)
    if fh:
        fh.write(line + "\n")
        fh.flush()


def open_log():
    os.makedirs(LOG_DIR, exist_ok=True)
    name = datetime.now(timezone.utc).strftime("base_%Y%m%d_%H%M%S.log")
    return open(os.path.join(LOG_DIR, name), "a")


# --- F9P configuration ------------------------------------------------------

def configure(ser, mode, fixed=None, min_dur=SVIN_MIN_DUR_S, fh=None):
    """mode: 'svin' or 'fixed'. fixed is a dict from POSITION_FILE."""
    cfg = []

    if mode == "svin":
        cfg.append(("CFG_TMODE_MODE", 1))
        cfg.append(("CFG_TMODE_SVIN_MIN_DUR", min_dur))
        cfg.append(("CFG_TMODE_SVIN_ACC_LIMIT", SVIN_ACC_LIMIT_MM * 10))
    else:
        # fixed mode, ECEF in cm plus a high-precision remainder in 0.1 mm
        cfg.append(("CFG_TMODE_MODE", 2))
        cfg.append(("CFG_TMODE_POS_TYPE", 0))          # 0 = ECEF
        cfg.append(("CFG_TMODE_ECEF_X", fixed["ecefX"]))
        cfg.append(("CFG_TMODE_ECEF_Y", fixed["ecefY"]))
        cfg.append(("CFG_TMODE_ECEF_Z", fixed["ecefZ"]))
        cfg.append(("CFG_TMODE_ECEF_X_HP", fixed["ecefXHP"]))
        cfg.append(("CFG_TMODE_ECEF_Y_HP", fixed["ecefYHP"]))
        cfg.append(("CFG_TMODE_ECEF_Z_HP", fixed["ecefZHP"]))
        cfg.append(("CFG_TMODE_FIXED_POS_ACC", int(fixed.get("accMM", 100)) * 10))

    for m in RTCM_MSGS:
        cfg.append((f"CFG_MSGOUT_RTCM_3X_TYPE{m}_USB", 1))

    cfg.append(("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1))
    cfg.append(("CFG_MSGOUT_UBX_NAV_SAT_USB", 0))      # noisy, keep off
    cfg.append(("CFG_MSGOUT_UBX_NAV_PVT_USB", 1))      # satellite count, fix type

    cfg.append(("CFG_USBOUTPROT_NMEA", 0))
    cfg.append(("CFG_USBOUTPROT_UBX", 1))
    cfg.append(("CFG_USBOUTPROT_RTCM3X", 1))

    msg = UBXMessage.config_set(LAYER_RAM, 0, cfg)
    ser.write(msg.serialize())

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
            log(f"F9P configured: {mode} mode, {len(cfg)} keys", fh)
            return True
        if parsed.identity == "ACK-NAK":
            log("F9P REJECTED configuration (ACK-NAK)", fh)
            return False
    log("F9P config sent, no ACK within 3 s", fh)
    return True


# --- commands ---------------------------------------------------------------

def cmd_run(args):
    signal.signal(signal.SIGTERM, _sigterm)
    fh = open_log()
    log("base station starting", fh)

    f9p_port = find_port(F9P_GLOB)
    radio_port = find_port(RADIO_SERIAL)
    if not f9p_port:
        log("FATAL: F9P not found on USB", fh)
        return 1
    if not radio_port:
        log(f"FATAL: radio {RADIO_SERIAL} not found on USB", fh)
        return 1
    log(f"F9P   {f9p_port}", fh)
    log(f"radio {radio_port}", fh)

    fixed = None
    if args.fixed:
        try:
            with open(POSITION_FILE) as f:
                fixed = json.load(f)
            log(f"using fixed position from {POSITION_FILE}", fh)
        except Exception as e:
            log(f"FATAL: --fixed but cannot read {POSITION_FILE}: {e}", fh)
            return 1

    with serial.Serial(f9p_port, F9P_BAUD, timeout=0.1) as f9p, \
         serial.Serial(radio_port, RADIO_BAUD, timeout=0.1) as radio:

        if not configure(f9p, "fixed" if fixed else "svin", fixed, fh=fh):
            return 1

        ubr = UBXReader(f9p, protfilter=6)
        rtcm_bytes = 0
        rtcm_msgs = 0
        nsat = 0
        svin_valid = bool(fixed)
        svin_acc = 0.0
        svin_dur = 0
        last_log = time.time()
        announced = bool(fixed)

        while not STOP:
            try:
                raw, parsed = ubr.read()
            except Exception:
                continue
            if parsed is None:
                continue

            ident = str(parsed.identity)

            if ident.isdigit():                 # RTCM3
                radio.write(raw)
                rtcm_bytes += len(raw)
                rtcm_msgs += 1
            elif ident == "NAV-SVIN":
                svin_dur = parsed.dur
                svin_acc = parsed.meanAcc / 10000.0
                if parsed.valid and not announced:
                    log(f"survey-in complete: {svin_dur} s, {svin_acc:.3f} m", fh)
                    announced = True
                svin_valid = bool(parsed.valid)
            elif ident == "NAV-PVT":
                nsat = parsed.numSV

            if time.time() - last_log >= args.interval:
                last_log = time.time()
                state = "FIXED" if svin_valid else f"svin {svin_dur}s"
                log(f"sats {nsat:>2}  {state:<14} acc {svin_acc:6.3f} m  "
                    f"rtcm {rtcm_msgs:>7} msgs {rtcm_bytes:>9} B", fh)

    log("base station stopping", fh)
    return 0


def cmd_survey(args):
    """Long survey to establish a permanent base position."""
    fh = open_log()
    dur = int(args.hours * 3600)
    log(f"long survey: {args.hours} h minimum, target {args.acc} m", fh)

    f9p_port = find_port(F9P_GLOB)
    if not f9p_port:
        log("FATAL: F9P not found", fh)
        return 1

    global SVIN_ACC_LIMIT_MM
    SVIN_ACC_LIMIT_MM = int(args.acc * 1000)

    with serial.Serial(f9p_port, F9P_BAUD, timeout=0.1) as f9p:
        if not configure(f9p, "svin", min_dur=dur, fh=fh):
            return 1
        ubr = UBXReader(f9p, protfilter=2)
        last = time.time()
        while True:
            try:
                _, parsed = ubr.read()
            except Exception:
                continue
            if parsed is None or parsed.identity != "NAV-SVIN":
                continue
            acc = parsed.meanAcc / 10000.0
            if time.time() - last >= 30:
                last = time.time()
                log(f"  {parsed.dur:>6} s  obs {parsed.obs:>7}  "
                    f"acc {acc:7.3f} m", fh)
            if parsed.valid:
                pos = {
                    "ecefX": parsed.meanX, "ecefY": parsed.meanY,
                    "ecefZ": parsed.meanZ,
                    "ecefXHP": parsed.meanXHP, "ecefYHP": parsed.meanYHP,
                    "ecefZHP": parsed.meanZHP,
                    "accMM": int(acc * 1000),
                    "surveyed": datetime.now(timezone.utc).isoformat(),
                    "duration_s": parsed.dur, "observations": parsed.obs,
                    "site": args.site,
                }
                os.makedirs(CONFIG_DIR, exist_ok=True)
                with open(POSITION_FILE, "w") as f:
                    json.dump(pos, f, indent=2)
                log(f"survey complete, {acc:.3f} m -> {POSITION_FILE}", fh)
                return 0


def cmd_check(args):
    """One-shot health report. Does not touch the radio."""
    f9p_port = find_port(F9P_GLOB)
    radio_port = find_port(RADIO_SERIAL)
    print(f"F9P    {f9p_port or 'NOT FOUND'}")
    print(f"radio  {radio_port or 'NOT FOUND'}")
    print(f"config {POSITION_FILE} "
          f"{'present' if os.path.exists(POSITION_FILE) else 'absent'}")
    if not f9p_port:
        return 1
    with serial.Serial(f9p_port, F9P_BAUD, timeout=0.1) as f9p:
        ubr = UBXReader(f9p, protfilter=6)
        seen = {}
        deadline = time.time() + 5
        while time.time() < deadline:
            try:
                _, parsed = ubr.read()
            except Exception:
                continue
            if parsed is None:
                continue
            i = str(parsed.identity)
            seen[i] = seen.get(i, 0) + 1
        print("\n5 s of F9P output:")
        for k, v in sorted(seen.items()):
            print(f"  {k:<12} {v}")
    return 0


COMMANDS = {"run": cmd_run, "survey": cmd_survey, "check": cmd_check}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=COMMANDS)
    p.add_argument("--fixed", action="store_true",
                   help="run: use coordinates from base_position.json")
    p.add_argument("--interval", type=float, default=30.0,
                   help="run: seconds between status log lines (default 30)")
    p.add_argument("--hours", type=float, default=4.0,
                   help="survey: minimum duration in hours (default 4)")
    p.add_argument("--acc", type=float, default=0.5,
                   help="survey: target accuracy in metres (default 0.5)")
    p.add_argument("--site", default="unnamed",
                   help="survey: site label stored with the position")
    args = p.parse_args()
    try:
        return COMMANDS[args.command](args)
    except KeyboardInterrupt:
        print("\nstopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
