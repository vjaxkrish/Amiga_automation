# Progress

Status as of **23 Oct 2026**. Platform: farm-ng Amiga, College Station,
secondary machine with a secondary Pi 5.

## Bring-up order

Prove each interface alone before combining. Steps 1-3 need no GPS; step 4
needs no robot. The two tracks run in parallel.

| # | Step | Status |
|---|---|---|
| 1 | **Listen on CAN.** Decode AmigaTpdo1, confirm wiring, bitrate, termination, firmware | **DONE 21 Oct 2026** |
| 2 | **Command motion on stands.** RPDO1 at 20 Hz, auto-active, small speed, deliberate kill | **DONE 22 Oct 2026** |
| 3 | **Teleop from the Pi.** Gamepad or keyboard, on the ground. **Exercise the angular rate field here** | not started |
| 4 | **GPS pair on the bench.** F9P as base, RTCM3 MSM4 into the radio, LG580P to RTK FIX with heading | scripts written, **not yet run against hardware** |
| 5 | **Log while driving manually.** RTK, heading, wheel odometry, on a rigid known baseline | not started |
| 6 | **Close the loop.** Pure pursuit, straight line first, headland turns after | not started |

## What is proven on the wire

Everything in this list was observed, not inferred.

- **CAN bus map** — COB-IDs, bitrate, node numbering. `docs/can-bus-map.md`
- **AmigaRpdo1 @ 0x20E, byte 0 and bytes 1-2.** State byte confirmed by the
  4 → 5 transition; linear speed confirmed by commanding 50 mm/s and reading
  50 mm/s back from 0x18E, holding within ±5 mm/s over 2 s
- **Control state enum, all values except BOOT**
- **Auto mode arming path.** The Dashboard screen tab is the gate; the start
  button is not. `docs/auto-mode-arming.md`
- **AUTO_ACTIVE held 274 s with no drift**
- **20 Hz TX timing on a Pi 5** under a desktop session, no special
  scheduling: mean 50.0 ms, median 50.0, max 52.0 over 252 frames
- **Watchdog at 552 ms**, kill-to-stop at **902 ms**, with active braking and
  a negative overshoot. `docs/watchdog-and-estop.md`
- **E-stop takes the whole bus down**, and ENOBUFS after ten frames is the
  correct symptom
- **int16 speed decode at 893 mm/s** forward and -379 mm/s reverse, no
  sign-extension or rollover artefacts
- **SOC byte is an estimator**, confirmed twice. Do not gate on it

## What is written but unproven

- **The four GPS scripts have never been run against the hardware.**
  `rtk_bench.py`, `rtk_radio.py`, `sik_range.py`, `lg580p_cmd.py`. They are
  the plan for step 4, not a result.
- **`devices.py` auto-detection** has only been exercised against the
  College Station device set.

## Explicitly untested — do not assume

- **RPDO1 bytes 3-4, angular rate.** Sent as zero throughout. **No turn has
  ever been commanded.** This is the biggest single gap in the CAN work
- **RPDO1 bytes 5-7**, PTO and H-bridge. Sent as zero
- **Speeds above 100 mm/s under our command.** `amiga_rpdo1.py` clamps at
  ±100 mm/s deliberately. The pendant has driven ~893 mm/s; we have not
- **Any behaviour under load**, on a slope, or on real ground. Everything so
  far is on stands
- **The Dashboard firmware version string** has never been read off the
  screen. The 8-byte payload was inferred from the wire, and it is right, but
  the version itself is unrecorded

## Dallas

The deployment target is the turf phenotyping work. The Dallas Amiga is
**believed** to be the same model on the same firmware — believed, not
verified. `SETUP.md` step 3 exists to verify it, and it should be the first
thing done on arrival.

Three numbers to re-measure there regardless of what College Station said:

1. **The watchdog and kill-to-stop timings.** Safety case, per machine.
2. **SiK range across the plot.** A property of that terrain, not of the radios.
3. **The firmware version**, read off the screen.

### Log of Dallas findings

Fill this in on site.

| Date | Finding |
|---|---|
| | |

## Dated log

**21 Oct 2026** — First CAN capture, read-only. Bus map built: 0x18E, 0x70E,
motor controller TPDO1s. AmigaTpdo1 decode established. Noticed the 0x10 /
0x11 NMT retry loop with no responder.

**22 Oct 2026** — Motion under Pi command. Auto mode arming path found after
most of a morning: the Dashboard screen tab, not the start button, and RPDO1
is ignored entirely from MANUAL_ACTIVE. Phases A, B and C all complete —
AUTO_ACTIVE reached and held, 50 mm/s commanded and measured, watchdog
verified at 552 ms. Four new COB-IDs seen (0x20F, 0x28E, 0x38E, 0x23A). E-stop
found to take the whole bus down. 0x10 / 0x11 loop reproduced in a second
independent capture.

**23 Oct 2026** — Code and findings extracted into this repo. `devices.py`
added so hardware paths resolve by USB VID:PID rather than by hardcoded serial
number. ENOBUFS handling added to `amiga_rpdo1.py` per the 22 Oct e-stop
finding. GPS scripts written against the planned bench setup, not yet run.
