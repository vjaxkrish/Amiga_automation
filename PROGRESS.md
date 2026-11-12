# Progress

Status as of **12 Nov 2026**. Platform: farm-ng Amiga, College Station,
secondary machine with a secondary Pi 5.

**Where it stands:** the Amiga drives itself to RTK waypoints and stops on
them. GPS error handling was reworked on 11 Nov (averaged teach marks, hold
on fix loss, per-plot heading, one shared antenna geometry) and passes 21
tests in simulation. The first teach-and-repeat serpentine with a tape-
measured ground truth is under way in College Station. Nothing has yet been
checked against a tape - every accuracy figure so far is the GPS measuring
itself.

## Bring-up order

| # | Step | Status |
|---|---|---|
| 1 | **Listen on CAN.** Decode AmigaTpdo1, confirm wiring, bitrate, termination, firmware | **DONE 21 Oct** |
| 2 | **Command motion on stands.** RPDO1 at 20 Hz, auto-active, small speed, deliberate kill | **DONE 22 Oct** |
| 3 | **Teleop from the Pi.** Exercise the angular rate field | **DONE 27 Oct** - `amiga_teleop.py`, used for the breakaway measurements |
| 4 | **GPS pair.** F9P base, RTCM3 MSM4 over the radio, LG580P to RTK FIXED with heading | **DONE 27 Oct** - over USB and radio |
| 5 | **Log while driving manually.** RTK, heading, wheel speed, one clock | **DONE 27 Oct** - `rover_log.py` |
| 6 | **Close the loop.** Point-to-point, then a multi-waypoint mission | **DONE 27 Oct** - `goto` and a 4-waypoint `mission` |
| 7 | **GPS error handling.** Shared geometry, averaged marks, hold on fix loss, per-plot heading | **DONE 11 Nov in simulation** - not yet driven |
| 8 | **Teach-and-repeat serpentine with ground truth.** 2 rows, headland turn, tape at every plot | **first run 12 Nov**; retuned, re-run pending |
| 9 | **Capture integration.** Camera trigger in the `mission` hook, flag and continue | not started |
| 10 | **Dallas.** Fixed base survey, re-measure constants, reproduce | not started |

## What is proven on the wire or on the ground

Everything in this list was observed, not inferred.

- **CAN bus map** - COB-IDs, bitrate, node numbering. `docs/can-bus-map.md`
- **AmigaRpdo1 @ 0x20E, bytes 0-4.** State byte by the 4 -> 5 transition;
  linear speed by commanding 50 mm/s and reading it back from 0x18E within
  +/-5 mm/s; **angular rate: positive mrad/s turns left** (27 Oct)
- **Control state enum**, all values except BOOT
- **Auto mode arming path.** The Dashboard screen tab is the gate; the start
  button is not. `docs/auto-mode-arming.md`
- **AUTO_ACTIVE held 274 s with no drift**
- **20 Hz TX timing on a Pi 5**: mean 50.0 ms, max 52.0 over 252 frames
- **Watchdog at 552 ms**, kill-to-stop at **902 ms**, active braking with a
  negative overshoot. `docs/watchdog-and-estop.md`
- **E-stop takes the whole bus down**; ENOBUFS after ten frames is the symptom
- **Dashboard firmware v0.8.0**
- **On turf:** nothing moves below **40 mm/s** linear or about **200 mrad/s**
  angular. Over 1340 mm/s observed on the ground
- **Three spins in place took the motors to 220 F**; hours of normal driving
  did not. Do not pivot in the plots
- **RTK:** LG580P takes the F9P's RTCM3 cleanly. Fixed in ~50 s over USB,
  ~6 min over the radio. Heading from `$PQTMTAR` (`$GNTHS` is empty on this
  firmware); receiver baseline 1.3315 m against 1.346 m on the tape
- **Antenna offset 0.794 / 0.813 m** (`calib-offset`), heading offset 180 deg
  on this mast. Both now live in `vehicle.py`. `docs/calibration.md`
- **No wheel position on the bus.** The motor controllers broadcast and store
  per-wheel velocity only, with no position register (`can_sdo.py`, Nov).
  Position comes from RTK
- **Turning in place on turf walks the machine backward ~0.28 m per
  radian** (12 Nov: mission turns and a pendant pivot test). The antenna
  offset is not the cause. Avoid final turns at plots
- **On turf the wheels follow turn commands but only ~10-25% of it becomes
  rotation at ~300 mm/s**, about the same left and right, jerky; ~1 s ramp
  in and out (12 Nov yaw-tests, wheel rate from 0x18E vs GPS)
- **GPS hold and resume works in the field** (12 Nov: corrections 5.1 s old
  at the far headland, held 2.4 s, carried on)
- **Survey-in:** 5 min converges to about 1 m absolute (1.241 m on 11 Nov).
  Irrelevant within a session, a frame shift between sessions

## Measured against GPS only - not yet against a tape

- **`goto` stopping error 1 - 1.5 cm typical, 3 cm worst of 15**, heading
  1.5 - 2 deg (27 Oct). This is the controller's own estimate. On 27 Oct
  `amiga_nav.py` used an antenna offset of 0.65 m against a measured 0.80, so
  the true rotation centre was about 15 cm from where it believed along the
  heading. The repeatability figure may stand; the absolute one needs the tape.
- **Teach-and-repeat before 11 Nov was off by about 1.44 m.** `rover_log.py`
  applied the antenna offset along the raw heading, which points backwards,
  so marks sat 1.6 m ahead of the vehicle and replays stopped past them.
  Reproduced from the old code in simulation, fixed 11 Nov; old teach files
  are repaired on load. Never caught on the ground because no plot was
  tape-checked.

## Written but unproven

- **11 Nov GPS handling** (`vehicle.py`, averaged marks, GPS hold and resume,
  per-plot heading, `plan`, `rover_log.py` through `rtcm_forward`): 21 tests
  pass on the laptop and the rover Pi, including a simulated serpentine with
  an 8 s dropout. Not yet driven. `CHANGES-2026-11-11.md`
- **`follow`** has never driven, and `AlignGate` (used by `follow` and
  `replay`) turns the wrong way by the verified sign. Fix before use.
- **`devices.py` auto-detection** only exercised against the College Station
  device set.

## Explicitly untested - do not assume

- **SiK radio range across a real plot.** Only tested within 10 m.
  `sik_range.py send` / `recv`
- **Whether an RPDO1 gap under 552 ms recovers** without a hand re-arm
- **RPDO1 bytes 5-7**, PTO and H-bridge. Sent as zero
- **Behaviour on a slope**
- **CANopen nodes 0x10 / 0x11** and **0x23A** - still unexplained,
  `docs/open-questions.md`

## Dallas

The Dallas Amiga is **believed** to be the same model on the same firmware -
believed, not verified. `docs/dallas-checklist.md` is the day-one list.

Order that matters: **survey the base with fixed coordinates before teaching
any plot** (`base_station.py survey --hours 4`, then `--fixed`). Teaching
first means teaching twice.

Re-measure there regardless of College Station: watchdog and kill-to-stop
timings, antenna and heading offsets, turf breakaway speeds, SiK range
across the plot, firmware version.

### Log of Dallas findings

Fill this in on site.

| Date | Finding |
|---|---|
| | |

## Dated log

**21 Oct 2026** - First CAN capture, read-only. Bus map built: 0x18E, 0x70E,
motor controller TPDO1s. AmigaTpdo1 decode established. Noticed the 0x10 /
0x11 NMT retry loop with no responder.

**22 Oct 2026** - Motion under Pi command. Auto mode arming path found: the
Dashboard screen tab, not the start button, and RPDO1 is ignored entirely
from MANUAL_ACTIVE. AUTO_ACTIVE reached and held, 50 mm/s commanded and
measured, watchdog verified at 552 ms. E-stop found to take the whole bus
down.

**23 Oct 2026** - Code and findings extracted into this repo. `devices.py`
resolves hardware by USB VID:PID. ENOBUFS handling in `amiga_rpdo1.py`.

**27 Oct 2026** - Steps 3-6. RTK base and rover, unattended base service,
`rtcm_forward` as sole owner of the GNSS ports, combined logger, and the
navigation controller. The Amiga drives itself to a waypoint and stops on it.
Twelve bugs found and fixed on the way, among them two inverted turn
directions, pure pursuit driving away from a target behind it, and control
on the antenna rather than the rotation centre. `CHANGES-2026-10-27.md`

**Early Nov 2026** - Wheel encoder check over CANopen SDO (`can_sdo.py`):
velocity only, no position register. Decision: RTK for the in-row advance.

**9 Nov 2026** - `RUNBOOK.md` written for day-to-day field sessions.

**11 Nov 2026** - GPS error handling (`CHANGES-2026-11-11.md`): one shared
antenna geometry in `vehicle.py`, which also fixed the 1.44 m teach-and-repeat
offset; 3 s averaged, fix-checked teach marks; GPS faults hold and resume
instead of ending the run; each mark squares to its taught heading;
`amiga_nav.py plan`; per-run stop-error CSV; `rover_log.py` reads through
`rtcm_forward`, so nothing is stopped to teach. 21 tests pass on the rover Pi.
Base survey-in converged to 1.241 m in 300 s. First ground-truth serpentine
test started that evening.

**12 Nov 2026** - First teach-and-repeat serpentine (2 rows, 4 plots, 2
headland points): mission completed unattended, one GPS hold at the far
headland (corrections 5.1 s old) recovered by itself. Plots 2-4 at 2.8 cm
(GPS-reported), plot_001 7.1 cm after a 9 deg final turn. Found that turning
in place on turf walks the machine backward ~0.28 m per radian (pivot test
confirms it is physical, not the antenna offset), and that the 220 mrad/s
yaw-test with the logger alongside: the wheels follow turn commands but on
turf only ~10-25% becomes rotation, either way. Controller retuned
against a simulator calibrated on that day's route: 1.5 cm deadband,
post-turn creep fix, smoothed stopping, `yaw-test` added; row-line
following built but left off (needs turning authority it lacks).
`CHANGES-2026-11-12.md`.
