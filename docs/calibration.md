# Calibration constants

Every number here was measured on the College Station Amiga on 27 Oct 2026.
They are properties of **this machine and this mast**, not of the software.
Re-measure all of them on the Dallas machine before trusting any of it.

`ANTENNA_OFFSET_M` and `HEADING_OFFSET_DEG` are defined **once**, in
`vehicle.py`, and imported by `amiga_nav.py` and `rover_log.py`. Change them
there and nowhere else.

---

## ANTENNA_OFFSET_M = 0.80

Distance from the GNSS antenna to the vehicle's centre of rotation, along
the vehicle axis.

**Why it matters.** The antenna is on a mast ahead of the rotation centre,
so a turn in place swings it through an arc of chord `2 * L * sin(theta/2)`
even though the machine has not translated. Controlling the antenna position
therefore means the machine moves whenever it turns.

Observed before the fix, during a four-waypoint mission:

| waypoint | reported on arrival | measured after the final turn | turn |
|---|---|---|---|
| plot_001 | 2.1 cm | 4.3 cm | 3.5 deg |
| plot_002 | 2.6 cm | 3.2 cm | 3.5 deg |
| plot_003 | 2.3 cm | **90.3 cm** | 75 deg |
| plot_004 | 2.0 cm | **117.4 cm** | 167 deg |

The error scales as the chord of the turn, which is the signature.

**Fix:** all control runs on the rotation centre. `vehicle.rotation_centre()`
shifts the antenna fix back along the **true** heading; `control_point()` in
`amiga_nav.py` does the same in local metres. Both use the one constant in
`vehicle.py`.

**What went wrong before 11 Nov 2026.** The constant was copied into both
files and the copies drifted (0.65 in `amiga_nav.py`, 0.794 in
`rover_log.py`), and `rover_log.py` applied it along the **raw** receiver
heading, which points backwards on this mast. Taught marks therefore sat
1.6 m in front of the true rotation centre, and a replay stopped about
**1.44 m past** the taught spot - more than one plot pitch. The controller's
own error report could not show this, because it measured against the same
wrong mark. `amiga_nav.py` now re-derives every mark from its stored antenna
fix and heading, so old teach files are repaired on load.

**Measure with** `python3 amiga_nav.py calib-offset`, which spins 180 deg and
halves the chord. Two readings gave 0.794 and 0.813 m; the spread is RTK
noise at both ends of the spin plus the stop tolerance. Average several.

---

## HEADING_OFFSET_DEG = 180.0

The LG580P reports heading 180 deg from the direction of travel with the mast
wired Antenna-1 forward, Antenna-2 rear.

Measured against course-over-ground across 1839 moving samples of a 380 s
driving log: circular mean **-177.6 deg**, circular sd 5.5 deg.

Independently confirmed: at a mark where the machine pointed straight at the
next mark 12.7 m away on a bearing of 137.5 deg, raw heading read 317.3 deg.

Applied in exactly one place, `vehicle.true_heading()`. Anything that reads
PQTMTAR goes through it.

**Set to 0.0 if the antenna cables are ever swapped.** The cheaper fix is to
swap the cables and delete the constant, but it was left as software so the
field wiring stays as built.

---

## Pivot slip: ~0.28 m backward per radian turned in place

Measured 12 Nov 2026 on turf. Every in-place turn moved the rotation centre
backward: plot_001 4.4 cm for ~9 deg, mark_5 43 cm for ~88 deg, mark_8 about
9 cm for ~39 deg; a pendant pivot test moved it 12 and 35 cm for two ~95 deg
turns. Not the antenna offset (the pivot test's turns fit +5 and -2 cm of
offset error, inconsistent). This is why the controller arrives square on
in-row legs and creeps to fix the along-row error after any final turn.

## Turning while rolling: ~10-25% of the wheel rate

Three `yaw-test`s on 12 Nov 2026 at ~300 mm/s, the last with
`rover_log.py drive` alongside to capture the wheel rate (0x18E): the wheels
reach the commanded rate in ~1 s and wind down over ~1 s, but GPS heading
turned at only ~10-25% of it, about the same left and right. Jerky
(stick-slip: one turn stalled ~2.4 s mid-way). At 600 mrad/s the inside
wheels stop and it pivots. `ANGULAR_MIN_ROLLING_MRAD_S` stays 220: smaller
commands do almost nothing.

## ANGULAR_MIN_MRAD_S = 200

Below roughly 150-200 mrad/s the machine does not rotate at all on turf.

**Consequence, and it shapes the whole controller:** the smallest available
steering correction is about 12 deg/s. Against a 3 deg error that is not a
correction, it is a lurch, and a proportional law built on it oscillates and
diverges rather than settling.

This is why:

- creep steering is **off** (`CREEP_STEER = False`)
- the final heading turn is **pulsed** rather than proportional
- any non-zero angular command is raised to this magnitude in `Rover.send()`,
  because sending less is the same as sending nothing but looks like steering

---

## CREEP_SPEED_MM_S = 40

Slowest commanded speed that reliably breaks static friction on turf with the
machine's weight on the wheels. Measured on the ground, not on stands - the
unloaded number is lower and useless.

At 40 mm/s a zero command stops the machine effectively instantly, so there
is no deceleration overshoot to compensate for at the end of an approach.

---

## Dashboard limits

| | |
|---|---|
| Linear speed ceiling | ~890 mm/s on stands, over 1340 mm/s observed on the ground |
| Angular accepted | 1000 mrad/s, ceiling not found |
| Firmware | v0.8.0 |

The speed limit is **configurable on the Dashboard GUI**. Whatever it is set
to in College Station must match Dallas, or identical code behaves
differently and it will look like a controller bug.

---

## Watchdog

| event | time |
|---|---|
| last RPDO1 frame to state reverting 5 -> 4 | **552 ms** |
| last RPDO1 frame to measured speed zero | **902 ms** |

Slower than the "below roughly 5 Hz" doc line implies, which would suggest
~200 ms. Use the measured figure.

At 1 m/s that is **~0.9 m of travel after the commanding process dies**. That
is the number for the safety case and it sets the minimum standoff for
anything placed in front of the machine.

Two details:

- The stop is **active braking, not a coast**: 54 -> 48 -> 44 -> 32 -> **-20**
  -> 0 in 350 ms. The negative overshoot is the Dashboard's own deceleration
  limit.
- **State reverts ~550 ms before motion stops.** Anything treating
  `state != 5` as "stationary" will be wrong. Gate on measured speed.

**Recovery is not automatic.** After a watchdog trip the Dashboard sits at
AUTO_READY and will not return to AUTO_ACTIVE on request - the auto tab must
be armed by hand. That is correct as a safety interlock and it means **any
transient strands an unattended machine**, which is the hard reliability
requirement for Dallas.

---

## Sign conventions, all verified on the wire

| | |
|---|---|
| RPDO1 bytes 3-4, positive mrad/s | turns **left** (counter-clockwise from above) |
| Bearing error positive | target is to the **right**, so command a negative rate |
| Heading | bearing from north, increasing clockwise |

The second line cost two debugging cycles. Getting it backwards makes the
error grow instead of shrink: observed 13 deg -> 77 deg over 12 s of turning
the wrong way, and separately 105 s stuck oscillating at +/-179 deg.
