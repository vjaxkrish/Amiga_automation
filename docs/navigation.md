# Navigation controller

`amiga_nav.py`. Four commands:

| | |
|---|---|
| `replay <csv>` | simulate against a logged path. No hardware, no robot. |
| `follow <csv>` | drive the path recorded in that log |
| `goto <lat> <lon>` | drive to one point and stop |
| `mission <json>` | drive the marks from a `rover_log.py teach` summary |
| `calib-offset` | spin 180 deg and measure the antenna offset |

---

## Run `replay` before anything drives

It starts a simulated vehicle offset from the path and reports whether the
controller converges. It is not a formality: the very first run caught a bug
that would have driven the machine off a field.

Pure pursuit computes zero curvature when the target sits directly behind the
vehicle, so at 180 deg of bearing error it steers nothing and drives straight
away forever. The recorded path had back-and-forth passes, so it hit that
case immediately. Fixed with a turn-in-place branch plus hysteresis.

Current convergence against a real 216 m hand-driven log:

| start offset | settled mean cross-track |
|---|---|
| 0.5 m | 0.17 m |
| 1.5 m | 0.17 m |
| 3.0 m | 0.17 m |

17 cm is mediocre, but that path is full of tight hand-driven turns. On
straight rows it is far better. `LOOKAHEAD_M` is the tuning knob.

---

## The approach, in four phases

**align** - turn in place until the target is within 8 deg of the nose.
Direction is **latched** when aligning starts. Near 180 deg the sign of the
bearing error flips on noise alone, so choosing direction fresh each tick
makes the machine rock either side of the antipode forever. Observed: 105 s
stuck at +/-179 deg.

**drive** - pure pursuit at cruise speed until 1.2 m out.

**final** - the approach line is locked at 1.2 m and the machine drives it.
Locking is necessary because the bearing to a nearby target is dominated by
RTK noise: at 0.4 m, 2 cm of jitter swings it 2.9 deg, and pure pursuit then
chases noise exactly where it should be settling. Observed: heading diverged
from -0.6 to +10.1 deg over the last 70 cm.

Lateral correction stays on during this phase, with a 3 cm deadband. Locking
the line at 2 m and driving blind was tried first and gave 8 cm of
cross-track drift - the machine crabs, because skid steer on turf tracks a
slow curve on equal commands.

Speed **ramps** across this phase rather than stepping at the creep boundary.
The Dashboard owns deceleration and it is not instant: a 500 -> 40 mm/s step
took about 0.5 s and 20 cm, so the machine arrived inside the creep zone
still doing 490 mm/s and only the last few centimetres were actually slow.

**creep** - last 30 cm, 40 mm/s, dead straight, no steering at all. Any
correction here would be a 12 deg/s lurch against a 1 cm error. Over 30 cm,
5 deg of uncorrected heading error puts the machine 2.6 cm to one side, which
is inside RTK noise, so there is nothing worth correcting.

**turn** - square up to the final heading. Continuous rotation above 8 deg of
error, then **pulses**: rotate briefly, stop, settle 350 ms, re-measure,
repeat. Pulsing is the only way to get fine angular resolution on a machine
that will not rotate below ~12 deg/s.

---

## Arrival is judged on along-track, not radial distance

Radial distance can never reach zero through RTK noise, so a radial threshold
either stops early or never fires. The along-track remainder - how far short
of the target plane the machine still is, measured along the locked approach
direction - crosses zero cleanly.

Cross-track is reported separately. It is the error that cannot be fixed by
driving further and is the honest measure of lateral accuracy.

A **settle gate** holds arrival until creep speed has been held 0.6 s, so the
machine does not declare success mid-deceleration while still carrying
momentum.

---

## Measured performance

Over 15+ `goto` runs from varied approach angles:

| | |
|---|---|
| Position error, typical | **1 - 1.5 cm** |
| Position error, worst of 15 | 3 cm |
| Final heading error | **1.5 - 2 deg** |
| Cross-track during approach | under 3 cm |

That is at RTK noise. Doing better needs the camera closing the loop, not a
better controller.

---

## Safety gates

The controller stops and says why if any of these go wrong:

- position, heading or CAN data older than 0.5 s
- fix quality is not 4 (RTK fixed)
- corrections older than 5 s
- Dashboard state is not AUTO_READY or AUTO_ACTIVE
- heading drifts more than 25 deg off the locked approach line
- the machine gets more than 50 cm past where the approach line was locked
- aligning for 12 s without reducing the error
- `mission` refuses a first waypoint more than 20 m away

The last four exist because the locked approach has no feedback to the
target, so a fault integrates rather than self-correcting. A control phase
that cannot see its own target needs a sanity bound.

`q` stops at any time. The pendant X button overrides the Pi entirely and is
the real abort.

---

## Known gaps

- `follow` has had no field testing. Only `goto` and `mission` have driven.
- The capture hook in `mission` is an empty comment. The camera trigger goes
  there, and it should flag and continue on failure rather than aborting.
- No check of fix quality immediately before a capture, which is where it
  matters most.
- Headland turns are handled by aligning in place then driving straight.
  Works on open ground, ungainly in a tight headland.
- `ANTENNA_OFFSET_M` is duplicated in `amiga_nav.py` and `rover_log.py` and
  must be kept in sync by hand.
