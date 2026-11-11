# Navigation controller

`amiga_nav.py`. Four commands:

| | |
|---|---|
| `replay <csv>` | simulate against a logged path. No hardware, no robot. |
| `follow <csv>` | drive the path recorded in that log |
| `goto <lat> <lon>` | drive to one point and stop |
| `mission <json>` | drive the marks from a `rover_log.py teach` summary |
| `plan <json>` | print what `mission` would do, leg by leg. No hardware. |
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

## GPS error: what cancels and what does not

Teach-and-repeat with a fixed base cancels every error that is the same at
teach time and at run time: base position, antenna offset, heading offset,
and the sideways lean of the mast on sloped turf. They cancel only if the
machine sits on the same patch, **facing the same way**. Rows alternate
direction, so:

**Teach each plot exactly as it will be driven** - the real route, in the
real direction, stopped square to the row. Taught facing the other way, the
lean and offset errors double instead of cancelling.

What does not cancel is RTK noise (1-2 cm) and losing the fix. Noise is
reduced at teach time by averaging; loss is handled by holding.

### Marks are averaged and checked

`rover_log.py` averages **3 s** of readings per mark, machine stopped, and
refuses the mark - without using up the plot id - unless every reading is
RTK fixed, the wheels did not turn, and the readings agree within 2 cm and
0.5 deg. A single reading baked its noise into every future run.

### Fix loss holds, it does not abort

A GPS fault (position or heading stale, fix not RTK fixed, corrections over
5 s old) puts `drive_loop` in a **hold**: zero speed, AUTO_ACTIVE kept alive,
controller frozen. When the fix has been good for **2 s** it resumes the same
leg - same locked approach line, same latched turn direction - rather than
restarting it, because re-aiming at a target a few centimetres away would
turn the machine in place inside the plot. After **90 s** it gives up and
stops cleanly; `mission --resume` picks up where it left off.

Hard faults - CAN silent, Dashboard out of auto - still stop at once.

`GPS_HOLD_MAX_S` and `GPS_RESUME_GOOD_S` are the knobs.

### Each mark squares to its taught heading

`mission` turns to the heading each mark was **taught** with. The old
`--heading` pinned every mark to one bearing, which on a serpentine route
turned the machine 180 deg in place at every plot of every second row.
`--heading` still exists as an override; do not use it on a serpentine.

Run `amiga_nav.py plan <json>` before a mission. It flags any mark that ends
with a turn over 20 deg. Only headland transit points should appear.

### Every stop is logged

`mission` writes `<teachfile>_run_<UTC>.csv`: per mark, the stop error
(along the row, positive = past the mark; across, positive = left), heading
error, fix quality and correction age at capture, any GPS fault during the
dwell, and hold count and time. A plot with GPS trouble is marked
`flag_gps` and the run carries on. The along/cross numbers are the residual
a crop in post-processing would undo.

---

## Known gaps

- `follow` has had no field testing. Only `goto` and `mission` have driven.
  **`AlignGate` (used only by `follow` and `replay`) turns the wrong way**
  by the verified sign convention - positive bearing error returns a
  positive (left) rate - and `replay`'s vehicle model has the matching
  inverted sign, so simulation cannot catch it. Fix both before `follow`
  drives.
- The capture hook in `mission` is still an empty comment. It now has the
  fix check and the stop error to hand; it needs the camera trigger.
- Headland turns are handled by aligning in place then driving straight.
  Works on open ground, ungainly in a tight headland.
- No dead reckoning through an RTK dropout - the machine holds instead.
