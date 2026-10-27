# RTK: base station and rover

## Hardware

| | |
|---|---|
| Base | Pi 4, ZED-F9P (SparkFun GPS-16481), ANN-MB-00, SiK radio A `DU0E44NK` |
| Rover | Pi 5, LG580P (SparkFun GPS-28019), 2x ANN-MB-00, SiK radio B `DU0E5OQL` |
| Mast | front-back baseline, 53 in (1.346 m). Receiver reports 1.3315 m mean. |

Antenna connectors are SMA on both boards and on the ANN-MB-00. No adapters.
Both LG580P SMA connectors supply 3.3 V for active antennas; no external DC
injection needed.

LG580P UART1 is 460800 baud over USB-C. The F9P is USB CDC.

## Base station

`base_station.py` on the Pi 4, started by `systemd/amiga-base.service`.
Configures the F9P, runs survey-in, pipes RTCM3 into the radio, logs status.
No operator, no screen, no network needed in the field.

RTCM3 message set: 1005, 1074, 1084, 1094, 1124, 1230 at 1 Hz.

### Survey-in accuracy is the thing to get right

A 5-minute survey-in converged to **1.031 m**. That figure freezes when
survey-in completes and the base transmits that position forever.

It is *absolute* error and every rover position inherits it. For plot-to-plot
geometry it cancels entirely and is irrelevant.

**It stops being irrelevant across sessions.** Survey-in runs fresh on every
boot, so tomorrow's coordinate frame can sit a metre from today's and taught
waypoints will not line up.

So, per site, once:

```
python3 base_station.py survey --hours 4 --acc 0.3 --site <name>
```

writes `base_position.json`, then add `--fixed` to the service's ExecStart.
**Do this at Dallas before teaching any plots.** Order matters.

## Rover

`rtcm_forward.py`, started by `systemd/rtcm-forward.service`, is the **only**
process that opens the radio or the receiver. Two readers on one serial port
each get a fraction of the bytes and both starve, which looks exactly like a
dead link.

It republishes position, heading, fix quality and correction age to
`/dev/shm/rover_state.json` at 10 Hz, written atomically. `amiga_nav.py`
reads that file and never touches the ports; it falls back to opening them
directly if the service is not running.

Health check:

```
python3 rtcm_forward.py --status
```

### Why it has to run continuously

The receiver drops out of RTK within about a minute of corrections stopping,
and reacquiring over the radio takes several minutes. Forwarding only while
some program happens to be running means waiting for a fix every time.

## Measured behaviour

| | |
|---|---|
| RTK fixed from cold, corrections over USB | ~50 s |
| RTK fixed from cold, corrections over radio | ~6 min |
| Correction age, USB | 0.1 - 1.0 s |
| Correction age, radio | 1.1 - 2.0 s |
| Radio packet loss, both ends within 10 m | 1 in 2681 |
| SiK serial rate | 57600 |

**Correction age above ~3 s means the link is struggling** and it shows there
before the fix actually drops. That is the field diagnostic.

Radio range across a real plot has **not** been measured. `sik_range.py` has
`send` and `recv` halves for doing it with one person. Open question.

## Heading

`$GNTHS` is empty on this firmware (LG580P03AANR01A04S_SH) even when attitude
is valid - a known complaint on the SparkFun forum. **Use `$PQTMTAR`**, which
carries heading, pitch, baseline length and satellite count.

Baseline length reported by the receiver matching the tape measure is the
check that the heading solution is genuinely resolved rather than a plausible
wrong answer.

See `calibration.md` for the 180 deg heading offset.
