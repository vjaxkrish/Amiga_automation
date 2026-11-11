# Runbook — field session, start to finish

Day-to-day operation. For one-time bring-up of a bare Pi see `SETUP.md`; for
what to verify on a different machine see `docs/dallas-checklist.md`.

Two machines, and they are **both named `raspberrypi`** — the Imager hostname
setting did not take on either. Tell them apart by username:

| | user | service | hardware |
|---|---|---|---|
| Base | `amiga-base` | `amiga-base.service` | Pi 4, F9P, radio A `DU0E44NK`, 1 antenna |
| Rover | `amiga-pi` | `rtcm-forward.service` | Pi 5, LG580P, radio B `DU0E5OQL`, CAN adapter, 2 antennas |

Because the hostnames collide, `ssh amiga-base@raspberrypi` may reach the
rover and fail on a host-key warning. **Connect by IP** and keep both written
down, or fix the hostnames once with `sudo hostnamectl set-hostname`.

---

## 1. Base station

Antenna outside, on a ground plane, clear sky, away from metal structures.
Radio antenna screwed on **before** power. Then power the Pi — the service
starts on its own.

```bash
ssh amiga-base@<base-ip>
journalctl -u amiga-base -n 5 --no-pager -o cat
```

`-o cat` matters: without it systemd's prefix pushes the useful columns off
the right edge of the terminal.

Want: satellite count in the high 20s, `FIXED`, and an RTCM message count
that climbs between two calls a few seconds apart.

The base needs **several minutes from power-on** before corrections are
usable — satellite acquisition first, then survey-in. Do the rover setup
while it converges.

If the service is not running:

```bash
systemctl status amiga-base
sudo systemctl restart amiga-base
```

### Survey-in accuracy, and why it matters more than it looks

A 5-minute survey-in converges to about **1 m**, and that number freezes when
survey-in completes — the base then transmits that position forever.

It is *absolute* error. Plot-to-plot geometry is unaffected and stays at
1–2 cm, so for a single session it does not matter at all.

**It matters between sessions.** Survey-in runs fresh on every boot, so
tomorrow's coordinate frame can sit a metre from today's. Any waypoint taught
in an earlier session points at the wrong ground now.

So either re-teach every session, or fix the base position once per site:

```bash
python3 base_station.py survey --hours 4 --acc 0.3 --site <name>
```

That writes `~/base/base_position.json`. Then add `--fixed` to `ExecStart` in
the service file and reboot. **Do this before teaching any plot waypoints** —
teaching first means teaching twice.

---

## 2. Rover

Power the Pi. Radio B and the LG580P on USB, CAN adapter on the Amiga bus,
both mast antennas connected, Amiga powered on.

### CAN comes up by itself

`udev/90-can.rules` configures `can0` whenever the adapter appears, including
on USB replug. Confirm rather than assume:

```bash
ip -details link show can0
```

Want `ERROR-ACTIVE`, `bitrate 250000`, `sample-point 0.875`, `restart-ms 0`.

If `can0` does not exist at all, the adapter is in DFU — `lsusb` shows
`0483:df11` instead of `1d50:606f`. Unplug, wait a full 5 seconds, plug back
in. A fast power-cycle puts CANable boards back into DFU on their own.

If the rule did not fire, do it by hand and check `which ip` against the path
in the rule:

```bash
sudo ip link set can0 up type can bitrate 250000 sample-point 0.875 restart-ms 0
```

### Corrections

`rtcm-forward.service` starts on boot and is the **only** process allowed to
open the radio or the receiver. Two readers on one serial port each get a
fraction of the bytes and both starve, which looks exactly like a dead link.

```bash
python3 rtcm_forward.py --status
```

Want `rtk_fixed`, satellites in the mid-20s, correction age under 3 s,
a heading value, and an age in milliseconds.

**First fix over the radio takes about 6 minutes from cold** — much slower
than the ~50 s you get over USB, because the added latency and occasional
dropped byte make ambiguity resolution work harder. It is not broken. Wait.

Once fixed, it stays fixed as long as the service keeps running. That is the
whole reason the service exists: the receiver drops to SBAS within about a
minute of corrections stopping, and then you wait 6 minutes again.

`dgps/sbas` with corrections arriving means the receiver is ignoring them —
usually nothing is forwarding. Check the service is actually up.

---

## 3. Teach waypoints

**Stop the forwarder first.** `rover_log.py` still opens both serial ports
directly and will fight the service.

```bash
sudo systemctl stop rtcm-forward
python3 rover_log.py check
```

Want all four non-zero: GGA, PQTMTAR, 0x18E, RTCM bytes. Zero RTCM means the
base is not reaching you. Zero PQTMTAR means heading is not resolving — check
the mast cabling. Zero 0x18E means CAN is down or the Amiga is off.

Then drive to each point on the pendant, **stop fully**, and press `m`:

```bash
python3 rover_log.py teach
```

`m` averages **3 seconds** of readings — keep hands off the pendant — and
labels sequentially, `plot_001`, `plot_002`, … `q` stops and writes
`~/logs/rover_<timestamp>.json`.

A mark is **refused** (with a beep, and the plot id not used up) if any
reading in those 3 s was not RTK fixed, the wheels moved, or the readings
scattered more than 2 cm. Wait for the fix, or move out of multipath, and
press `m` again.

**Teach the way it will be driven.** The real route, in the real order and
the real direction — row 2 facing back down row 2 — stopped square to the
row, including the headland transit points. Each mark records the heading it
was taught at, and `mission` squares up to that. Teaching every plot facing
the same way would make the machine spin in place at every plot of the
reversed rows. Marks are stored as the **rotation centre**, which is what
`amiga_nav` drives to.

Restart the forwarder when done:

```bash
sudo systemctl start rtcm-forward
```

---

## 4. Drive

Arm auto on the **tab on the Dashboard touchscreen**. Not the start button —
start engages MANUAL_ACTIVE, and RPDO1 is ignored entirely while in manual.
This is a physical interlock and no software replaces it.

```bash
python3 amiga_nav.py plan ~/logs/rover_<timestamp>.json      # check first, no motion
python3 amiga_nav.py mission ~/logs/rover_<timestamp>.json --dwell 5
python3 amiga_nav.py mission <file> --resume      # skip completed plots
python3 amiga_nav.py goto <lat> <lon> --heading <row bearing>
```

`plan` lists every leg and flags any mark that ends with a turn over 20°.
Only headland transit points should be flagged; a flagged plot was taught
facing the wrong way.

First line of output should read `reading position and heading from
rtcm_forward`. If it says `using the ports directly`, the state file is stale
and you are back in the conflicting arrangement.

Each mark squares up to the heading it was **taught** at. `--heading` still
exists to force one bearing on every mark — **do not use it on a serpentine
route**; it spins the machine 180° on every plot of the reversed rows.

**If the fix drops** the machine stops where it is and prints `HOLD` with
the reason. It stays in auto and carries on by itself once the fix has been
back for 2 s. After 90 s without a fix it stops cleanly; fix the cause and
re-run with `--resume`.

Each run writes `<teachfile>_run_<UTC>.csv` next to the teach file: stop
error along and across the row for every plot, fix status at capture, and
any GPS holds. Copy it off with the logs.

Teach files from before 11 Nov 2026 still load — their marks are re-derived
from the stored antenna fix — but they were taught one reading at a time
and possibly facing the wrong way. Re-teach for real runs.

`q` stops. **The pendant X button overrides the Pi entirely** and is the real
abort — it works whether or not the software is responding.

### Expect

| | |
|---|---|
| Position error | 1–1.5 cm typical, 3 cm worst of 15 runs |
| Final heading error | 1.5–2° |
| Time, ~10 m leg with a turn | 30–40 s |

### Do not pivot in the plots

Three spins in place took the motors to **220 °F**. Hours of normal driving
did not. Skid-steer rotating on turf scrubs every wheel sideways and produces
no useful motion, so nearly all the input becomes heat.

The Amiga cuts out on over-temperature by itself, which protects the motors —
but a thermal stop drops AUTO_ACTIVE, and **the Pi cannot re-arm it**. That
means walking out to the machine.

Headland turns are a three-point manoeuvre taught as transit waypoints:
continue past the row end, 90°, translate one row spacing, 90°, down the next
row. `calib-offset` is the one command that does spin in place — run it
sparingly.

---

## 5. Recovery

**Watchdog tripped** (state dropped 5 → 4): re-arm the auto tab by hand. The
Pi cannot do it. This is by design and it is the hard reliability constraint
for unattended operation — the nav process must not die.

**E-stop**: takes the whole CAN bus down, not just motion. Every node goes
silent, the TX queue fills, and `sock.send` raises `OSError 105`. Release the
e-stop, power-cycle the Amiga, then check the interface came back:

```bash
ip -details -statistics link show can0
```

Want `ERROR-ACTIVE` with zero `bus-off`. `restart-ms 0` means a bus-off does
**not** self-recover — deliberately, so a fault is loud.

**Interface index changed** (`4: can0` became `5: can0`): the adapter
re-enumerated on USB. Harmless in itself, but a loose USB cable will do it
again at a worse moment. Check the cable.

**Stuck in float with corrections arriving**: move the machine into the open.
Multipath off a building or vehicle is the classic cause — corrections flow,
ambiguities never resolve.

**`Permission denied` on a serial port**: the shell predates `dialout` group
membership. `newgrp dialout` fixes that terminal; logging out and back in
fixes all of them.

---

## 6. Before putting it away

Copy `~/logs/` off the rover. Note anything that behaved unexpectedly while
it is fresh — the calibration constants in `docs/calibration.md` each cost a
debugging cycle to find, and the next one will too.
