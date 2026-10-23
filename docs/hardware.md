# Hardware

## Why the Pi and not the Amiga Brain

| Decision | Reasoning |
|---|---|
| Skip the Brain, drive from the Pi | The Brain has **no GPS module installed**. Without it the Filter service (UKF) cannot converge, so Track Follower and teach-and-repeat are both dead. The Brain's entire autonomy stack is unavailable to us regardless |
| Keep the Dashboard in the loop | The Dashboard is the VCU, not the Brain. It owns the motor controllers, enforces speed, acceleration and state-transition limits, and auto mode is armed physically on its screen. Non-optional, and also our safety layer |
| LG580P cannot feed farm-ng's GPS service | farm-ng's GPS service emits PVT and RELPOSNED — u-blox UBX message names, so it parses UBX specifically. The LG580P is Quectel (NMEA / PQTM). An independent second reason the Brain path is closed |
| Local base + radio, not NTRIP | No cell service at the test site. farm-ng also warns that VRS networks shift reference frame on reboot; a static base is the correct answer anyway |
| No compute at the base station | The F9P runs base mode standalone, RTCM3 straight out its UART into the radio. Base = receiver + antenna + radio + battery. Nothing to boot, nothing to crash |
| Write our own pure pursuit | Dual-antenna heading replaces the main thing the UKF existed to produce: heading without motion. RTK position + true heading + wheel odometry is enough for a waypoint follower in a few hundred lines |
| Retain the Brain for later | It is a Jetson with a GPU. Free prototyping platform for perception work, even if production compute ends up cheaper and repeated |

## Bill of materials

| Item | Part | Role | Cost |
|---|---|---|---|
| Base receiver | SparkFun GPS-RTK-SMA, u-blox **ZED-F9P** (GPS-16481) | Base station | $275 |
| Rover receiver | SparkFun Quadband GNSS RTK w/ Heading, Quectel **LG580P** (GPS-28019) | Rover position + heading | $249 |
| Antennas | 3 × u-blox ANN-MB-00 | 1 base, 2 rover heading baseline | $61 ea |
| Radio link | SiK Telemetry Radio V3, 915 MHz (WRL-19032) | Base-to-rover RTCM. **The kit contains two radios** — qty 1 is correct | $170 |
| Rover compute | Raspberry Pi 5 + active cooler | GPS, control loop, CAN | $230 |
| CAN interface | USB-CAN adapter (CANable / candleLight class) | Pi onto the Amiga bus. MCP2515 is ample at 250 kbps | ~$30 |

**Band note:** the ANN-MB and the F9P are both L1/L2. The LG580P is quad-band
but can only use what the base provides, so this is effectively an L1/L2 RTK
solution. Fine for cm-level.

**Still to source:** the M12 connector to tap the Amiga CAN bus, and a
24 V-to-5 V buck if powering the Pi off the bus. farm-ng's own
**Microcontroller Kit ($99)** ships a validated M12 male pigtail, a DC-DC
converter and a Feather M4 CAN — expensive for a cable, cheap for a known-good
reference CAN node if you ever need to prove the bus independently of the Pi.

## Wiring the GPS chain

The rover radio connects to the **Pi over USB**, and the Pi forwards RTCM down
USB-C to the LG580P. This avoids a custom cable — the SiK radio is 6-pin
JST-GH and the LG580P's UART3 is 4-pin — and the Pi needs to see the GPS
stream anyway.

```
BASE (no computer)
  F9P ── UART ──> SiK radio A ))) 915 MHz

ROVER
  ((( SiK radio B ── USB ──> Pi ── USB-C ──> LG580P ── 2× antenna
                              │
                              └── USB-CAN ──> Amiga M12 ──> Dashboard
```

For bench work both radios sit on one laptop so a single process sees both
ends — that is what `rtk_radio.py` assumes. `sik_range.py` is the two-ended
version for walking the plot.

## USB device identification

Every serial path in this repo resolves through `devices.py`, which
auto-detects by USB VID:PID. The only ambiguous case is the SiK pair: two
identical FT231X adapters, distinguishable only by serial number. Plug them in
one at a time and run `detect_devices.py --write` to pin the mapping down.
