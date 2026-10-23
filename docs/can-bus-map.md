# Amiga CAN bus map

Everything here was read off the wire on a farm-ng Amiga at College Station,
21-22 Oct 2026, except where marked as taken from farm-ng documentation.

## Physical layer

| Property | Value | Source |
|---|---|---|
| Standard | NMEA 2000 / DeviceNet | farm-ng docs |
| Connector | M12 5-pin A-coded | farm-ng docs |
| Bitrate | **250 kbps** | farm-ng FAQ, and the NMEA 2000 standard rate - two independent sources |
| Pin 1 | shield | farm-ng docs |
| Pin 2 | **+24 VDC** | farm-ng docs |
| Pin 3 | GND | |
| Pin 4 | CAN_H | |
| Pin 5 | CAN_L | |

The CAN tap and the Pi's power source are the same connector. The Brain's old
port is free and was used for this work.

**Power note, matters later:** the main pack is 44 VDC nominal and each fork
carries an Anderson connector for 44 V accessories. The 24 V accessory rail
comes off a Power Buck fused with 10 A automotive fuses. The Pi and CAN gear
can live on 24 V. Anything drawing real current cannot.

## Node numbering

The Dashboard is the vehicle control unit, CANopen node **0x0E**. COB-IDs
follow standard CANopen: function code in the high nibble group, node ID in
the low 7 bits.

| COB-ID | Direction | Meaning | Status |
|---|---|---|---|
| **0x20E** | Pi → Dashboard | **AmigaRpdo1** - commanded state and velocity | validated |
| **0x18E** | Dashboard → bus | **AmigaTpdo1** - control state, measured speed | validated |
| 0x70E | Dashboard → bus | NMT heartbeat. 0x05 = OPERATIONAL | validated |
| 0x20F | Dashboard → pendant | RPDO1. Only two payloads ever seen: `0000000000` and `008F809090` | observed |
| 0x28E | Dashboard → bus | TPDO2. 65 distinct payloads, **contents not decoded** | observed |
| 0x38E | Dashboard → bus | TPDO3. All zeros across 488 frames | observed |
| **0x23A** | ? | **Unmapped.** 12.4 Hz, locked to the dashboard TPDO cadence, 32 distinct payloads. Does not fit the node numbering | open question |
| 0x10, 0x11 | Dashboard → ? | NMT'd at ~1 Hz continuously, **never respond** | open question |

Motor controller TPDO1s broadcast at roughly 8 Hz each. 0x18E measures at
12-14 Hz depending on the capture.

## AmigaRpdo1 @ 0x20E — the command frame

Eight bytes. Firmware past v0.1.9 grew this from 5 to 8 bytes when PTO and
H-bridge control were added. **Getting this wrong fails silently**, so confirm
the frame length against the firmware before trusting a new machine.

```
byte 0     requested control state (uint8)
bytes 1-2  linear speed,  int16 LE, mm/s
bytes 3-4  angular rate,  int16 LE, mrad/s   <- NOT YET EXERCISED
bytes 5-7  PTO / H-bridge / reserved          <- sent as 0
```

Validated how:

- **byte 0** by the observed 4 → 5 transition on 0x18E.
- **bytes 1-2** by commanding 50 mm/s and reading 50 mm/s back from 0x18E,
  holding within ±5 mm/s over 2 s.
- **bytes 3-4 and 5-7 are unvalidated.** Sent as zero throughout. Exercise the
  angular rate field during teleop on stands before trusting it.

Stream at **20 Hz**. The dashboard watchdog reverts AUTO_ACTIVE to AUTO_READY
below roughly 5 Hz. Measured TX timing over 252 frames on a Pi 5 under a
desktop session, no special scheduling: mean 50.0 ms, median 50.0, max 52.0.
20 Hz holds comfortably.

## AmigaTpdo1 @ 0x18E — the state broadcast

```
byte 0     control state (uint8)
bytes 1-2  measured linear speed, int16 LE, mm/s
bytes 3-4  measured angular rate, int16 LE, mrad/s
byte 7     state of charge, %
```

The int16 speed decode was validated at six times the magnitude of the first
capture: the pendant drove **~893 mm/s** sustained with a clean ramp, and
-379 mm/s in reverse, with no sign-extension or rollover artefacts.

**The speed signal carries ±5 mm/s of jitter** around a steady command, and is
rougher in the first ~0.5 s of a move. A pure pursuit loop closing on it will
want filtering.

**The SOC byte is an estimator, not a measurement.** Confirmed twice: boots at
56, converges to 78-81, read 53 immediately after an e-stop. Do not gate
anything on it.

## Control state enum

| Value | Name | Confirmed |
|---|---|---|
| 0 | BOOT | not observed |
| 1 | MANUAL_READY | **observed** - stable post-boot resting state |
| 2 | MANUAL_ACTIVE | **observed** - latched by pendant or start button |
| 3 | CC_ACTIVE | **observed** - pendant cruise control |
| 4 | AUTO_READY | **observed** - commanded by us, and the watchdog fallback target |
| 5 | AUTO_ACTIVE | **observed** - commanded by us, held 274 s with no drift |
| 6 | ESTOPPED | **observed** |

Fully exercised except BOOT.

## Capture log

`logs/candump-2026-10-21_165954.log` is the 21 Oct read-only capture the first
half of this map was built from. Worth diffing a fresh capture against it on
any new machine: an unexpected COB-ID or a missing one is the fastest signal
that the firmware is not the same.

```bash
candump -l can0                       # writes candump-<timestamp>.log
canbusload can0@250000 -r -t -b -c    # sanity-check bus utilisation
```
