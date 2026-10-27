# Open questions

## Resolved 27 Oct 2026

- ~~Does the LG580P ingest F9P RTCM3 MSM4 cleanly?~~ **Yes.** All six message
  types, RTK FIXED over both USB and radio. Was the highest-risk unknown.
- ~~Antenna baseline length for 0.1 deg heading.~~ 1.35 m resolves cleanly.
  Receiver-reported baseline matched the tape to ~1.5 cm.
- ~~What arms AUTO_ACTIVE?~~ A tab on the Dashboard touchscreen, by hand.
  The start button goes to MANUAL_ACTIVE. RPDO1 is ignored entirely while in
  manual.
- ~~Angular rate and RPDO1 bytes 3-4.~~ Validated. Positive mrad/s turns left.
- ~~Minimum speed that moves on turf.~~ 40 mm/s.
- ~~Stopping repeatability.~~ 1-1.5 cm typical over 15+ runs, 3 cm worst.

## Open

1. **SiK radio range across a real plot.** Only tested with both ends within
   10 m. `sik_range.py send` / `recv` does it with one person.
2. **Does a brief RPDO1 gap recover?** A full watchdog trip needs the screen
   re-armed by hand. Whether a gap shorter than 552 ms recovers cleanly is
   untested, and it decides how much margin an unattended machine has.
3. **CANopen nodes 0x10 and 0x11.** The Dashboard NMTs both at ~1 Hz
   continuously; neither ever responds. Reproduced in two independent
   captures. Was briefly suspected of gating auto mode - that hypothesis is
   dead. Still unexplained, plausibly related to the open intermittent
   rear-motor and battery telemetry faults.
4. **What is 0x23A?** 12.4 Hz, locked to the Dashboard TPDO cadence, 32
   distinct payloads, does not fit the node numbering.
5. **Wheel odometry for bridging RTK dropouts.** 0x18E speed at 12-14 Hz
   carries +/-5 mm/s of jitter around a steady command, rougher in the first
   ~0.5 s of a move. Any loop closing on it wants filtering.
6. **Does `follow` work?** Written and passing in simulation, never driven.
7. **Capture integration.** The hook in `mission` is an empty comment. Needs
   the camera trigger, a success/failure return, and a fix-quality check
   immediately before firing.

## Deliberately not pursued

- An EKF. The failures so far were control-law sign errors against correct
  measurements, which a filter does not help. It earns its place later for
  bridging RTK dropouts and smoothing the 12-14 Hz speed signal.
- Querying farm-ng about 0x10/0x11. Not worth the time.
