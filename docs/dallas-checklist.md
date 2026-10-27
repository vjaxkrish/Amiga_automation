# Dallas day-one checklist

The College Station machine is *believed* identical - believed, not verified.
These checks are cheap and each one catches a difference that would otherwise
look like a software bug.

## Before commanding any motion

- [ ] **Read the Dashboard firmware version off the screen.** College Station
      is **v0.8.0**. A different version may change the RPDO1 payload size,
      and that **fails silently**.
- [ ] **Capture 35 s of CAN** through boot, manual driving and e-stop. Diff
      the ID set against `logs/` from 21 Oct. Confirm `0x18E` is **8 bytes,
      not 5** - 5 bytes means firmware predates v0.1.9 and the layout differs.
- [ ] **Confirm the auto tab exists on the Dashboard screen** and behaves the
      same. The start button engages MANUAL_ACTIVE, not auto; auto is armed
      from its own tab. This cost most of a morning to find.
- [ ] **Read the Dashboard speed limit** off the GUI and match it, or record
      both. It is configurable and identical code behaves differently.

## Re-measure, do not assume

Every constant in `calibration.md` is a property of that machine and mast:

- [ ] `ANTENNA_OFFSET_M` - `python3 amiga_nav.py calib-offset`, several times
- [ ] `HEADING_OFFSET_DEG` - drive straight, compare heading to
      course-over-ground. Depends on how the mast is wired and mounted.
- [ ] `ANGULAR_MIN_MRAD_S` - slowest rate that actually rotates, on turf
- [ ] `CREEP_SPEED_MM_S` - slowest speed that actually moves, on turf
- [ ] Watchdog and kill-to-stop timings
- [ ] SiK range across that plot

## Base station, in this order

1. [ ] Antenna in its **permanent** spot, ground plane, clear sky, away from
       metal structures
2. [ ] `python3 base_station.py survey --hours 4 --acc 0.3 --site dallas`
3. [ ] Add `--fixed` to the service ExecStart, reboot, confirm it comes up
4. [ ] **Only then** teach plot waypoints

Teaching before the base has fixed coordinates means re-teaching after.

## Mast

- [ ] Rigid. At 1.35 m, a centimetre of flex is 0.4 deg of heading error.
- [ ] Baseline aligned with the vehicle axis, or record the offset angle
- [ ] Measure the separation with a tape and check the receiver agrees

## Operational

- [ ] Confirm the wheels run in the 1 ft alleys and straddle the 3 ft plots
- [ ] Power bank does not cut output at low draw - leave it an hour
- [ ] `vcgencmd get_throttled` reads `0x0` on both Pis
