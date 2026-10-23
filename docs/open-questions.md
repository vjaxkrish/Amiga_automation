# Open questions

As of 23 Oct 2026. Resolved ones are kept, struck through, because knowing a
question was closed is as useful as the answer.

## Resolved

1. ~~**CAN connector type, and whether the Brain's old port is reusable.**~~
   **RESOLVED 21 Oct.** M12 5-pin A-coded NMEA 2000. The Brain's old port is
   live and usable.
2. ~~**Dashboard firmware version, therefore RPDO1 payload size.**~~
   **RESOLVED: 8 bytes.** The version string has still never actually been
   read off the screen — worth doing opportunistically, and **worth doing
   deliberately on the Dallas machine**, since the whole same-firmware
   assumption rests on it.
3. ~~**Does the start button or an NMT handshake gate auto mode?**~~
   **RESOLVED 22 Oct: no.** Auto is gated by the screen tab. See
   `docs/auto-mode-arming.md`.

## Open — GPS, now the critical path

4. **Does the LG580P ingest F9P RTCM3 MSM4 cleanly, with no message-set
   mismatch?** **The highest-risk unknown in Phase 0.** Two vendors' message
   sets is where a quiet incompatibility hides, and everything from bring-up
   step 4 onward depends on it. Test with `rtk_bench.py bridge`.
5. **Antenna baseline length needed to hit the claimed 0.1° heading.**
   Gates mast fabrication, so it gates anything physical in Dallas.
6. **Actual SiK range across the test plot**, with real terrain and real
   antenna height. Test with `sik_range.py send` / `recv`. Plot-specific —
   **must be re-measured at Dallas regardless of the College Station result.**

## Open — control

7. **Is wheel odometry good enough to bridge brief RTK dropouts?** The CAN
   sources are 0x18E speed at 12-14 Hz and the motor controller TPDO1s at
   ~8 Hz each. Now also known: **the 0x18E speed signal carries ±5 mm/s of
   jitter** around a steady command, rougher in the first ~0.5 s of a move. A
   pure pursuit loop closing on it will want filtering.
8. **Angular rate and RPDO1 bytes 5-7 are unvalidated.** Only the state byte
   and the linear speed have been proven on the wire. Exercise angular rate
   during teleop on stands before trusting it.

## Open — unexplained bus traffic

9. **What are CANopen nodes 0x10 and 0x11?** The Dashboard NMTs both at ~1 Hz
   continuously and neither ever responds. Reproduced in two independent
   captures. Was briefly suspected of gating auto mode — **that hypothesis is
   dead.** Still unexplained, still plausibly related to the open intermittent
   rear-motor and battery telemetry faults. **Query with farm-ng, log
   attached.**
10. **What is 0x23A?** 12.4 Hz, locked to the dashboard's TPDO cadence, 32
    distinct payloads, does not fit the node numbering established so far.
    Not investigated.
11. **What is in 0x28E (Dashboard TPDO2)?** 65 distinct payloads, contents
    not decoded.

## Questions this repo exists to answer

12. **Is the Dallas Amiga genuinely identical?** Assumed, not verified. The
    first hour there should be spent proving it rather than assuming it —
    `SETUP.md` step 3 is written for exactly this. A firmware difference that
    changes the RPDO1 payload length **fails silently**.
