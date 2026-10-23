# Watchdog and e-stop — measured behaviour

Both measured 22 Oct 2026 on the College Station Amiga, wheels on stands.

## Watchdog: 552 ms, not the ~200 ms the docs imply

Phase C test: `SIGKILL` the commanding process mid-motion at 50 mm/s.

| Event | t (s) | Delta from last frame |
|---|---|---|
| Last 0x20E frame sent | 21.122 | — |
| 0x18E state 5 → 4 | 21.674 | **552 ms** |
| Measured speed reaches 0 | 22.024 | **902 ms** |

The *behaviour* is correct — it reverts to AUTO_READY exactly as farm-ng
documents. The *timing* is more than double what the "below roughly 5 Hz" line
in the docs implies. **Use the measured number.**

### The number for the safety case

**At 1 m/s field speed, 902 ms of kill-to-stop is ~0.9 m of travel after the
process dies.** That sets the minimum standoff for anything placed in front of
the machine, and it bounds how close any guard or skirt can sit to a hazard.

Re-measure this on each machine. It is a one-command test and it is the single
number a safety conversation will turn on.

### Two details that matter for controller code

- **The stop is active braking, not a coast.** Speed runs 54 → 48 → 44 → 32 →
  **-20** → 0 over 350 ms. The negative overshoot is the Dashboard's own
  deceleration limit doing its job. The profile is enforced below us and
  should look the same under load.
- **State reverts ~550 ms before motion stops.** Anything that treats
  `state != 5` as "the robot is stationary" will be wrong, by about half a
  metre at field speed. **Gate on measured speed, not on state.**

That crash-stops-the-robot is the default failure mode is a property to
preserve, not to work around.

## E-stop takes the whole CAN bus down

Not just motion. On e-stop the motor controllers, dashboard and pendant all go
silent, leaving our adapter as the only powered node on the bus.

A lone CAN node gets no ACK, so the TX queue (`qlen 10`) fills and `sock.send`
raises **`OSError: [Errno 105] No buffer space available`** after exactly ten
frames.

**This is correct behaviour, not a fault.** Error counters stayed at zero
throughout; the controller never went bus-off.

### Implication for Phase 1 and beyond

**An e-stop kills the commanding path as well as the actuators.** Any safety
story that assumes software can do something deliberate *during* an e-stop is
wrong. Whatever needs to happen on e-stop has to be true of the hardware.

### Consequences already implemented

- `amiga_rpdo1.py` wraps every send in `safe_send()` and **breaks on
  `OSError` rather than retrying**. If the bus is gone there is nothing to
  command, and exiting keeps the watchdog story simple.
- **`restart-ms 0`** is the right setting and is what `can_setup.sh` applies.
  A silently self-recovering interface would hide exactly the faults we need
  to see.
- After any bus event, confirm `ip -details -statistics link show can0` reads
  **ERROR-ACTIVE** before trusting a run. `./can_setup.sh check` does this.

## Watch for USB re-enumeration

The CAN adapter came back as a **new interface index** (`4:` → `5:`) with all
counters zeroed and the bitrate unset. On stands this is a curiosity. A loose
USB cable will do this at a much worse moment in a field.

`detect_devices.py` reports the interface state, and `can_setup.sh check`
warns if the bitrate is unset. Run one of them if anything behaves oddly.
