# Entering auto mode — the control path

**This cost most of a morning to find and is not written down in any farm-ng
document we have.** If one page of this repo saves time on a new machine, it
is this one.

## The short version

**The Dashboard touchscreen arms auto mode as a physical interlock. The Pi
owns the state machine and velocity only after that.**

## What works

1. Power on. Wait for the dashboard NMT heartbeat on 0x70E to read **0x05**
   (OPERATIONAL).
2. State sits at **1 MANUAL_READY**, stable indefinitely. **Touch nothing.**
3. **Arm auto from the tab on the Dashboard touchscreen.**
4. Start streaming RPDO1 at 0x20E, 20 Hz. The state request is then honoured:
   4 → 5 observed at t = 0.05 s, i.e. within one frame.

`amiga_rpdo1.py ready` and `amiga_rpdo1.py active` both gate on step 1
automatically and will time out with a clear message if the heartbeat never
reaches OPERATIONAL.

## What does not work — each one verified, each one a dead end

- **The "start" button engages MANUAL_ACTIVE, not auto.** Pressed with a live
  RPDO1 stream already arriving at 0x20E, it still goes to manual. It is not
  the auto path.
- **RPDO1 is ignored entirely while the dashboard is in MANUAL_ACTIVE
  (state 2).** Requesting state 4 or state 5 from manual does nothing at all.
  The pendant and the start button both latch MANUAL_ACTIVE and hold it. **It
  does not time out.** If you are stuck here, the machine must be returned to
  MANUAL_READY before anything will respond.
- **Requesting AUTO_ACTIVE from state 1 does nothing** if the auto tab has not
  been armed on the screen.

MANUAL_READY is the only state auto can be entered from, and it is where the
machine sits after boot. Every earlier failure began from the wrong side of it.

## Do not try to automate step 3

A vehicle that can put itself into autonomous mode over the bus, with nobody
standing at the machine, is exactly what the interlock exists to prevent.

This is the same division of labour the whole architecture assumes: the
Dashboard enforces state and limits, the Pi commands motion. Keep it.

## On a new machine

If step 4 does not produce a 4 → 5 transition within a second or two, work
through this in order:

1. Is 0x70E reading 0x05? If not, the Amiga is not fully booted or the
   adapter is not actually on the bus.
2. Is 0x18E reading **1**, not 2? If it reads 2, something latched manual —
   the pendant or the start button. Clear it.
3. Did the auto tab actually arm on the screen? It is a physical act and has
   to be redone after every power cycle.
4. Is the frame 8 bytes? Older firmware wants 5. See `docs/can-bus-map.md`.
