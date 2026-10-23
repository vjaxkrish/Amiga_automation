# amiga-phase0

Driving a **farm-ng Amiga** from a Raspberry Pi over raw CAN, with RTK
guidance from a local base station — without using the Amiga Brain.

Built for turf phenotyping at Texas A&M. Phase 0 is navigation only: get the
base vehicle driving itself along a row under RTK guidance. No perception, no
implements.

**Status: the vehicle moves under Pi command on stands. The GPS half is
written but has not yet met the hardware.** See [PROGRESS.md](PROGRESS.md).

---

## Why not the Brain

The Brain has no GPS module installed, so farm-ng's Filter service (UKF) can
never converge, and Track Follower and teach-and-repeat are both dead with it.
Independently, farm-ng's GPS service parses u-blox UBX specifically, and our
rover receiver is Quectel. Both doors are shut, so the Pi talks to the
Dashboard directly.

**The Dashboard stays in the loop.** It is the vehicle control unit — it owns
the motor controllers, enforces speed, acceleration and state-transition
limits, and auto mode is armed physically on its touchscreen. It is the safety
layer, not an obstacle. The Pi plans; the Dashboard enforces.

Full reasoning in [docs/hardware.md](docs/hardware.md).

---

## Quick start

```bash
pip install -r requirements.txt

python3 detect_devices.py        # what is plugged in, and as what
sudo ./can_setup.sh              # can0 at 250 kbps, restart-ms 0
python3 amiga_rpdo1.py ready     # request AUTO_READY
```

Auto mode will not engage until it is **armed on the Dashboard touchscreen**.
That is a physical interlock and it is deliberate — read
[docs/auto-mode-arming.md](docs/auto-mode-arming.md) before concluding
anything is broken. It is the finding that cost the most to make.

**Setting this up on a machine this code has not touched:** follow
[SETUP.md](SETUP.md) start to finish.

---

## The scripts

| Script | What it does |
|---|---|
| `amiga_rpdo1.py` | Stream AmigaRpdo1 to the Dashboard over raw SocketCAN. `ready` / `active` / `drive` / `kill-test`. **Stdlib only** — no python-can, no farm-ng packages |
| `can_setup.sh` | Bring up the CAN interface, and check its health |
| `detect_devices.py` | Map USB devices to roles; freeze the mapping with `--write` |
| `devices.py` | Every hardware-specific string, in one place. Resolves by env var, then `~/.amiga_devices`, then USB VID:PID auto-detect, then a baked-in default |
| `rtk_bench.py` | F9P as base, LG580P as rover, wired, no radio. Answers the RTCM3 compatibility question |
| `rtk_radio.py` | The same chain with the SiK pair in the middle |
| `sik_range.py` | Two-ended radio range test — walk the plot and watch loss climb |
| `lg580p_cmd.py` | Send Quectel PQTM commands to the LG580P; checksums computed for you |

`amiga_rpdo1.py` having no dependencies is deliberate: if the venv is broken
or pip cannot reach the network, the thing that commands the vehicle still
runs.

---

## Moving to different hardware

`devices.py` is the only file that should need attention, and usually it needs
none — it auto-detects by USB VID:PID, so a ZED-F9P is found under whatever
serial number it happens to carry.

```bash
python3 detect_devices.py            # check
python3 detect_devices.py --write    # freeze to ~/.amiga_devices
export AMIGA_LG580P=/dev/ttyACM1     # or override one at a time
```

The one ambiguous case is the SiK radio pair — two identical FT231X adapters
distinguishable only by serial number. Plug them in one at a time, then
`--write`.

**A different Amiga is the harder case, and it is not a code problem.**
Firmware past v0.1.9 grew the RPDO1 payload from 5 bytes to 8, and getting it
wrong **fails silently**. `SETUP.md` step 3 is a half-hour procedure that
proves the machine matches before anything is commanded.

---

## Safety

- **Wheels off the ground** for anything that commands motion. `amiga_rpdo1.py`
  prints this and means it.
- Speed is clamped to **±100 mm/s** in software. The Dashboard enforces its own
  limits underneath that.
- **Kill-to-stop is 902 ms** — about 0.9 m of travel at 1 m/s after the
  process dies. Measured, not estimated. That is the standoff number for
  anything placed in front of the machine.
- **An e-stop takes the whole CAN bus down**, not just motion. Any safety story
  that assumes software can act deliberately *during* an e-stop is wrong.
- **State reverts ~550 ms before motion stops.** Gate on measured speed, never
  on `state != 5`.

[docs/watchdog-and-estop.md](docs/watchdog-and-estop.md) has the measurements.

---

## Documentation

| | |
|---|---|
| [SETUP.md](SETUP.md) | Bare Pi to commanded motion, on an unfamiliar machine |
| [PROGRESS.md](PROGRESS.md) | What is done, what is proven, what is untested |
| [docs/can-bus-map.md](docs/can-bus-map.md) | COB-IDs, frame layouts, state enum |
| [docs/auto-mode-arming.md](docs/auto-mode-arming.md) | The interlock, and every dead end ruled out |
| [docs/watchdog-and-estop.md](docs/watchdog-and-estop.md) | Measured failure behaviour |
| [docs/hardware.md](docs/hardware.md) | BOM, wiring, and why the Brain is skipped |
| [docs/open-questions.md](docs/open-questions.md) | What is still unknown, ranked |

`logs/candump-2026-10-21_165954.log` is the reference capture the bus map was
built from. Diff a fresh capture against it on any new machine — an
unexpected or missing COB-ID is the fastest signal that the firmware differs.
