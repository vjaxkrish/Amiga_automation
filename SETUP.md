# Setup — bare Pi to commanded motion

Written to be followed on an Amiga and a Pi that this code has never touched.
That is the Dallas case, and it is also the test of whether anything here is
accidentally specific to the College Station rig.

**Budget half a day for steps 1-6.** Step 3 is the one that is worth not
rushing: it is where a quietly different machine announces itself, and a
difference found there costs an hour, while the same difference found in
step 6 costs a day.

---

## 0. Before you travel

- [ ] Read `docs/auto-mode-arming.md`. It is the thing that is not in any
      farm-ng document.
- [ ] Pack a USB-CAN adapter **and a spare**, plus a short USB cable with a
      strain-relief plan. A loose adapter re-enumerates the interface. See the
      last section of `docs/watchdog-and-estop.md`.
- [ ] Pack the M12 pigtail. The CAN tap and the Pi's power are the same
      connector.
- [ ] Confirm you can get onto the Dallas Amiga's Dashboard screen. Everything
      here depends on arming auto mode physically.

---

## 1. Pi setup

```bash
sudo apt update
sudo apt install -y can-utils python3-pip python3-venv git

git clone <this repo> amiga-phase0
cd amiga-phase0

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`can-utils` gives you `candump`, `cansend` and `canbusload`, all of which you
will want in step 3.

`amiga_rpdo1.py` needs no packages at all — raw SocketCAN, stdlib only. If the
venv is broken or pip cannot reach the network, the CAN half of this repo
still runs. That is deliberate.

---

## 2. Bring up CAN

Plug the adapter into the Amiga's M12 CAN tap — the Brain's old port works and
is free.

```bash
sudo ./can_setup.sh          # 250 kbps, restart-ms 0, txqueuelen 10
./can_setup.sh check         # expect ERROR-ACTIVE
```

If `can0` does not exist, the adapter did not enumerate:

```bash
ls /sys/class/net | grep can
dmesg | tail -20
```

**ERROR-ACTIVE is the state you want.** ERROR-PASSIVE means the controller is
seeing errors — check termination and wiring before going further; do not
"just try it".

---

## 3. Prove the machine is the same machine

**Do not skip this.** Everything downstream assumes the firmware matches, and
the failure mode of a mismatch is silence, not an error.

### 3a. Listen only

Power the Amiga, touch nothing, and capture for a couple of minutes:

```bash
candump -l can0
```

Then compare against the reference capture:

```bash
# which COB-IDs appear here
awk '{print $3}' candump-*.log | cut -d'#' -f1 | sort -u

# which appeared at College Station
awk '{print $3}' logs/candump-2026-10-21_165954.log | cut -d'#' -f1 | sort -u
```

Expect: `0x18E`, `0x20F`, `0x28E`, `0x38E`, `0x70E`, `0x23A`, plus motor
controller TPDO1s. See `docs/can-bus-map.md`.

- **An ID present here that is missing there**, or vice versa → the machines
  differ. Work out how before commanding anything.
- **`0x10` / `0x11` NMT retry loop present** → same as College Station. It is
  unexplained but it is consistent, which is information.

### 3b. Check the frame length

Watch 0x20E — nothing should be transmitting there, it is the dashboard's
inbox. Watch 0x18E and confirm **8-byte** payloads:

```bash
candump can0,18E:7FF -l
```

**If 0x18E is 5 bytes rather than 8, the firmware predates v0.1.9 and the
RPDO1 layout is different.** Stop and resolve that; `amiga_rpdo1.py` assumes
8. This is the single most likely way Dallas differs.

### 3c. Read the firmware version off the Dashboard screen

Never actually done at College Station. Do it here and write the number into
`PROGRESS.md`, so the next machine has something to compare against.

### 3d. Exercise the pendant

Drive it manually and watch 0x18E decode sensibly — speed tracking the stick,
sign correct in reverse. The College Station pendant topped out at ~893 mm/s
forward and -379 mm/s reverse with clean ramps.

---

## 4. Auto mode, zero velocity

Wheels still on the ground is fine here; nothing moves.

```bash
python3 amiga_rpdo1.py ready     # request AUTO_READY, state 4
```

Follow `docs/auto-mode-arming.md` exactly:

1. Power on, wait for 0x70E = 0x05. The script does this and will say so.
2. State should sit at **1 MANUAL_READY**. **Touch nothing** — the pendant or
   the start button will latch MANUAL_ACTIVE and it does not time out.
3. **Arm auto on the Dashboard touchscreen.**
4. The script's state request is now honoured, within one frame.

When `ready` works, try `active` (state 5, still zero velocity). Hold it a
minute and confirm it does not drift. College Station held AUTO_ACTIVE for
274 s with no drift.

**If this does not work, go back to `docs/auto-mode-arming.md` — the
troubleshooting list at the bottom is ordered by likelihood.**

---

## 5. Motion — WHEELS OFF THE GROUND

Jack stands. Pendant in hand. The circle button disengages.

```bash
python3 amiga_rpdo1.py drive     # 50 mm/s, 2 s, ramped, back to READY
```

Confirm on 0x18E that the measured speed tracks the commanded 50 mm/s within
±5 mm/s. If it does, the command path is proven end to end.

---

## 6. Prove the failure mode

**Do this on every machine. It is the number the safety case rests on.**

Terminal 1:

```bash
python3 amiga_rpdo1.py kill-test     # holds 50 mm/s
```

Terminal 2:

```bash
kill -9 $(pgrep -f amiga_rpdo1)
```

Watch the wheels stop. Capture the timings with `candump -l can0` running
alongside and measure:

- time from last 0x20E frame to the 0x18E state 5 → 4 transition
  (College Station: **552 ms**)
- time from last frame to measured speed 0
  (College Station: **902 ms**)

Write both into `PROGRESS.md`. At 1 m/s that second number is how far the
machine travels after the software dies, and it sets the standoff for
anything you put in front of it.

Expect active braking with a negative overshoot, not a coast. See
`docs/watchdog-and-estop.md`.

---

## 7. GPS — needs no robot, run in parallel

Everything below can be done at a desk while the Amiga work waits on weather,
a jack, or someone else's turn with the machine.

```bash
python3 detect_devices.py            # what is plugged in, and as what
python3 detect_devices.py --write    # freeze the mapping
```

The SiK pair is the one ambiguous case — two identical FT231X adapters.
Plug them in one at a time to work out which is which, then `--write`.

### 7a. F9P as base, LG580P as rover, wired, no radio

```bash
python3 rtk_bench.py config     # F9P into survey-in, RTCM3 out, RAM only
python3 rtk_bench.py svin       # watch survey-in converge
python3 rtk_bench.py bridge     # F9P RTCM3 -> LG580P, report both ends
```

**This answers open question 4 — the highest-risk unknown in Phase 0.** Two
vendors' message sets is where a quiet incompatibility hides. You want the
LG580P reaching RTK FIX with a sane correction age.

Config is written to RAM only, so a power cycle reverts the F9P. Deliberate:
nothing permanent until the setup is proven.

### 7b. Dual-antenna heading

```bash
python3 lg580p_cmd.py PQTMVERNO
python3 lg580p_cmd.py PQTMCFGNAVMODE,R
python3 lg580p_cmd.py PQTMCFGMSGRATE,W,PQTMTAR,1,1
python3 lg580p_cmd.py PQTMCFGMSGRATE,W,THS,1
python3 lg580p_cmd.py PQTMSAVEPAR
python3 lg580p_cmd.py PQTMSRR
```

Heading quality depends on the antenna baseline length — open question 5, and
it gates mast fabrication.

### 7c. The radio link

Bench first, both radios on one machine:

```bash
python3 rtk_radio.py airtest    # pattern A -> B, loss only, no GNSS
python3 rtk_radio.py bridge     # full chain with the radio in the middle
```

Then the plot, two-ended:

```bash
python3 sik_range.py send       # base, with radio A
python3 sik_range.py recv       # rover, with radio B, walking
```

**Re-measure range at Dallas regardless of the College Station number.** Range
is a property of that plot's terrain and antenna height, not of the radios.

---

## 8. What is not written yet

Steps 3, 5 and 6 of the bring-up order in `PROGRESS.md`: teleop, logging while
driving manually, and the pure pursuit loop. The teleop step is where the
angular rate field finally gets exercised — it is the one part of RPDO1 still
unvalidated.
