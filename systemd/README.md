# systemd units

Both use `Restart=always`: neither machine has an operator.

## Base, on the Pi 4

`amiga-base.service` -> `base_station.py run`

Add `--fixed` to ExecStart once a site position has been surveyed and
`base_position.json` exists. See `../docs/rtk-setup.md`.

## Rover, on the Pi 5

`rtcm-forward.service` -> `rtcm_forward.py`

**This is the only process allowed to open the radio or the receiver.**
`rover_log.py` still opens them directly, so stop this service before
logging or teaching:

    sudo systemctl stop rtcm-forward
    python3 rover_log.py teach
    sudo systemctl start rtcm-forward

`amiga_nav.py` does not need this - it detects the service and reads
`/dev/shm/rover_state.json` instead.

## Install

    sudo cp <unit> /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable --now <name>
    systemctl status <name>
