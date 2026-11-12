# systemd units

Both use `Restart=always`: neither machine has an operator.

## Base, on the Pi 4

`amiga-base.service` -> `base_station.py run`

Add `--fixed` to ExecStart once a site position has been surveyed and
`base_position.json` exists. See `../docs/rtk-setup.md`.

## Rover, on the Pi 5

`rtcm-forward.service` -> `rtcm_forward.py`

**This is the only process allowed to open the radio or the receiver.**
Leave it running. `rover_log.py` and `amiga_nav.py` both detect it and read
`/dev/shm/rover_state.json` instead of the ports; each opens the ports
itself only when the service is not running.

After copying a new `rtcm_forward.py` to the Pi:

    sudo systemctl restart rtcm-forward

The fix survives a restart: the receiver holds RTK for about a minute
without corrections, and the restart takes a second or two.

## Install

    sudo cp <unit> /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable --now <name>
    systemctl status <name>
