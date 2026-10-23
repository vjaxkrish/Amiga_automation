#!/usr/bin/env bash
# can_setup.sh - bring up the CAN interface for the Amiga bus.
#
#   sudo ./can_setup.sh          bring up can0 at 250 kbps
#   sudo ./can_setup.sh down     take it down
#   ./can_setup.sh check         report state without touching anything
#
# 250 kbps is confirmed twice over: the farm-ng FAQ states it, and it is the
# NMEA 2000 standard rate, which is what the Amiga bus physically is.
#
# restart-ms 0 is deliberate. An interface that silently recovers from bus-off
# hides exactly the faults that matter on a vehicle that moves. If the bus
# drops we want the run to fail loudly.

set -u

IFACE="${AMIGA_CAN_IFACE:-can0}"
BITRATE=250000
TXQLEN=10

check() {
    if ! ip link show "$IFACE" >/dev/null 2>&1; then
        echo "FAIL  $IFACE does not exist."
        echo "      The USB-CAN adapter is unplugged, or it re-enumerated."
        echo "      Check: ls /sys/class/net | grep can"
        return 1
    fi
    ip -details -statistics link show "$IFACE"
    echo
    local d
    d=$(ip -details link show "$IFACE")
    if grep -q "ERROR-ACTIVE" <<<"$d"; then
        echo "OK    ERROR-ACTIVE - healthy, this is the state you want."
    elif grep -q "ERROR-PASSIVE" <<<"$d"; then
        echo "WARN  ERROR-PASSIVE - the controller is seeing errors."
        echo "      Do not trust a run. Check termination and wiring."
    elif grep -q "BUS-OFF" <<<"$d"; then
        echo "FAIL  BUS-OFF - the controller has given up."
        echo "      sudo ./can_setup.sh down && sudo ./can_setup.sh"
    fi
    if ! grep -q "bitrate $BITRATE" <<<"$d"; then
        echo "WARN  bitrate is not $BITRATE. The Amiga bus is 250 kbps."
    fi
    return 0
}

case "${1:-up}" in
    check)
        check
        ;;
    down)
        ip link set "$IFACE" down && echo "$IFACE down"
        ;;
    up)
        if [[ $EUID -ne 0 ]]; then
            echo "needs root: sudo $0"
            exit 1
        fi
        ip link set "$IFACE" down 2>/dev/null
        ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 0 || {
            echo "FAIL  could not configure $IFACE."
            echo "      Adapter present? ls /sys/class/net | grep can"
            exit 1
        }
        ip link set "$IFACE" txqueuelen "$TXQLEN"
        ip link set "$IFACE" up || exit 1
        echo "$IFACE up at $BITRATE bps, restart-ms 0"
        echo
        check
        ;;
    *)
        echo "usage: $0 [up|down|check]"
        exit 2
        ;;
esac
