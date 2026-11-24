#!/usr/bin/env bash
# Bring up the two ARX USB2CAN adapters on this PC. No motor commands are sent.
set -euo pipefail

readonly adapter_a=/dev/serial/by-id/usb-ARX_USB2CAN_003800444C48570920373330-if00
readonly adapter_b=/dev/serial/by-id/usb-ARX_USB2CAN_002000314C48570B20353734-if00

for adapter in "$adapter_a" "$adapter_b"; do
    if [[ ! -c $adapter ]]; then
        printf 'USB-CAN adapter not found: %s\n' "$adapter" >&2
        exit 1
    fi
done

sudo -v
if ! command -v slcand >/dev/null || ! command -v candump >/dev/null; then
    sudo apt-get install -y can-utils
fi
sudo modprobe slcan
sudo modprobe can_raw

start_can() {
    local adapter=$1 interface=$2 slcand_bin
    slcand_bin=$(command -v slcand)

    if [[ -e /sys/class/net/$interface ]] &&
        ! pgrep -fx -- "$slcand_bin -o -f -s8 $adapter $interface" >/dev/null; then
        printf '%s already exists; verify its USB-CAN binding before continuing.\n' "$interface" >&2
        return 1
    fi

    if [[ ! -e /sys/class/net/$interface ]]; then
        sudo "$slcand_bin" -o -f -s8 "$adapter" "$interface"
        for _ in {1..30}; do
            [[ -e /sys/class/net/$interface ]] && break
            sleep 0.1
        done
    fi

    sudo ip link set dev "$interface" up
    ip -brief link show dev "$interface"
}

start_can "$adapter_a" can1
start_can "$adapter_b" can3
