#!/usr/bin/env bash
set -euo pipefail
# Limit access to the two Orbbec camera models installed at this station.
sudo tee /etc/udev/rules.d/69-robol2l-orbbec.rules >/dev/null <<'RULES'
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0800|0840", GROUP="plugdev", MODE="0660", TAG+="uaccess"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2bc5
sudo udevadm settle
