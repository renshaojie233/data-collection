#!/bin/bash
set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
    echo "Please run as a normal user, not root."
    exit 1
fi

RULE_SRC="$HOME/99-gello-ftdi.rules"
RULE_DST="/etc/udev/rules.d/99-gello-ftdi.rules"

if [ ! -f "$RULE_SRC" ]; then
    echo "Missing $RULE_SRC. Make sure the bundle is extracted to your home directory."
    exit 1
fi

echo "Installing udev rule..."
sudo cp "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger

if ! id -nG "$USER" | tr ' ' '\n' | grep -q '^dialout$'; then
    echo "Adding user to dialout group..."
    sudo usermod -aG dialout "$USER"
    echo "Please run: newgrp dialout (or log out and log back in)."
fi

echo "Done."
echo "Real robot: ~/gello_software/run_fr3_real_ros2.sh"
echo "Simulation: ~/gello_software/run_fr3_sim.sh"
