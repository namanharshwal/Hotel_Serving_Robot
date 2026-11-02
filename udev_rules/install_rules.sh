#!/usr/bin/env bash
# ==============================================================================
# INSTALL UDEV RULES FOR NAMMY ROBOT
# Safely installs 99-nammy-robot.rules without overwriting hover_ws rules
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_FILE="${DIR}/99-nammy-robot.rules"
TARGET_FILE="/etc/udev/rules.d/99-nammy-robot.rules"

echo "======================================================================"
echo "INSTALLING NAMMY ROBOT UDEV RULES..."
echo "======================================================================"

if [ "$EUID" -ne 0 ]; then
    echo "[INFO] Requesting sudo privileges to install rule into /etc/udev/rules.d/..."
    sudo cp "$RULE_FILE" "$TARGET_FILE"
    sudo chmod 644 "$TARGET_FILE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
else
    cp "$RULE_FILE" "$TARGET_FILE"
    chmod 644 "$TARGET_FILE"
    udevadm control --reload-rules
    udevadm trigger
fi

echo "[✓] Successfully installed: $TARGET_FILE"
echo "[✓] Udev daemon reloaded and hardware triggers refreshed."
echo "======================================================================"
