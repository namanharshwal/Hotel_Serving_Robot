#!/usr/bin/env bash
# ==============================================================================
# enable_camera.sh - Un-blacklist & load UVC driver for Logitech C270 Webcam
# Author: Naman Harshwal <namanharshwal@gmail.com>
# ==============================================================================

set -e

echo "=================================================================="
echo "    ENABLING LOGITECH C270 WEBCAM (UVC VIDEO DRIVER SETUP)        "
echo "=================================================================="

# 1. Un-blacklist uvcvideo if blacklisted in /etc/modprobe.d/uvc.conf
if [ -f /etc/modprobe.d/uvc.conf ]; then
    echo "[*] Found /etc/modprobe.d/uvc.conf with blacklist rules. Removing..."
    sudo sed -i 's/^blacklist uvcvideo/# blacklist uvcvideo/g' /etc/modprobe.d/uvc.conf
    echo "[+] uvcvideo un-blacklisted in /etc/modprobe.d/uvc.conf"
fi

# 2. Insert uvcvideo kernel driver
echo "[*] Loading uvcvideo kernel module..."
sudo modprobe uvcvideo

# 3. Check for /dev/video0
sleep 1
if [ -e /dev/video0 ]; then
    echo -e "\n\033[1;32m[SUCCESS] Logitech C270 Webcam is ready at /dev/video0!\033[0m"
    ls -l /dev/video*
else
    echo -e "\n\033[1;33m[!] uvcvideo loaded. If /dev/video0 did not appear, please unplug and re-plug the C270 USB connector.\033[0m"
fi
echo "=================================================================="
