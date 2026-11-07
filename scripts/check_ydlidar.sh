#!/usr/bin/env bash
# ==============================================================================
# check_ydlidar.sh - Hardware Diagnostic & Port Verifier for YDLIDAR G2
# Author: Naman Harshwal <namanharshwal@gmail.com>
# ==============================================================================

echo "=================================================================="
echo "          YDLIDAR G2 PHYSICAL HARDWARE DIAGNOSTIC TOOL            "
echo "=================================================================="

# 1. Check USB Bus Enumeration (Silicon Labs CP2102: 10c4:ea60)
echo "[1] Checking USB Bus Enumeration..."
if lsusb | grep -qiE "10c4:ea60|cp210|silicon"; then
    echo -e "    \033[1;32m[OK] Silicon Labs CP2102 USB Bridge detected:\033[0m"
    lsusb | grep -iE "10c4|cp210|silicon"
else
    echo -e "    \033[1;31m[FAIL] YDLIDAR CP2102 USB Bridge NOT detected in lsusb!\033[0m"
    echo "    Please inspect your physical USB cable:"
    echo "    - Ensure the YDLIDAR USB-C / Micro-USB connector is firmly seated."
    echo "    - Plug directly into one of the blue USB 3.0 ports on the Jetson Orin Nano."
    echo "    - Check that the small red/green power LED inside the LiDAR body is ON."
    echo "=================================================================="
    exit 1
fi

# 2. Check Device Node & Udev Symlink
echo -e "\n[2] Checking Linux Serial Device Node..."
if [ -e /dev/ydlidar ]; then
    echo -e "    \033[1;32m[OK] Udev symlink verified: /dev/ydlidar -> $(readlink -f /dev/ydlidar)\033[0m"
elif [ -e /dev/ttyUSB0 ]; then
    echo -e "    \033[1;33m[WARN] Device found at /dev/ttyUSB0 (symlink /dev/ydlidar missing)\033[0m"
else
    echo -e "    \033[1;31m[FAIL] Neither /dev/ydlidar nor /dev/ttyUSB0 exist!\033[0m"
    exit 1
fi

# 3. Test High-Speed Serial Communication @ 230400 baud
echo -e "\n[3] Testing YDLIDAR G2 Packet Stream at 230400 baud..."
python3 -c "
import serial, sys, os
port = '/dev/ydlidar' if os.path.exists('/dev/ydlidar') else '/dev/ttyUSB0'
try:
    s = serial.Serial(port, 230400, timeout=1.5)
    data = s.read(200)
    s.close()
    if len(data) > 0:
        print(f'    \033[1;32m[SUCCESS] Received {len(data)} bytes of valid laser scan data from {port}!\033[0m')
    else:
        print('    \033[1;33m[WARN] Serial port opened, but 0 bytes read (check motor power wire).\033[0m')
except Exception as e:
    print('    \033[1;31m[ERROR]\033[0m', e)
"
echo "=================================================================="
