#!/usr/bin/env bash
# ==============================================================================
# FLASH ESP32-S3 FIRMWARE SCRIPT FOR NAMMY ROBOT
# Uses esptool directly from the isolated nammy_ws virtual environment
# ==============================================================================
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="${WS_DIR}/.venv/bin/python3"
ESPTOOL="${WS_DIR}/.venv/bin/esptool"
FIRMWARE_DIR="${WS_DIR}/firmware/esp32s3_smartelex_driver"

echo "======================================================================"
echo "NAMMY ROBOT - ESP32-S3 FIRMWARE FLASHER"
echo "======================================================================"

# 1. Detect ESP32-S3 Port
PORT="${1:-/dev/nammy_esp32}"

if [ ! -e "$PORT" ]; then
    echo "[-] Port '$PORT' not found. Scanning for available USB serial devices..."
    AVAILABLE_PORTS=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
    if [ -z "$AVAILABLE_PORTS" ]; then
        echo "[!] ERROR: No ESP32-S3 USB serial devices found."
        echo "    Please plug in the ESP32-S3 DevKitC-1 via USB and retry."
        exit 1
    fi
    PORT=$(echo "$AVAILABLE_PORTS" | head -n 1)
    echo "[✓] Auto-selected device: $PORT"
fi

echo "[✓] Target Serial Port: $PORT"

# 2. Query Chip Information
echo "[+] Checking ESP32-S3 Connection..."
$ESPTOOL --port "$PORT" chip_id || {
    echo "[!] Could not query chip on $PORT. Make sure your user is in 'dialout' group:"
    echo "    sudo usermod -aG dialout $USER"
    exit 1
}

# 3. Check for PlatformIO
PIO_BIN="${WS_DIR}/.venv/bin/pio"
if [ -f "$PIO_BIN" ]; then
    echo "[+] PlatformIO detected in .venv. Building and uploading firmware..."
    cd "$FIRMWARE_DIR"
    $PIO_BIN run -t upload --upload-port "$PORT"
    echo "[✓] ESP32-S3 Firmware Flashed Successfully via PlatformIO!"
else
    echo "----------------------------------------------------------------------"
    echo "[INFO] PlatformIO not found in .venv."
    echo "You can compile and flash manually with:"
    echo "  ${WS_DIR}/.venv/bin/pip install platformio"
    echo "----------------------------------------------------------------------"
fi

echo "======================================================================"
