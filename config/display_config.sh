#!/usr/bin/env bash
# ==============================================================================
# DISPLAY & TOUCHSCREEN CONFIGURATION FOR 10.1" WAVESHARE IPS HDMI (SKU: WVSH0067)
# Resolution: 1280x800 @ 60Hz (Aspect Ratio: 16:10)
# ==============================================================================

echo "======================================================================"
echo "CONFIGURING 10.1\" CAPACITIVE TOUCH DISPLAY (1280x800)..."
echo "======================================================================"

export DISPLAY=:0

# 1. Detect connected HDMI / DP display output
CONNECTED_OUTPUT=$(xrandr 2>/dev/null | grep -w connected | awk '{print $1}' | head -n 1)

if [ -z "$CONNECTED_OUTPUT" ]; then
    echo "[!] No active display detected on DISPLAY=:0"
    echo "    Checking DRM connector status..."
    cat /sys/class/drm/*/status 2>/dev/null
    exit 0
fi

echo "[✓] Detected Display Output: $CONNECTED_OUTPUT"

# 2. Check if 1280x800 mode exists, else create modeline
if ! xrandr | grep -q "1280x800"; then
    echo "[+] Creating CVT Modeline for 1280x800 @ 60Hz..."
    # 1280x800 59.81 Hz (CVT 1.02MA) hsync: 49.70 kHz; pclk: 83.50 MHz
    xrandr --newmode "1280x800_60.00"   83.50  1280 1352 1480 1680  800 803 809 831 -hsync +vsync 2>/dev/null || true
    xrandr --addmode "$CONNECTED_OUTPUT" "1280x800_60.00" 2>/dev/null || true
    xrandr --output "$CONNECTED_OUTPUT" --mode "1280x800_60.00" 2>/dev/null || true
else
    echo "[✓] Applying 1280x800 resolution..."
    xrandr --output "$CONNECTED_OUTPUT" --mode "1280x800" 2>/dev/null || true
fi

# 3. Map Capacitive Touchscreen Coordinate Matrix
TOUCH_DEVICE=$(xinput --list --name-only 2>/dev/null | grep -iE "touch|waveshare|byd|egalax" | head -n 1 || true)
if [ -n "$TOUCH_DEVICE" ]; then
    echo "[✓] Mapping Touch Device '$TOUCH_DEVICE' to '$CONNECTED_OUTPUT'..."
    xinput map-to-output "$TOUCH_DEVICE" "$CONNECTED_OUTPUT" 2>/dev/null || true
fi

echo "======================================================================"
echo "Display and Touch Configuration Complete."
echo "======================================================================"
