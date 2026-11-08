#!/usr/bin/env bash
# ==============================================================================
# Hotel Serving Robot (Nammy Robot) - Physical Manual Control (Zero Simulation)
# Author & Copyright: Naman Harshwal <namanharshwal@gmail.com>
# License: Apache-2.0
# ==============================================================================
# Description:
#   Direct physical teleoperation launcher for manual robot movement.
#   Launches the physical robot hardware bridge (ESP32-S3), BNO055 IMU heading
#   stabilization, and runs the keyboard teleoperation interface directly
#   in the foreground of this terminal.
#   NO simulations, NO RViz, NO Nav2, NO mapping - pure physical manual control.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ROS 2 Humble and workspace
source /opt/ros/humble/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Check for hardware ports
if [ ! -e "/dev/nammy_esp32" ] && [ ! -e "/dev/ttyACM0" ]; then
    echo "[!] WARNING: ESP32 device node not found (/dev/nammy_esp32 or /dev/ttyACM0)."
    echo "    Please plug in the ESP32 USB cable."
fi

echo "======================================================================"
echo "          NAMMY ROBOT - PHYSICAL MANUAL CONTROL (TELEOP)             "
echo "======================================================================"
echo " Mode: Physical Hardware ONLY (Zero Simulation, Zero RViz)"
echo " Hardware Port: /dev/nammy_esp32"
echo " IMU I2C Bus:   /dev/i2c-7 (Pin 27=SDA, Pin 28=SCL)"
echo " Network:       ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "======================================================================"

# Clean up any lingering teleop or bridge processes
pkill -f "esp32_hardware_bridge" 2>/dev/null || true
pkill -f "bno055_node" 2>/dev/null || true
pkill -f "teleop_keyboard" 2>/dev/null || true
sleep 0.5

# Function to clean up background processes on exit
cleanup() {
    echo ""
    echo "[*] Shutting down physical manual control..."
    if [ -n "${BRIDGE_PID}" ]; then
        kill "${BRIDGE_PID}" 2>/dev/null || true
    fi
    echo "[✓] Physical robot halted safely."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Launch hardware base in background (robot hardware only, no rviz, no sim)
ros2 launch nammy_bringup robot.launch.py \
    simulate_hardware:=false \
    simulate_lidar:=false \
    launch_lidar:=false \
    launch_camera:=false \
    launch_sonar:=true \
    launch_rviz:=false \
    i2c_bus:=7 \
    teleop:=false &
BRIDGE_PID=$!

echo "[*] Initializing hardware bridge & IMU heading hold (waiting 3s)..."
sleep 3

# Run interactive teleoperation keyboard in foreground
echo "[✓] Hardware ready! Launching teleop driving controls..."
ros2 run nammy_teleop teleop_keyboard
