#!/usr/bin/env bash
# ==============================================================================
# HOTEL SERVING ROBOT - SROS2 SECURITY KEYSTORE SETUP
# Author & Copyright: Naman Harshwal (namanharshwal@gmail.com)
# License: Apache License, Version 2.0
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KEYSTORE_DIR="${WS_DIR}/security/keystore"

echo "======================================================================"
echo "         CONFIGURING SROS2 / SECURE DDS CRYPTOGRAPHIC VAULT          "
echo "======================================================================"
echo "[*] Workspace Directory: ${WS_DIR}"
echo "[*] SROS2 Keystore Path: ${KEYSTORE_DIR}"

# 1. Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
else
    echo "[!] ERROR: /opt/ros/humble/setup.bash not found!"
    exit 1
fi

# 2. Check if ros2 security is available
if ! ros2 security --help > /dev/null 2>&1; then
    echo "[!] SROS2 utilities not installed. Installing sros2..."
    sudo apt-get update && sudo apt-get install -y ros-humble-sros2
fi

# 3. Initialize Keystore if not exists
if [ ! -d "${KEYSTORE_DIR}" ]; then
    echo "[*] Initializing Master SROS2 Keystore & Root CA..."
    ros2 security create_keystore "${KEYSTORE_DIR}"
    echo "[✓] Keystore created successfully at ${KEYSTORE_DIR}"
else
    echo "[✓] Keystore already exists at ${KEYSTORE_DIR}"
fi

# 4. Generate Cryptographic Artifacts for All Nodes
NODES=(
    "nammy_base_node"
    "motor_manager_node"
    "bno055_imu_node"
    "battery_monitor_node"
    "touch_hmi_node"
    "teleop_keyboard_node"
    "sonar_fusion_node"
    "ydlidar_node"
    "robot_state_publisher"
)

echo "[*] Generating X.509 Keys & Enclaves for ${#NODES[@]} Robot Nodes..."
for node in "${NODES[@]}"; do
    echo "    -> Creating enclave for: /${node}"
    ros2 security create_enclave "${KEYSTORE_DIR}" "/${node}" || true
done

# Restrict permissions on private keys
chmod -R 700 "${KEYSTORE_DIR}"
find "${KEYSTORE_DIR}" -name "*.key" -exec chmod 600 {} + 2>/dev/null || true

echo "======================================================================"
echo "[✓] SROS2 SECURITY VAULT INITIALIZED WITH DETERMINISTIC ENCLAVES"
echo "To launch ROS 2 with mandatory cryptographic encryption and authentication:"
echo "    export ROS_SECURITY_ENABLE=true"
echo "    export ROS_SECURITY_STRATEGY=Enforce"
echo "    export ROS_SECURITY_KEYSTORE=\"${KEYSTORE_DIR}\""
echo "======================================================================"
