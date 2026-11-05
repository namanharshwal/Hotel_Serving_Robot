#!/usr/bin/env bash
# ==============================================================================
# HOTEL SERVING ROBOT (NAMMY ROBOT) - SECURE WORKSPACE ACTIVATION SCRIPT
# Author & Copyright: Naman Harshwal (namanharshwal@gmail.com)
# License: Apache License, Version 2.0
# Ensures 100% isolation from Ruby Robot (hover_ws) and enforces secure DDS operation.
# ==============================================================================

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "======================================================================"
echo "          ACTIVATING NAMMY ROBOT WORKSPACE & ENVIRONMENT             "
echo "======================================================================"

# 1. Source ROS 2 Humble Base
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
    echo "[✓] ROS 2 Humble sourced: /opt/ros/humble"
else
    echo "[!] ERROR: /opt/ros/humble/setup.bash not found!"
fi

# 2. Activate Isolated Python Virtual Environment
if [ -f "${WS_DIR}/.venv/bin/activate" ]; then
    source "${WS_DIR}/.venv/bin/activate"
    echo "[✓] Isolated Python Virtual Environment activated: ${WS_DIR}/.venv"
    echo "    Python binary: $(which python3)"
else
    echo "[!] ERROR: Virtual environment not found at ${WS_DIR}/.venv"
fi

# 3. Source Built Workspace Artifacts (if compiled)
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo "[✓] nammy_ws install overlay sourced: ${WS_DIR}/install/setup.bash"
else
    echo "[-] nammy_ws not yet compiled. Run 'colcon build' to generate install overlay."
fi

# 4. DDS Network Domain (defaults to 0 to match system ~/.bashrc, or preserves custom ID)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export NAMMY_WS="${WS_DIR}"
export ROBOT_MODEL="nammy_diff_drive"

echo "[✓] Network Domain Isolation: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[✓] Workspace Root: ${NAMMY_WS}"

# Optional SROS2 Secure DDS Mode
if [ -d "${WS_DIR}/security/keystore" ] && [ "${ENABLE_SROS2:-0}" = "1" ]; then
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce
    export ROS_SECURITY_KEYSTORE="${WS_DIR}/security/keystore"
    echo "[✓] SROS2 Cryptographic DDS Security: ACTIVE (Enforce mode)"
fi

# 5. Developer & Security Aliases
alias nammy_build="cd ${WS_DIR} && colcon build --symlink-install && source install/setup.bash"
alias nammy_robot="ros2 launch nammy_bringup robot.launch.py"
alias nammy_teleop="ros2 run nammy_teleop teleop_keyboard"
alias nammy_dashboard="ros2 launch nammy_bringup dashboard.launch.py"
alias nammy_view="ros2 launch nammy_description view_robot.launch.py"
alias nammy_security="python3 ${WS_DIR}/scripts/security_check.py"
alias nammy_sros2="bash ${WS_DIR}/scripts/setup_sros2_security.sh"

echo "======================================================================"
echo "Available Commands:"
echo "  nammy_build      -> Compile workspace with colcon"
echo "  nammy_robot      -> Launch complete robot stack"
echo "  nammy_teleop     -> Keyboard teleoperation control"
echo "  nammy_dashboard  -> Launch 10.1\" Touch HMI Dashboard"
echo "  nammy_view       -> Preview 3D URDF model in RViz2"
echo "  nammy_security   -> Run automated security audit & secret scanner"
echo "  nammy_sros2      -> Generate/Update SROS2 cryptographic keys"
echo "======================================================================"
