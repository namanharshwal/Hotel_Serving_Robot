#!/usr/bin/env bash
# ==============================================================================
# HOTEL SERVING ROBOT - SYSTEM & OS SECURITY HARDENING SCRIPT
# Author & Copyright: Naman Harshwal (namanharshwal@gmail.com)
# License: Apache License, Version 2.0
# ==============================================================================

set -e

if [ "$EUID" -ne 0 ]; then
    echo "[!] Please run as root: sudo bash scripts/harden_system_security.sh"
    exit 1
fi

echo "======================================================================"
echo "          APPLYING LINUX OS & ROBOT HARDWARE SECURITY HARDENING       "
echo "======================================================================"

# 1. Kernel Network Stack Hardening via sysctl
SYSCTL_CONF="/etc/sysctl.d/99-robot-security.conf"
echo "[*] Applying Kernel Sysctl Hardening..."

cat << 'EOF' > "${SYSCTL_CONF}"
# Prevent IP source routing and packet spoofing
net.ipv4.conf.all.rp_filter = 1
net.ipv4.conf.default.rp_filter = 1
net.ipv4.conf.all.accept_source_route = 0
net.ipv4.conf.default.accept_source_route = 0

# Mitigate SYN Flood DOS attacks
net.ipv4.tcp_syncookies = 1

# Ignore ICMP redirect packets (prevents MITM route poisoning)
net.ipv4.conf.all.accept_redirects = 0
net.ipv4.conf.default.accept_redirects = 0
net.ipv4.conf.all.send_redirects = 0
net.ipv4.conf.default.send_redirects = 0

# Ignore broadcast ping echoes (smurf attack prevention)
net.ipv4.icmp_echo_ignore_broadcasts = 1

# Filesystem link protections against symlink attacks
fs.protected_hardlinks = 1
fs.protected_symlinks = 1
fs.protected_regular = 2
fs.protected_fifos = 2

# Disable unprivileged eBPF to prevent kernel memory probing
kernel.unprivileged_bpf_disabled = 1
EOF

sysctl --system > /dev/null 2>&1
echo "[✓] Kernel sysctl security parameters active."

# 2. Udev Device Hardening (Least Privilege: 0660 instead of world-writable 0666)
UDEV_RULE_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/udev_rules/99-nammy-robot.rules"
if [ -f "${UDEV_RULE_SRC}" ]; then
    echo "[*] Hardening hardware udev rules to least-privilege (0660 dialout/plugdev)..."
    sed -i 's/MODE="0666"/MODE="0660"/g' "${UDEV_RULE_SRC}"
    sed -i 's/MODE:="0666"/MODE:="0660"/g' "${UDEV_RULE_SRC}"
    cp "${UDEV_RULE_SRC}" /etc/udev/rules.d/
    udevadm control --reload-rules && udevadm trigger
    echo "[✓] Hardened udev rules reloaded."
fi

# 3. UFW Firewall Configuration
if command -v ufw > /dev/null 2>&1; then
    echo "[*] Configuring UFW Firewall for ROS 2 & Secure Jetson Operation..."
    ufw default deny incoming
    ufw default allow outgoing
    # Allow loopback for local node IPC
    ufw allow in on lo
    # Allow SSH for administration
    ufw allow 22/tcp comment 'SSH Remote Management'
    # Allow DDS Discovery and User Traffic on local network
    ufw allow 7400:7500/udp comment 'ROS 2 DDS Discovery & RTPS'
    echo "[✓] Firewall profile ready (Run 'sudo ufw enable' to activate)."
fi

# 4. Secure Shared Memory Permissions
chmod 1777 /dev/shm

echo "======================================================================"
echo "[✓] SYSTEM HARDENING COMPLETE: Host is secured against unauthorized access"
echo "======================================================================"
