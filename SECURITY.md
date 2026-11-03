# Security Policy & Hardening Specification

**Hotel Serving Robot (Nammy Robot Platform)**  
**Author & Copyright Owner:** Naman Harshwal  
**Contact:** [namanharshwal@gmail.com](mailto:namanharshwal@gmail.com)  

---

## 1. Security Architecture Overview

The Hotel Serving Robot operates in public and shared environments (hotels, restaurants, hospitals). Security is treated as a fundamental prerequisite for human safety, data privacy, and physical asset protection.

The platform implements a **Defense-in-Depth (DiD)** multi-tier security framework:

```
+-----------------------------------------------------------------------------+
|                     MULTI-TIER ROBOT SECURITY FRAMEWORK                     |
+-----------------------------------------------------------------------------+
|  Tier 1: Supply Chain & Repository Security                                 |
|    - Zero embedded secrets, credentials, or private keys in source control  |
|    - Automated static secret scanning (scripts/security_check.py)           |
|    - Strict, isolated virtual environment with pinned dependencies          |
+-----------------------------------------------------------------------------+
|  Tier 2: ROS 2 SROS2 & DDS-Security Specification                           |
|    - Mutual Authentication: X.509 PKI certificate validation per node       |
|    - Access Control: Granular permissions.xml restricting topic pub/sub     |
|    - Wire Encryption: AES-GCM (128/256-bit) cryptographic payload security  |
|    - Network Isolation: ROS_DOMAIN_ID=42 preventing cross-fleet crosstalk   |
+-----------------------------------------------------------------------------+
|  Tier 3: Host Operating System & Linux Kernel Hardening                     |
|    - UFW Firewall: Denies unauthorized incoming traffic                     |
|    - Least-Privilege Udev: Hardware devices restricted to dialout/plugdev   |
|    - Kernel Sysctl: Spoof protection, SYN cookies, and symlink protection   |
+-----------------------------------------------------------------------------+
|  Tier 4: Microcontroller & Hardware Failsafe Protection                     |
|    - Binary Frame Validation: 16-bit CCITT CRC verification (0x1021)        |
|    - Velocity Clamping & Nan/Inf rejection on all motor commands            |
|    - 300 ms Hardware Watchdog: Halts motors on communications dropout       |
|    - Autonomous Reflex Braking: < 10 ms hardware stop on obstacle breach    |
|    - LiFePO4 Undervoltage Protection: 20.0V cutoff prevents cell damage     |
+-----------------------------------------------------------------------------+
```

---

## 2. Reporting a Vulnerability

Security issues are taken with highest priority. If you identify a security vulnerability, please report it privately:

* **Primary Email:** [namanharshwal@gmail.com](mailto:namanharshwal@gmail.com)
* **Subject:** `[SECURITY VULNERABILITY] Hotel Serving Robot - <Brief Description>`
* **Please Include:**
  - Description of the vulnerability and attack vector.
  - Affected components, nodes, or firmware files.
  - Step-by-step reproduction steps or proof-of-concept.
  - Potential impact assessment.

We request you practice responsible disclosure and refrain from publicly opening issues or publishing exploits until a patch has been released.

---

## 3. SROS2 (Security for ROS 2) & DDS Security

In enterprise and public hotel deployments, standard DDS multicast traffic is unencrypted and vulnerable to packet sniffing or rogue `/cmd_vel` injection. The Nammy Robot supports full **SROS2 (OMG DDS-Security)**:

### Enabling SROS2 Cryptographic Protection
Run the automated keystore and permissions generator:
```bash
source /home/comppoint/Documents/nammy_ws/activate_nammy.sh
bash /home/comppoint/Documents/nammy_ws/scripts/setup_sros2_security.sh
```

To enforce encrypted, authenticated communication across all ROS 2 nodes:
```bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/home/comppoint/Documents/nammy_ws/security/keystore
```

### Security Capabilities Enforced by SROS2:
1. **Authentication (PKI):** Each node has an X.509 certificate signed by the Robot CA. Unauthenticated nodes cannot join the ROS 2 graph.
2. **Access Control:** A rogue node cannot publish to `/cmd_vel` or subscribe to sensor topics unless explicitly authorized by the signed `permissions.xml`.
3. **Cryptographic Encryption:** All DDS payloads over Ethernet/Wi-Fi are encrypted with AES-GCM, preventing eavesdropping.

---

## 4. Hardware & Firmware Security

### 4.1 Packet Protocol Sanitization & CRC-16 Verification
All serial communication between the Jetson and ESP32-S3 utilizes binary frames protected with CRC-16 CCITT:
* Polynomial: `0x1021`, Initial Seed: `0xFFFF`.
* Corrupted frames, framing errors, buffer overflows, or unauthorized message IDs are instantly dropped and logged.

### 4.2 Velocity Bounds & Sanity Validation
To protect against malicious or accidental extreme velocity commands:
* Linear velocity ($v_x$) is strictly clamped within $[-1.2, +1.2]\text{ m/s}$.
* Angular yaw rate ($\omega_z$) is strictly clamped within $[-2.5, +2.5]\text{ rad/s}$.
* Any command containing `NaN` or `Infinity` is immediately rejected and replaced with an active emergency stop.

### 4.3 Deterministic Watchdog & Reflex Braking
* **Communications Watchdog (300 ms):** The ESP32 motor driver automatically engages dynamic brakes if no valid command packet is received within 300 ms.
* **Autonomous Reflex Braking (< 10 ms):** The ESP32 evaluates ultrasonic bumper distance independent of the host SBC. Obstacles within 15 cm trigger a direct hardware stop.

---

## 5. Host Linux & Network Hardening

Execute the system hardening script to enforce host security:
```bash
sudo bash /home/comppoint/Documents/nammy_ws/scripts/harden_system_security.sh
```
This script:
1. Sets up UFW rules allowing only necessary SSH and DDS ports from trusted subnets.
2. Tightens udev permissions so serial devices are accessible only to the `dialout` group.
3. Applies Linux kernel sysctl network hardening parameters.
