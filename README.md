# Nammy Robot - Modular Autonomous Mobile Robot Platform

**Project Architect & Maintainer:** Naman Harshwal  
**Organization:** COMP-POINT PVT. LTD. India  
**Official Contact:** [namanharshwal@gmail.com](mailto:namanharshwal@gmail.com)  
**Target SBC:** NVIDIA Jetson Orin Nano Super Developer Kit (8GB RAM, NVMe SSD)  
**Real-Time Mediator:** ESP32-S3-DevKitC-1 (Xtensa 32-bit Dual-Core LX7 @ 240 MHz)  
**Middleware:** ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS / JetPack 6.x)  
**Network Domain Isolation:** `ROS_DOMAIN_ID=42` (Guaranteed 100% isolation from `hover_ws`)  
**Workspace Root:** `/home/comppoint/Documents/nammy_ws` (symlinked at `/home/comppoint/nammy_ws`)  
**License:** Apache License, Version 2.0  

---

## 1. Executive Summary & Engineering Architecture

The **Nammy Robot** is an industrial-grade, heavy-payload differential drive Autonomous Mobile Robot (AMR) designed, developed, and maintained by **Naman Harshwal** at **COMP-POINT PVT. LTD. India**. 

The platform employs a **two-tier compute and control architecture**:
1. **High-Level SBC Layer (NVIDIA Jetson Orin Nano Super):** Runs the full ROS 2 Humble robotics stack, SLAM mapping, Nav2 autonomous waypoint navigation, volumetric 3D voxel filtering, 10.1" Capacitive Touch HMI dashboard, BNO055 9-DOF IMU filtering, and multi-sensor Extended Kalman Filter (EKF) state estimation.
2. **Low-Level Real-Time Mediator Layer (ESP32-S3-DevKitC-1):** Connected via a shielded USB Type-C link to the Jetson. Executes deterministic 100 Hz closed-loop PID velocity control, generates 20 kHz silent ultrasonic PWM for dual SmartElex 30A motor drivers, decodes high-resolution 4x optical quadrature encoder pulses (6,000 counts/wheel rev), monitors 24V LiFePO4 battery voltage via ADC, and enforces a hardware watchdog cutoff.

```
+---------------------------------------------------------------------------------------------------------+
|                                    NAMMY ROBOT COMPLETE SYSTEM TOPOLOGY                                 |
|                                                                                                         |
|  +---------------------------------------------------------------------------------------------------+  |
|  | [HIGH-LEVEL COMPUTE SBC] NVIDIA Jetson Orin Nano Super Developer Kit                              |  |
|  |   - nammy_bringup     : Master orchestrator & centralized parameters (robot.launch.py)            |  |
|  |   - nammy_base        : USB Serial Mediator Hardware Bridge, Odometry, TF Broadcaster             |  |
|  |   - nammy_motors      : SmartElex 30S & AT120 Motor Manager, Limits, & Health Diagnostics         |  |
|  |   - nammy_imu         : Bosch BNO055 9-DOF IMU Driver (I2C Bus 7 /dev/i2c-7 @ 50 Hz)               |  |
|  |   - nammy_power       : 24V LiFePO4 Battery Monitor, SOC Curve, Diagnostic Alerts                 |  |
|  |   - nammy_display     : 10.1" (1280x800) IPS Capacitive Touch HMI Dashboard App                   |  |
|  |   - nammy_teleop      : Keyboard & Gamepad Velocity Control Engine                                |  |
|  |   - nammy_description : 3D URDF/Xacro Kinematics Model, Joint State Publisher, RViz2 Visuals    |  |
|  |   - Isolated Python Virtual Environment (.venv) with Zero Global or hover_ws Interference         |  |
|  +---------------------------------------------------|-----------------------------------------------+  |
|                                                      | Shielded USB Type-C Serial Link                  |
|                                                      | 115200 bps Framed Binary CRC-16 Protocol         |
|  +---------------------------------------------------v-----------------------------------------------+  |
|  | [REAL-TIME MOTION MEDIATOR] ESP32-S3-DevKitC-1 Wireless Module (Dual-Core 240 MHz)               |  |
|  |   - 100 Hz Dual Closed-Loop Velocity PID + Feedforward Loops                                       |  |
|  |   - 20 kHz Silent Ultrasonic PWM via Hardware LEDC Timers (0–1023 Duty)                            |  |
|  |   - 4x Optical Quadrature Decoding (6,000 ticks/wheel rev -> 0.05236 mm/tick resolution)          |  |
|  |   - 300 ms Hardware Watchdog (Auto-Brakes Motors on Comm Loss)                                     |  |
|  |   - 12-Bit ADC Battery Voltage Sampling with Exponential Moving Average Filter                      |  |
|  +-------------------|-------------------------------------------------------|-----------------------+  |
|                      | 20 kHz PWM & DIR Logic                                | Incremental Enc A/B   |
|  +-------------------v-------------------+       +---------------------------v-----------------------+  |
|  | 2x SmartElex 30S Motor Drivers (30A)  |       | 2x AT120 Otis PAX DC Gear Motors (24V DC)          |  |
|  |   - Left Driver : GPIO 4 (PWM), 5(DIR)|       |   - 15:1 Integrated Planetary Gearbox                 |  |
|  |   - Right Driver: GPIO 6 (PWM), 7(DIR)|       |   - 100-Pulse/Rev Optical Shaft Encoders              |  |
|  +-------------------|-------------------+       +---------------------------|-----------------------+  |
|                      +------------------------->-----------------------------+                          |
|                                                                                                         |
|  +---------------------------------------------------------------------------------------------------+  |
|  | [POWER & HMI SUBSYSTEMS]                                                                          |  |
|  |   - Battery: 24V ~15Ah LiFePO4 (LFP) 8S Pack (380Wh, 32140 Cells, 20A TDT BMS)                    |  |
|  |   - Wheels:  2x Heavy Duty HD Disc Wheels 100mm Dia (r = 0.050 m, Track B = 0.380 m)              |  |
|  |   - Display: 10.1" IPS Capacitive Touch 1280x800 HDMI Display (ThinkRobotics WVSH0067)            |  |
|  |   - IMU:     Bosch BNO055 9-DOF Absolute Orientation Sensor (Jetson I2C Bus 7, Pins 27 & 28)      |  |
|  +---------------------------------------------------------------------------------------------------+  |
+---------------------------------------------------------------------------------------------------------+
```

---

## 2. Component Bill of Materials (BOM) & Direct Sources

| Component | Technical Specifications & Performance Ratings | Direct Procurement Link |
| :--- | :--- | :--- |
| **Motor Drivers (2x)** | **SmartElex 30S Single Channel 30A DC Motor Driver**<br>• Input Voltage: 7V to 28V DC (24V Nominal)<br>• Current: 30A Continuous, 85A Peak (2 seconds)<br>• Control: 3.3V/5V TTL PWM & Direction Input<br>• Protection: Overcurrent, Under-Voltage Lockout (UVLO), Status LEDs | [Robocraze SmartElex 30S Driver](https://robocraze.com/products/smartelex-30s-single-channel-30a-dc-motor-driver?variant=48113148231904) |
| **Battery Pack** | **24V ~15Ah LiFePO4 (LFP) 8S Battery Pack (380Wh)**<br>• Chemistry: Lithium Iron Phosphate (32140-3.2V 15Ah A-Grade Cells)<br>• Voltage: 25.6V Nominal, 29.2V Full Charge, 20.0V Cutoff<br>• BMS: Integrated 20A TDT BMS with balance charging<br>• Endurance: 2,000+ Full Discharge Cycles | [Amazon India 24V 15Ah LFP Battery](https://amzn.in/d/09jY1yoz) |
| **Touch Display** | **10.1" Capacitive Touch Display 1280x800 IPS HDMI**<br>• SKU: WVSH0067 (Waveshare / ThinkRobotics)<br>• Active Area: 1280 × 800 pixels (16:10 aspect ratio), 178° IPS<br>• Interface: Standard HDMI Video + MicroUSB Multi-Touch Capacitive HID<br>• Power: 5V DC via MicroUSB port | [ThinkRobotics 10.1" Touch Display](https://tkro.in/7bnxz) |
| **Drive Wheels (2x)** | **Heavy Duty HD Disc Wheel 100mm Diameter**<br>• Diameter: 100 mm ($r = 0.050\text{ m}$), Width: 35 mm<br>• Tread: High-traction, shock-absorbing polyurethane solid rubber<br>• Core: Reinforced HD disc hub with standard motor shaft adapter | [Robu.in 100mm HD Disc Wheel](https://robu.in/product/heavy-dutyhd-disc-wheel-100mm-dia-1pcgrey-color) |
| **Microcontroller** | **ESP32-S3-DevKitC-1 Wireless Development Board**<br>• MCU: ESP32-S3 (Xtensa 32-bit Dual-Core LX7 @ 240 MHz, 512KB SRAM, 8MB Flash)<br>• Interfaces: Dual Type-C USB (UART + Native USB OTG CDC), Hardware LEDC PWM, Pulse Counter<br>• Wireless: 2.4 GHz Wi-Fi + Bluetooth 5 (LE) | [Probots ESP32-S3 DevKitC-1](https://probots.co.in/esp32-s3-devkitc-1-development-board-wireless-module-wifi-bluetooth.html) |
| **DC Gear Motors (2x)** | **AT120 Otis PAX DC Gear Motors with Encoder (KM949299)**<br>• Voltage: 24V DC Rated<br>• Gear Transmission: 15:1 Integrated Planetary Reduction<br>• Speed: ~200 RPM rated output ($\approx 20.94\text{ rad/s}$)<br>• Torque: ~1.05–1.08 N·m continuous, heavy peak torque<br>• Feedback: Dual-channel optical incremental encoder (100 pulses/rev) | [Kone / Otis AT120 Motor (KM949299)](https://share.google/D32h0ckD8dXbc6RLv) |
| **9-DOF IMU Sensor** | **Bosch BNO055 9-Axis Absolute Orientation Sensor**<br>• Output: Triaxial 14-bit Accelerometer, 16-bit Gyroscope, Geomagnetic Field Sensor<br>• Onboard Fusion: BSX3.0 FusionLib Cortex-M0 MCU outputting absolute Quaternions<br>• Interface: I2C (Address: 0x28 or 0x29) with Clock Stretching | Standard Bosch BNO055 breakout |

---

## 3. Deep Dive: BNO055 Connection Analysis (Jetson GPIO vs. ESP32-S3)

A fundamental architectural question is: **Where should the BNO055 9-DOF IMU be connected?**

### Architectural Comparison Matrix

| Technical Metric | Option A: Jetson 40-Pin I2C (Recommended) | Option B: ESP32-S3 I2C Bus | Winner |
| :--- | :--- | :--- | :--- |
| **Sensor Fusion & EKF Integration** | **Direct:** Publishes `/imu/data` directly into ROS 2 `robot_localization` (EKF) and Nav2 without latency or serial packet overhead. | **Indirect:** Must bundle orientation into serial telemetry packets, decode on Jetson, and re-publish. | 🏆 **Jetson GPIO** |
| **Microcontroller Motor Loop Jitter** | **Zero Impact:** The ESP32 100 Hz PID loop is dedicated purely to wheel encoders and SmartElex PWM timing. | **Risk:** BNO055 I2C reads take 1–2 ms due to sensor clock stretching, which can induce jitter in motor PID timing. | 🏆 **Jetson GPIO** |
| **EMI Noise Isolation** | **Protected:** Sensor wiring stays in the low-power logic bay away from high-current motor wires. | **Vulnerable:** I2C lines near motor drivers can experience bus lockups from motor inductive spikes. | 🏆 **Jetson GPIO** |
| **Publishing Rate & Bandwidth** | Up to 100 Hz raw IMU rate directly on the Tegra SoC I2C bus. | Limited by USB serial bandwidth and packet framing. | 🏆 **Jetson GPIO** |
| **Wiring Simplicity** | 4 wires to Jetson 40-pin header (Pins 1, 6, 27, 28). | 4 wires to ESP32 header pins (3.3V, GND, GPIO 8, 9). | 🤝 **Tie** |

### Definitive Recommendation
* **Primary (Recommended):** Connect the BNO055 to **Jetson 40-Pin Header I2C Bus 7 (Pins 27 & 28)**. This feeds cleanly into our [`nammy_imu`](src/nammy_imu) ROS 2 package, directly powering Nav2 EKF odometry fusion.
* **Secondary (Firmware Supported):** If you prefer keeping all sensors on the microcontroller, connect BNO055 to ESP32 `GPIO 8` (SDA) and `GPIO 9` (SCL).

---

## 4. Complete Wiring Schematic & Pin-to-Pin Interconnection Matrix

### 4.1 Master System Wiring Diagram

```
+--------------------------------------------------------------------------------------------------------------------+
|                                         NAMMY ROBOT COMPLETE WIRING SCHEMATIC                                      |
+--------------------------------------------------------------------------------------------------------------------+

                                      +---------------------------------------------+
                                      | 24V 15Ah LiFePO4 (LFP) 8S Battery Pack      |
                                      | (25.6V Nom, 29.2V Full, 20A TDT BMS)        |
                                      +----------------------|----------------------+
                                                             |
                                            [ 20A Main Fuse & Kill Switch ]
                                                             |
                      +--------------------------------------+--------------------------------------+
                      | +24V Raw High-Current Bus                   | GND Common Ground Bus        |
                      |                                             |                              |
                      v                                             v                              |
    +------------------------------------+        +------------------------------------+           |
    | SmartElex 30S (Left Driver)        |        | SmartElex 30S (Right Driver)       |           |
    |   B+ : +24V Battery                |        |   B+ : +24V Battery                |           |
    |   B- : Ground (GND)                |        |   B- : Ground (GND)                |           |
    |   M1 : Left Motor Armature (+)     |        |   M1 : Right Motor Armature (+)    |           |
    |   M2 : Left Motor Armature (-)     |        |   M2 : Right Motor Armature (-)    |           |
    |   PWM: <-- ESP32-S3 GPIO 4         |        |   PWM: <-- ESP32-S3 GPIO 6         |           |
    |   DIR: <-- ESP32-S3 GPIO 5         |        |   DIR: <-- ESP32-S3 GPIO 7         |           |
    |   GND: --- Common Logic GND        |        |   GND: --- Common Logic GND        |           |
    +------------------------------------+        +------------------------------------+           |
                      |                                             |                              |
                      +----------------------+----------------------+                              |
                                             |                                                     |
                      +----------------------+----------------------+                              |
                      |                                             |                              |
                      v                                             v                              |
    +------------------------------------+        +------------------------------------+           |
    | DC-DC Step-Down Buck (24V -> 19V)  |        | DC-DC Step-Down Buck (24V -> 5.1V) |           |
    | Input : 24V Battery                |        | Input : 24V Battery                |           |
    | Output: 19V @ 5A (95W Regulated)   |        | Output: 5.1V @ 5A (25W Regulated)  |           |
    +-----------------|------------------+        +-----------------|------------------+           |
                      |                                             |                              |
                      v DC Barrel Jack (5.5x2.5mm)                  +---------------+              |
    +------------------------------------+                          |               |              |
    | NVIDIA Jetson Orin Nano Super SBC  |                          v               v              |
    |   DC Jack: 19V Main Power Input    |           [10.1" Touch Display]   [Sensors/Fans]        |
    |   USB 3.2: 5V VBUS & Data Link ----+-----------> MicroUSB 5V Power     5V Rail               |
    |   HDMI   : Video Output 1280x800 --+-----------> HDMI Video Input                            |
    |   40-Pin Header I2C Bus 7:         |                                                         |
    |     - Pin 1 : 3.3V Power Out ------+---------------------------------------------+            |
    |     - Pin 6 : GND Ground ----------+---------------------------------------+     |            |
    |     - Pin 27: I2C-7 SDA -----------+---------------------------------+     |     |            |
    |     - Pin 28: I2C-7 SCL -----------+---------------------------+     |     |     |            |
    +------------------------------------+                           |     |     |     |            |
                                                                     |     |     |     |            |
                                                                     v     v     v     v            |
                                                  +------------------------------------+            |
                                                  | Bosch BNO055 9-DOF IMU Breakout    |            |
                                                  |   Vin: 3.3V (Jetson Pin 1)         |            |
                                                  |   GND: Ground (Jetson Pin 6)       |            |
                                                  |   SDA: I2C-7 SDA (Jetson Pin 27)   |            |
                                                  |   SCL: I2C-7 SCL (Jetson Pin 28)   |            |
                                                  +------------------------------------+            |
                                                                                                    |
                      +-----------------------------------------------------------------------------+
                      | Shielded Type-A to Type-C USB Cable
                      v
    +------------------------------------+
    | ESP32-S3-DevKitC-1 Microcontroller |
    |   Type-C "UART": USB to Jetson     | (Powers ESP32 logic & provides low-latency serial link)
    |   GPIO 4 : PWM Left Driver         |
    |   GPIO 5 : DIR Left Driver         |
    |   GPIO 6 : PWM Right Driver        |
    |   GPIO 7 : DIR Right Driver        |
    |   GPIO 15: Left Encoder A (ISR)    |<--- Left AT120 Optical Encoder A
    |   GPIO 16: Left Encoder B (ISR)    |<--- Left AT120 Optical Encoder B
    |   GPIO 17: Right Encoder A (ISR)   |<--- Right AT120 Optical Encoder A
    |   GPIO 18: Right Encoder B (ISR)   |<--- Right AT120 Optical Encoder B
    |   GPIO 1 : Battery Voltage ADC     |<--- 100k/10k Resistor Voltage Divider from +24V Bus
    |   GND    : Common Logic Ground     |----+
    +------------------------------------+    |
                                              v
                             [ Linked to All Driver & Sensor Grounds ]
```

---

### 4.2 Comprehensive Pin-by-Pin Interconnection Matrix

#### Group 1: Jetson Orin Nano 40-Pin Header to Bosch BNO055 IMU
| Jetson 40-Pin Pin Number | Jetson Header Label | BNO055 Breakout Pin | Signal Description |
| :--- | :--- | :--- | :--- |
| **Pin 1** | `3.3V PWR` | `VIN` / `VCC` | 3.3V Clean Regulated Power |
| **Pin 6** | `GND` | `GND` | Ground Reference |
| **Pin 27** | `GEN2_I2C_SDA` (`/dev/i2c-7`) | `SDA` | I2C Bidirectional Data Line |
| **Pin 28** | `GEN2_I2C_SCL` (`/dev/i2c-7`) | `SCL` | I2C Clock Line (Supports Clock Stretching) |

#### Group 2: ESP32-S3 to Dual SmartElex 30S Motor Drivers
| ESP32-S3 Pin | Physical Pin Function | SmartElex 30S Terminal | Signal Characteristics |
| :--- | :--- | :--- | :--- |
| **GPIO 4** | LEDC Timer Channel 0 | **Left Driver: PWM** | 20 kHz Ultrasonic PWM (Duty: 0–1023) |
| **GPIO 5** | Digital Output | **Left Driver: DIR** | HIGH = Forward, LOW = Reverse |
| **GPIO 6** | LEDC Timer Channel 1 | **Right Driver: PWM** | 20 kHz Ultrasonic PWM (Duty: 0–1023) |
| **GPIO 7** | Digital Output | **Right Driver: DIR** | HIGH = Forward, LOW = Reverse |
| **GND** | Digital Logic Ground | **Driver Logic GND** | Common Ground Return |

#### Group 3: ESP32-S3 to AT120 Otis Motor Encoders
| ESP32-S3 Pin | Interrupt Configuration | Motor Encoder Wire | Resolution / Specs |
| :--- | :--- | :--- | :--- |
| **GPIO 15** | Hardware ISR (`CHANGE`) | **Left Motor: Channel A** | Optical Square Wave (100 pulses/rev) |
| **GPIO 16** | Hardware ISR (`CHANGE`) | **Left Motor: Channel B** | 90° Phase-Shifted Quadrature Channel |
| **GPIO 17** | Hardware ISR (`CHANGE`) | **Right Motor: Channel A**| Optical Square Wave (100 pulses/rev) |
| **GPIO 18** | Hardware ISR (`CHANGE`) | **Right Motor: Channel B**| 90° Phase-Shifted Quadrature Channel |
| **VBUS / 5V** | 5V Regulated | **Encoder VCC** | Encoder Sensor Power Supply |
| **GND** | Logic Ground | **Encoder GND** | Sensor Return Ground |

#### Group 4: 24V LiFePO4 Voltage Divider to ESP32 ADC
| Battery Rail | Resistor Configuration | ESP32-S3 Pin | ADC Voltage Range |
| :--- | :--- | :--- | :--- |
| **+24V Battery (+)** | $R_1 = 100\text{ k}\Omega$ (1% precision) | — | Unregulated 20.0V–29.2V |
| **Divider Node** | Node between $R_1$ and $R_2$ | **GPIO 1 (ADC1 CH0)** | Scaled: 1.81V to 2.65V (< 3.3V Limit) |
| **Battery GND (-)** | $R_2 = 10\text{ k}\Omega$ (1% precision) to GND | **GND** | Ground Reference |

#### Group 5: ESP32-S3 to TXS0108E 8-Channel Level Shifter to 4x US-100 Ultrasonic Array
> [!IMPORTANT]
> **US-100 Mode Selection:** The jumper on the back of all 4 US-100 sensors **MUST BE REMOVED (OPEN)** to select standard **Trigger/Echo Mode**!

| Sensor Location | Function | ESP32-S3 Pin (3.3V) | TXS0108E Side A (3.3V) | TXS0108E Side B (5.0V) | US-100 Pin (5.0V) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Level Shifter Power**| Power Supply | **3V3 Pin** | **VCCA** (3.3V Input) | **VCCB** (5.0V Input from BEC)| **VCC** (All 4 sensors) |
| **Level Shifter Enable**| Output Enable | **3V3 Pin** | **OE** (Active HIGH)  | — | — |
| **Common Ground** | Ground Return | **GND Pin** | **GND** | **GND** | **GND** (All 4 sensors) |
| **1. Front Sonar** | Trigger Pulse | **GPIO 8** | **A1** | **B1** | `Trig/TX` Pin |
| **1. Front Sonar** | Echo Return   | **GPIO 9** | **A2** | **B2** | `Echo/RX` Pin |
| **2. Left Sonar**  | Trigger Pulse | **GPIO 10**| **A3** | **B3** | `Trig/TX` Pin |
| **2. Left Sonar**  | Echo Return   | **GPIO 11**| **A4** | **B4** | `Echo/RX` Pin |
| **3. Right Sonar** | Trigger Pulse | **GPIO 12**| **A5** | **B5** | `Trig/TX` Pin |
| **3. Right Sonar** | Echo Return   | **GPIO 13**| **A6** | **B6** | `Echo/RX` Pin |
| **4. Rear Sonar**  | Trigger Pulse | **GPIO 14**| **A7** | **B7** | `Trig/TX` Pin |
| **4. Rear Sonar**  | Echo Return   | **GPIO 21**| **A8** | **B8** | `Echo/RX` Pin |

#### Group 6: YDLIDAR G2 360-Degree 2D Laser Scanner to Jetson Orin Nano
| YDLIDAR G2 Cable / Port | Connection Destination | Protocol / Parameters |
| :--- | :--- | :--- |
| **USB Type-C Cable** | **Jetson USB 3.0 Port** | USB Serial via CP2102 Bridge (`/dev/ydlidar` @ 230400 bps) |
| **Power Requirements** | 5V DC via USB Bus (or external 5V 2A BEC)| 5V ± 5%, Operating current ~450 mA |

---

### 4.3 Architectural Analysis: Sensor Placement (Jetson vs. ESP32-S3)

#### Why 4x Ultrasonic Sensors Belong on the ESP32-S3 (and NOT directly on Jetson Linux):
1. **Microsecond Precision vs. Linux Kernel Preemption:**
   Ultrasonic measurement requires timing acoustic echoes with microsecond resolution ($v = 343\text{ m/s} \implies 58.2\ \mu\text{s/cm}$). On a non-real-time Linux OS like Ubuntu on the Jetson, CPU context switches, GPU load spikes, and thread preemption introduce random multi-millisecond timing delays ($1\text{ ms} = 17.1\text{ cm}$ false error!). The ESP32-S3 runs FreeRTOS with microsecond hardware timers (`micros()`) delivering sub-millimeter ranging precision.
2. **Crosstalk-Free Sequential Round-Robin Firing:**
   If all 4 ultrasonic sensors fire simultaneously, ultrasonic sound waves reflect off diagonal walls and trigger false echoes on adjacent sensors. The ESP32-S3 runs a deterministic 35ms round-robin scheduler (Front $\rightarrow$ Left $\rightarrow$ Right $\rightarrow$ Rear), completely eliminating acoustic interference.
3. **Autonomous Hardware Reflex Braking (Fail-Safe Collision Prevention):**
   If an obstacle or transparent glass barrier suddenly appears within 15 cm of the robot, the ESP32-S3 executes an **instantaneous hardware stop in < 10 ms**, directly halting the SmartElex 30S drivers even if the Jetson OS freezes, throttles, or ROS 2 navigation crashes.

#### Why YDLIDAR G2 Belongs on the Jetson Orin Nano:
* The YDLIDAR G2 produces continuous high-bandwidth 2D point cloud streams (9,000 points/second at 230,400 baud, 10 Hz 360° scans).
* Processing these streams requires the Jetson's 6-core ARM Cortex CPU and Ampere GPU for Cartographer SLAM, Slam Toolbox, Nav2 Costmap2D obstacle layers, and voxel grid filtering.

#### Why BNO055 IMU Belongs on Jetson I2C Bus 7:
* Connecting the BNO055 directly to Jetson I2C Bus 7 (`/dev/i2c-7`, Pins 27/28) allows `robot_localization` EKF to fuse IMU angular velocities and orientations with zero serial transport latency.
* It completely protects the ESP32-S3's 100 Hz PID motor loop from the BNO055's known ~450 $\mu$s I2C clock-stretching delays.

#### Complementary LiDAR + Sonar Fusion (Glass & Low Obstacle Protection):
| Characteristic | YDLIDAR G2 (2D Planar Laser) | 4x US-100 Ultrasonic Array | Fused Robot Behavior |
| :--- | :--- | :--- | :--- |
| **Operating Medium** | Infrared Light (905 nm) | Ultrasonic Sound (40 kHz) | Multi-physics sensor diversity |
| **Clear Glass & Mirrors**| **BLIND** (Laser passes through glass) | **100% REFLECTED** (Sound bounces off glass) | Robot detects glass doors & windows |
| **Low Obstacles (< 12 cm)**| **BLIND** (Scans horizontal plane above) | **DETECTED** (Cones cover floor to bumper) | Protects against toys, cables, step drops |
| **Table & Chair Overhangs**| **BLIND** (Scans under tabletop) | **DETECTED** (Upper cone edge strikes table) | Prevents wedging robot under furniture |
| **Range & Angular FOV** | 360° continuous, 0.12m – 12.0m | 15° cone per sensor, 0.02m – 4.5m | Sonars fill blind spots; LiDAR maps room |

---

## 5. Mathematical Kinematics & Sub-Millimeter Odometry

### 5.1 Physical Drive Parameters
* **Wheel Diameter ($D$):** $100\text{ mm} = 0.100\text{ m}$
* **Wheel Radius ($r$):** $r = 0.050\text{ m}$
* **Wheel Circumference ($C$):** $C = 2\pi r = \pi \times 0.100\text{ m} \approx 0.314159\text{ m}$
* **Track Width / Separation ($B$):** $B = 0.380\text{ m}$ (distance between left and right wheel contact patches)
* **Gearbox Reduction Ratio ($G$):** $15:1$ integrated planetary transmission
* **Motor Optical Encoder:** 100 pulses per motor shaft revolution

### 5.2 Quadrature Count Resolution
When decoding both rising and falling edges on Channels A and B (4x quadrature decoding):
$$N_{\text{motor\_quad}} = 100 \times 4 = 400\text{ counts/motor rev}$$
Accounting for the $15:1$ gearbox reduction:
$$N_{\text{wheel\_quad}} = 400 \times 15 = 6,000\text{ counts/wheel revolution}$$

The linear ground displacement per encoder tick is:
$$\Delta d_{\text{tick}} = \frac{C}{N_{\text{wheel\_quad}}} = \frac{0.314159\text{ m}}{6,000\text{ ticks}} = 0.00005236\text{ m/tick} = \mathbf{52.36\ \mu m/\text{tick}}$$
Every encoder tick translates to **$52.36\text{ }\mu\text{m}$**, providing sub-millimeter dead-reckoning odometry.

### 5.3 Velocity & Kinematic Conversions
At rated 24V load, the AT120 Otis DC motor delivers ~200 RPM at the output shaft:
$$\omega_{\max} = 200 \times \frac{2\pi}{60} \approx 20.944\text{ rad/s}$$
$$v_{\max} = \omega_{\max} \cdot r = 20.944\text{ rad/s} \times 0.050\text{ m} \approx \mathbf{1.047\text{ m/s}}\ (\approx 3.77\text{ km/h})$$

#### Inverse Kinematics (Cmd_Vel to Wheel Setpoints):
Given desired robot linear velocity $v_x$ (m/s) and angular velocity $\omega_z$ (rad/s):
$$\omega_{\text{left}} = \frac{v_x - \left(\omega_z \cdot \frac{B}{2}\right)}{r}, \qquad \omega_{\text{right}} = \frac{v_x + \left(\omega_z \cdot \frac{B}{2}\right)}{r}$$

#### Forward Kinematics (Encoder Feedback to Pose Update):
From left and right wheel encoder displacements $\Delta s_L = \Delta \theta_L \cdot r$ and $\Delta s_R = \Delta \theta_R \cdot r$:
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \qquad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{B}$$
$$x_{k+1} = x_k + \Delta s \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$y_{k+1} = y_k + \Delta s \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$\theta_{k+1} = \theta_k + \Delta \theta$$

---

## 6. The 8 Modular ROS 2 Component Packages Explained

The workspace is organized into **8 cleanly separated, independent packages**:

```
nammy_ws/src/
├── nammy_base/          # [1] Hardware bridge, USB serial mediator interface & odometry
├── nammy_motors/        # [2] SmartElex 30S driver manager & AT120 motor health diagnostics
├── nammy_imu/           # [3] Bosch BNO055 9-DOF IMU driver (I2C-7) & /imu/data publisher
├── nammy_power/         # [4] 24V LiFePO4 battery monitor, SOC curve & safety alerts
├── nammy_display/       # [5] 10.1" (1280x800) capacitive touch HMI dashboard app
├── nammy_teleop/        # [6] Keyboard and gamepad teleoperation velocity driver
├── nammy_description/   # [7] 3D URDF/Xacro model, joints, sensor links & RViz2 visuals
├── nammy_bringup/       # [8] Master umbrella orchestrator (robot.launch.py, params.yaml)
├── nammy_sonar/         # [9] 4x US-100 Sonar Array & Costmap PointCloud2 Fusion
└── nammy_lidar/         # [10] YDLIDAR G2 360-Degree Laser Scanner Bringup & Sim Fallback
```

### Detailed Package Responsibilities:

1. **[`nammy_base`](src/nammy_base):**
   * Node: `esp32_hardware_bridge`
   * Subscribes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
   * Publishes: `/odom` (`nav_msgs/msg/Odometry`), `/joint_states` (`sensor_msgs/msg/JointState`), `/nammy/battery_state` (`sensor_msgs/msg/BatteryState`), `/sonar/front`, `/sonar/left`, `/sonar/right`, `/sonar/rear` (`sensor_msgs/msg/Range`)
   * Broadcasts TF: `odom -> base_footprint`
   * Features: Automatic USB hotplug re-connection and simulation fallback.

2. **[`nammy_motors`](src/nammy_motors):**
   * Node: `motor_manager`
   * Publishes: `/nammy/motors/status` (`std_msgs/msg/String`), `/nammy/motors/limits` (`std_msgs/msg/Float32MultiArray`)
   * Features: Enforces speed scaling, monitors current limits (30A/85A), and prevents gearbox shock.

3. **[`nammy_imu`](src/nammy_imu):**
   * Node: `bno055_node`
   * Interfaces: Jetson 40-Pin Header `/dev/i2c-7` (Address: `0x28`)
   * Publishes: `/imu/data` (`sensor_msgs/msg/Imu` with orientation quaternion and covariances), `/imu/mag`, `/imu/calibration`
   * Features: Direct input to `robot_localization` EKF.

4. **[`nammy_power`](src/nammy_power):**
   * Node: `battery_monitor_node`
   * Subscribes: `/nammy/battery_state`
   * Publishes: `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`), `/nammy/power/alert` (`std_msgs/msg/String`)
   * Features: Calculates per-cell voltages ($V_{\text{pack}}/8$), 8S LiFePO4 SOC percentage, and issues critical cutoff alerts below 20.0V.

5. **[`nammy_display`](src/nammy_display):**
   * Node: `touch_hmi_app`
   * Framework: High-contrast dark-themed 1280×800 touch GUI for Waveshare 10.1" IPS HDMI display.
   * Features: Real-time LiFePO4 battery gauge bar, live linear/angular velocity gauges, large capacitive touch D-pad, and high-visibility Emergency Stop.

6. **[`nammy_teleop`](src/nammy_teleop):**
   * Node: `teleop_keyboard`
   * Features: Continuous keyboard velocity driving (`w/a/s/d`), speed increments (`q/z/e/c`), and spacebar brake.

7. **[`nammy_description`](src/nammy_description):**
   * URDF/Xacro: [`urdf/nammy_robot.urdf.xacro`](src/nammy_description/urdf/nammy_robot.urdf.xacro)
   * Elements: 100mm HD Disc wheels, 10.1" tilted touch console, 24V LiFePO4 pack, SmartElex 30S dual drivers, ESP32-S3, YDLIDAR G2 optical assembly, BNO055 `imu_link`, and 4x US-100 sonar sensor links (`front_sonar_link`, `left_sonar_link`, `right_sonar_link`, `rear_sonar_link`).

8. **[`nammy_bringup`](src/nammy_bringup):**
   * Master Launch: [`launch/robot.launch.py`](src/nammy_bringup/launch/robot.launch.py)
   * Dashboard Launch: [`launch/dashboard.launch.py`](src/nammy_bringup/launch/dashboard.launch.py)
   * Parameters: Centralized [`config/robot_params.yaml`](src/nammy_bringup/config/robot_params.yaml).

9. **[`nammy_sonar`](src/nammy_sonar):**
   * Node: `sonar_fusion_node`
   * Subscribes: `/sonar/front`, `/sonar/left`, `/sonar/right`, `/sonar/rear` (`sensor_msgs/msg/Range`)
   * Publishes: `/sonar/obstacle_cloud` (`sensor_msgs/msg/PointCloud2`), `/nammy/sonar_safety_stop` (`std_msgs/msg/Bool`)
   * Features: Projects 15-degree ultrasonic detection cones into 3D Cartesian coordinates in `base_link` frame for Nav2 Costmap2D integration, detecting glass, table edges, and floor drops.

10. **[`nammy_lidar`](src/nammy_lidar):**
    * Driver: System `ydlidar_ros2_driver_node` (Config: [`config/ydlidar_g2_params.yaml`](src/nammy_lidar/config/ydlidar_g2_params.yaml))
    * Node: `lidar_sim_fallback` (Runs 360° synthetic scan when hardware is unplugged)
    * Publishes: `/scan` (`sensor_msgs/msg/LaserScan` @ 10 Hz, 0.12m – 12.0m range)
    * Port: `/dev/ydlidar` (Baud: 230400 bps)

---

## 7. Step-by-Step Installation, Flashing & Operational Guide

### Step 1: Activate the Isolated Environment
Open a terminal on the Jetson and source the workspace activation script:
```bash
source /home/comppoint/Documents/nammy_ws/activate_nammy.sh
```
*Sets up `ROS_DOMAIN_ID=42` to guarantee zero crosstalk with Ruby Robot (`hover_ws`), activates the `.venv`, and adds convenient developer aliases.*

### Step 2: Install Hardware Udev Rules (One-Time Setup)
```bash
sudo /home/comppoint/Documents/nammy_ws/udev_rules/install_rules.sh
```
*Creates persistent symlinks `/dev/nammy_esp32` and `/dev/input/nammy_touchscreen` with `latency_timer = 1 ms` and non-root access permissions.*

### Step 3: Compile the Entire Workspace
```bash
cd /home/comppoint/Documents/nammy_ws
colcon build --symlink-install
source install/setup.bash
```
*(Or simply run the shortcut alias: `nammy_build`).*

### Step 4: Flash the ESP32-S3 Mediator Firmware
Connect the ESP32-S3 to the Jetson via USB and run the flasher:
```bash
/home/comppoint/Documents/nammy_ws/scripts/flash_esp32.sh
```
*(PlatformIO in `.venv` will automatically compile and flash the firmware).*

### Step 5: Run Pre-Flight Hardware Diagnostics
Verify serial communication, encoder counts, closed-loop speed, and 24V battery ADC:
```bash
python3 /home/comppoint/Documents/nammy_ws/scripts/esp32_mediator_test.py
```

### Step 6: Launch the Complete Robot Stack
In your primary terminal:
```bash
source /home/comppoint/Documents/nammy_ws/activate_nammy.sh
ros2 launch nammy_bringup robot.launch.py
```
*(To test without physical hardware connected, append `simulate_hardware:=true`).*

### Step 7: Launch the 10.1" Touch HMI Dashboard
On the robot's physical display:
```bash
source /home/comppoint/Documents/nammy_ws/activate_nammy.sh
ros2 launch nammy_bringup dashboard.launch.py
```

### Step 8: Teleoperate the Robot
In a separate terminal or via SSH:
```bash
source /home/comppoint/Documents/nammy_ws/activate_nammy.sh
ros2 run nammy_teleop teleop_keyboard
```

---

## 8. Authorship & Intellectual Property Notice

```
================================================================================
NAMMY ROBOT PLATFORM - MODULAR AUTONOMOUS ROBOTICS COMPUTING SUITE
Copyright (c) 2025-2026 Naman Harshwal, COMP-POINT PVT. LTD. India.
All rights reserved.

Architect & Maintainer: Naman Harshwal (COMP-POINT PVT. LTD. India)
Official Email: namanharshwal@gmail.com
Repository: https://github.com/namanharshwal/hover_ws-comp-point-pvt-ltd.git
License: Apache License, Version 2.0
================================================================================
```
