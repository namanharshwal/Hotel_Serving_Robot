#!/usr/bin/env python3
"""
Stand-alone BNO055 IMU Hardware Diagnostic & Live Motion Test.
Tests physical I2C connectivity on Jetson 40-pin header (Bus 7 / Bus 1),
verifies chip identity, configures Mode 0x08 (IMU relative fusion),
and streams live Heading (Yaw), Pitch, Roll, and Quaternion data.

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
"""

import sys
import time
import struct

try:
    import smbus2
except ImportError:
    print("Error: smbus2 is not installed. Run: pip install smbus2")
    sys.exit(1)

BNO055_CHIP_ID_ADDR = 0x00
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E
BNO055_SYS_TRIGGER_ADDR = 0x3F

# Output data registers
BNO055_EUL_HEADING_LSB = 0x1A
BNO055_QUA_DATA_W_LSB  = 0x20
BNO055_GYR_DATA_X_LSB  = 0x14
BNO055_CALIB_STAT      = 0x35

# Modes
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_IMU    = 0x08  # Gyro + Accel fusion (proven hover_ws mode)

def find_bno055():
    """Scan candidate buses (1, 7) and addresses (0x28, 0x29)."""
    for bus_num in [1, 7]:
        try:
            bus = smbus2.SMBus(bus_num)
            for addr in [0x28, 0x29]:
                try:
                    chip_id = bus.read_byte_data(addr, BNO055_CHIP_ID_ADDR)
                    if chip_id == 0xA0:
                        return bus_num, addr, bus
                except Exception:
                    pass
            bus.close()
        except Exception:
            pass
    return None, None, None

def main():
    print("=" * 68)
    print("        BOSCH BNO055 9-DOF IMU DIRECT HARDWARE TEST SUITE        ")
    print("=" * 68)
    print("Primary Bus: Jetson 40-pin Header /dev/i2c-1 (Pin 3=SDA, Pin 5=SCL)")
    print("Secondary  : Jetson 40-pin Header /dev/i2c-7 (Pin 27=SDA, Pin 28=SCL)")
    print("Power      : Pin 1 (3.3V) or Pin 2/4 (5V) -> VIN | Pin 6 (GND) -> GND")
    print("-" * 68)

    bus_num, addr, bus = find_bno055()

    if not bus:
        print("[*] BNO055 not detected yet. Starting live auto-scan loop on Bus 1 & 7...")
        print("[*] Please inspect your 4 jumper wires on the 40-pin header:")
        print("    - Pin 1  (3.3V) or Pin 2 (5V) ➔ BNO055 VIN (Check if sensor LED is ON)")
        print("    - Pin 6  (GND)                ➔ BNO055 GND")
        print("    - Pin 3  (I2C1 SDA)           ➔ BNO055 SDA")
        print("    - Pin 5  (I2C1 SCL)           ➔ BNO055 SCL")
        print("\nWaiting for connection (Press Ctrl+C to stop)...")

        dots = 0
        while not bus:
            time.sleep(1.0)
            bus_num, addr, bus = find_bno055()
            dots = (dots + 1) % 4
            sys.stdout.write(f"\rScanning I2C buses [{'.' * dots:<3}]")
            sys.stdout.flush()

        print("\n")

    print(f"\n[>>> SUCCESS: BNO055 DETECTED! <<<]")
    print(f"  Port: /dev/i2c-{bus_num}")
    print(f"  Address: 0x{addr:02X}")

    try:
        chip_id = bus.read_byte_data(addr, BNO055_CHIP_ID_ADDR)
        print(f"  Chip ID: 0x{chip_id:02X} (Expected: 0xA0)")

        # Switch to CONFIG mode to set parameters
        bus.write_byte_data(addr, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
        time.sleep(0.03)

        # Set Normal Power mode
        bus.write_byte_data(addr, BNO055_PWR_MODE_ADDR, 0x00)
        time.sleep(0.02)

        # Switch to IMU mode (0x08) - relative Gyro + Accel fusion matching hover_ws
        bus.write_byte_data(addr, BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMU)
        time.sleep(0.04)

        print("  Operating Mode: IMU Relative Fusion (0x08 - Gyro + Accel)")
        print("-" * 68)
        print("LIVE REAL-TIME MOTION TELEMETRY (Rotate or tilt robot/sensor):")
        print("=" * 68)

        while True:
            try:
                # Read 6 bytes of Euler angles (Heading/Yaw, Roll, Pitch, 1 LSB = 1/16 deg)
                eul_raw = bus.read_i2c_block_data(addr, BNO055_EUL_HEADING_LSB, 6)
                yaw_raw, roll_raw, pitch_raw = struct.unpack('<hhh', bytes(eul_raw))
                yaw = yaw_raw / 16.0
                roll = roll_raw / 16.0
                pitch = pitch_raw / 16.0

                # Read 8 bytes of fused Quaternion (W, X, Y, Z, 1 LSB = 2^-14)
                quat_raw = bus.read_i2c_block_data(addr, BNO055_QUA_DATA_W_LSB, 8)
                qw, qx, qy, qz = struct.unpack('<hhhh', bytes(quat_raw))
                scale = 1.0 / 16384.0
                norm = ((qw*scale)**2 + (qx*scale)**2 + (qy*scale)**2 + (qz*scale)**2) ** 0.5
                if norm > 0:
                    qw_norm = (qw * scale) / norm
                    qz_norm = (qz * scale) / norm
                else:
                    qw_norm, qz_norm = 1.0, 0.0

                # Read Angular Velocity Gyro Z (1 LSB = 1/16 rad/s / 900.0)
                gyr_raw = bus.read_i2c_block_data(addr, BNO055_GYR_DATA_X_LSB, 6)
                gx, gy, gz = struct.unpack('<hhh', bytes(gyr_raw))
                wz = gz / 900.0

                # Read Calibration status
                calib = bus.read_byte_data(addr, BNO055_CALIB_STAT)
                sys_cal = (calib >> 6) & 0x03
                gyr_cal = (calib >> 4) & 0x03
                acc_cal = (calib >> 2) & 0x03
                mag_cal = calib & 0x03

                status_str = f"YAW: {yaw:+06.1f}° | PITCH: {pitch:+05.1f}° | ROLL: {roll:+05.1f}° | Wz: {wz:+05.2f} r/s | Quat Z: {qz_norm:+05.2f} | Cal[S:{sys_cal} G:{gyr_cal} A:{acc_cal}]"
                sys.stdout.write(f"\r{status_str}")
                sys.stdout.flush()

                time.sleep(0.04)  # 25 Hz terminal update rate

            except Exception as e:
                sys.stdout.write(f"\r[I2C Glitch: {e}] Re-polling...                        ")
                sys.stdout.flush()
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")
    finally:
        try:
            bus.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()
