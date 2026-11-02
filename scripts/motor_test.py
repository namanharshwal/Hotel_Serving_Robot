#!/usr/bin/env python3
"""
Motor Spin and Direction Diagnostic Test for SmartElex 30S and AT120 Motors.
Slowly spins Left and Right motors forward and reverse to verify polarity and wiring.
"""

import sys
import os
import time
import serial

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'nammy_controller'))
from nammy_controller.packet_protocol import pack_cmd_vel, pack_estop

def main():
    print("======================================================================")
    print("NAMMY ROBOT - SMARTELEX 30S & AT120 MOTOR POLARITY TEST")
    print("CAUTION: Robot wheels should be elevated off the ground!")
    print("======================================================================")

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/nammy_esp32'
    baud = 115200

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"[✓] Connected to {port}")
    except Exception as e:
        print(f"[!] Could not connect to {port}: {e}")
        print("    Usage: python3 motor_test.py /dev/ttyUSB0")
        sys.exit(1)

    try:
        # Step 1: Spin Left Motor Forward
        print("\n1. Spinning LEFT motor forward at 2.0 rad/s (3 seconds)...")
        t_end = time.time() + 3.0
        while time.time() < t_end:
            ser.write(pack_cmd_vel(2.0, 0.0))
            time.sleep(0.05)

        # Stop
        ser.write(pack_cmd_vel(0.0, 0.0))
        time.sleep(1.0)

        # Step 2: Spin Right Motor Forward
        print("2. Spinning RIGHT motor forward at 2.0 rad/s (3 seconds)...")
        t_end = time.time() + 3.0
        while time.time() < t_end:
            ser.write(pack_cmd_vel(0.0, 2.0))
            time.sleep(0.05)

        # Stop
        ser.write(pack_cmd_vel(0.0, 0.0))
        time.sleep(1.0)

        # Step 3: Both Forward
        print("3. Spinning BOTH motors forward at 2.0 rad/s (3 seconds)...")
        t_end = time.time() + 3.0
        while time.time() < t_end:
            ser.write(pack_cmd_vel(2.0, 2.0))
            time.sleep(0.05)

        print("\n[✓] Motor test sequence completed successfully.")

    except KeyboardInterrupt:
        print("\n[!] Test interrupted by user.")
    finally:
        ser.write(pack_estop())
        ser.close()
        print("Motors stopped and serial closed.")

if __name__ == '__main__':
    main()
