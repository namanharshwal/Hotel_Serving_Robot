#!/usr/bin/env python3
"""
Interactive Hardware Test Utility for ESP32-S3 Wheel Mediator and SmartElex 30S.
Provides live encoder monitoring, closed-loop speed stepping, and PWM override tests.
"""

import sys
import os
import time
import serial
import serial.tools.list_ports

# Add nammy_base package to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'nammy_base'))
from nammy_base.packet_protocol import (
    pack_cmd_vel, pack_estop, pack_heartbeat, pack_raw_pwm, pack_pid_tune,
    PacketParser, unpack_telemetry, MSG_TELEMETRY
)

def print_menu():
    print("\n" + "=" * 65)
    print("      NAMMY ROBOT - ESP32-S3 WHEEL MEDIATOR HARDWARE CLI       ")
    print("=" * 65)
    print("  [1] Live Telemetry Monitor (Encoders, Speeds & 24V Battery)")
    print("  [2] Closed-Loop Velocity Step Test (Forward at 2.0 rad/s)")
    print("  [3] Differential Turn Test (Spin Left / Right in Place)")
    print("  [4] Open-Loop Direct PWM Calibration (-1023 to +1023)")
    print("  [5] Live PID Gain Tuning (Update Kp, Ki, Kd, Kff)")
    print("  [6] Emergency Stop (Immediate Motor Cutoff)")
    print("  [7] Live Sonar Range Stream (Front, Left, Right, Rear & Reflex Safety)")
    print("  [q] Quit")
    print("=" * 65)

def get_serial_port():
    default_port = '/dev/nammy_esp32'
    if os.path.exists(default_port):
        return default_port
    available = [p.device for p in serial.tools.list_ports.comports()]
    if available:
        return available[0]
    return default_port

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else get_serial_port()
    baud = 115200

    print(f"Connecting to ESP32-S3 mediator on {port} @ {baud} bps...")
    try:
        ser = serial.Serial(port, baud, timeout=0.08)
        print("[✓] Connected successfully to ESP32-S3!")
    except Exception as e:
        print(f"[!] Could not open port {port}: {e}")
        print("    Usage: python3 esp32_mediator_test.py [PORT]")
        sys.exit(1)

    parser = PacketParser()

    try:
        while True:
            print_menu()
            choice = input("Select an option [1-6, q]: ").strip()

            if choice == '1':
                # Live Telemetry Stream
                print("\nStreaming real-time telemetry from ESP32-S3 (Press Ctrl+C to stop)...")
                try:
                    while True:
                        ser.write(pack_heartbeat())
                        if ser.in_waiting:
                            chunk = ser.read(ser.in_waiting)
                            parser.feed(chunk)
                            while True:
                                pkt = parser.parse()
                                if pkt is None:
                                    break
                                msg_type, payload = pkt
                                if msg_type == MSG_TELEMETRY:
                                    t = unpack_telemetry(payload)
                                    if t:
                                        print(f"\r[TELEMETRY] Batt: {t.battery_volts:5.2f} V | "
                                              f"L_Pos: {t.left_pos_rad:7.2f} rad ({t.left_vel_rad_s:+5.2f} rad/s) | "
                                              f"R_Pos: {t.right_pos_rad:7.2f} rad ({t.right_vel_rad_s:+5.2f} rad/s) | "
                                              f"Flags: 0x{t.status_flags:02X}   ", end='', flush=True)
                        time.sleep(0.02)
                except KeyboardInterrupt:
                    print("\n[Stopped]")

            elif choice == '2':
                # Velocity Step Test
                speed = float(input("Enter target speed in rad/s (e.g. 2.0): ") or "2.0")
                duration = float(input("Enter duration in seconds (e.g. 3.0): ") or "3.0")
                print(f"Driving BOTH wheels at {speed:.2f} rad/s for {duration:.1f}s...")
                t_end = time.time() + duration
                while time.time() < t_end:
                    ser.write(pack_cmd_vel(speed, speed))
                    time.sleep(0.04)
                ser.write(pack_cmd_vel(0.0, 0.0))
                print("[Done]")

            elif choice == '3':
                # Differential Turn
                turn_speed = float(input("Enter turn speed in rad/s (e.g. 1.5): ") or "1.5")
                print("Spinning robot clockwise (Left +, Right -) for 2.5s...")
                t_end = time.time() + 2.5
                while time.time() < t_end:
                    ser.write(pack_cmd_vel(turn_speed, -turn_speed))
                    time.sleep(0.04)
                ser.write(pack_cmd_vel(0.0, 0.0))
                print("[Done]")

            elif choice == '4':
                # Open-Loop PWM
                duty = int(input("Enter raw PWM duty [-1023 to +1023] (e.g. 300): ") or "300")
                duration = float(input("Enter duration in seconds (e.g. 2.0): ") or "2.0")
                print(f"Applying PWM duty {duty} to SmartElex 30S for {duration:.1f}s...")
                t_end = time.time() + duration
                while time.time() < t_end:
                    ser.write(pack_raw_pwm(duty, duty))
                    time.sleep(0.04)
                ser.write(pack_cmd_vel(0.0, 0.0))
                print("[Done]")

            elif choice == '5':
                # PID Tune
                print("Current defaults: Kp=45.0, Ki=110.0, Kd=1.2, Kff=42.0")
                kp = float(input("New Kp: ") or "45.0")
                ki = float(input("New Ki: ") or "110.0")
                kd = float(input("New Kd: ") or "1.2")
                kff = float(input("New Kff: ") or "42.0")
                ser.write(pack_pid_tune(kp, ki, kd, kff))
                print(f"[✓] Sent updated PID gains: Kp={kp}, Ki={ki}, Kd={kd}, Kff={kff}")

            elif choice == '6':
                # E-Stop
                ser.write(pack_estop())
                print("[✓] Emergency Stop Sent to ESP32-S3.")

            elif choice == '7':
                # Live Sonar Monitoring
                print("\nStreaming 4-channel US-100 Ultrasonic distances (Press Ctrl+C to stop)...")
                try:
                    while True:
                        ser.write(pack_heartbeat())
                        if ser.in_waiting:
                            chunk = ser.read(ser.in_waiting)
                            parser.feed(chunk)
                            while True:
                                pkt = parser.parse()
                                if pkt is None:
                                    break
                                msg_type, payload = pkt
                                if msg_type == MSG_TELEMETRY:
                                    t = unpack_telemetry(payload)
                                    if t:
                                        front_alert = " [CRITICAL STOP]" if (t.status_flags & 0x08) else ""
                                        rear_alert  = " [CRITICAL STOP]" if (t.status_flags & 0x10) else ""
                                        print(f"\r[SONARS] Front: {t.sonar_front_m:4.2f}m{front_alert:16s} | "
                                              f"Left: {t.sonar_left_m:4.2f}m | Right: {t.sonar_right_m:4.2f}m | "
                                              f"Rear: {t.sonar_rear_m:4.2f}m{rear_alert:16s} ", end='', flush=True)
                        time.sleep(0.03)
                except KeyboardInterrupt:
                    print("\n[Stopped]")

            elif choice.lower() == 'q':
                break

    except KeyboardInterrupt:
        pass
    finally:
        ser.write(pack_estop())
        ser.close()
        print("\nDisconnected from ESP32-S3 mediator.")

if __name__ == '__main__':
    main()
