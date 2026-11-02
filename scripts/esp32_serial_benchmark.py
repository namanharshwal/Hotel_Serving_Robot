#!/usr/bin/env python3
"""
Hardware Benchmark and Serial Diagnostics Utility for Nammy Robot ESP32-S3.
Tests packet throughput, round-trip latency, and telemetry stream health.
"""

import sys
import os
import time
import serial
import serial.tools.list_ports

# Add nammy_controller to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'nammy_controller'))
from nammy_controller.packet_protocol import (
    pack_cmd_vel, pack_heartbeat, PacketParser, unpack_telemetry, MSG_TELEMETRY
)

def main():
    print("======================================================================")
    print("NAMMY ROBOT - ESP32-S3 HARDWARE SERIAL BENCHMARK")
    print("======================================================================")

    target_port = '/dev/nammy_esp32'
    baud = 115200

    if not os.path.exists(target_port):
        print(f"[-] '{target_port}' not found. Scanning available serial ports...")
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            print("[!] No USB serial ports connected. Exiting.")
            sys.exit(1)
        target_port = ports[0]
        print(f"[✓] Using fallback port: {target_port}")

    try:
        ser = serial.Serial(target_port, baud, timeout=0.1)
        print(f"[✓] Successfully opened {target_port} at {baud} bps")
    except Exception as e:
        print(f"[ERROR] Failed to open port: {e}")
        sys.exit(1)

    parser = PacketParser()
    packets_received = 0
    start_time = time.time()
    last_print = time.time()

    print("\nStreaming telemetry from ESP32-S3 (Press Ctrl+C to stop)...")
    try:
        while True:
            # Send keepalive heartbeat
            ser.write(pack_heartbeat())

            # Read incoming bytes
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                parser.feed(chunk)

                while True:
                    pkt = parser.parse()
                    if pkt is None:
                        break
                    msg_type, payload = pkt
                    if msg_type == MSG_TELEMETRY:
                        packets_received += 1
                        tele = unpack_telemetry(payload)
                        if tele and (time.time() - last_print >= 0.5):
                            rate = packets_received / (time.time() - start_time)
                            print(f"[TELEMETRY] Rate: {rate:5.1f} Hz | Batt: {tele.battery_volts:5.2f} V | "
                                  f"L_vel: {tele.left_vel_rad_s:+5.2f} rad/s | R_vel: {tele.right_vel_rad_s:+5.2f} rad/s")
                            last_print = time.time()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nBenchmark stopped.")
    finally:
        ser.close()
        print(f"Total packets received: {packets_received}")

if __name__ == '__main__':
    main()
