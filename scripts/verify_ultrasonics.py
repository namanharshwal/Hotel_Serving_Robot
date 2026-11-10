#!/usr/bin/env python3
"""
==============================================================================
verify_ultrasonics.py - Live Verification Tool for 4x US-100 Ultrasonic Sensors
Author: Naman Harshwal <namanharshwal@gmail.com>
Repository: Hotel_Serving_Robot (https://github.com/namanharshwal/Hotel_Serving_Robot)

Checks and visualizes live readings from the 4 US-100 ultrasonic sensors:
  - Front Sonar
  - Left Sonar
  - Right Sonar
  - Rear Sonar

Hardware Wiring Specification:
  1. ESP32-S3 3V3  -> TXS0108E VCCA (3.3V) & OE (Output Enable, HIGH)
  2. 5V BEC Rail   -> TXS0108E VCCB (5.0V) & US-100 VCC (All 4)
  3. Star Ground   -> TXS0108E GND & US-100 GND (All 4)
  4. Channels:
     - Front: ESP32 GPIO 8  -> TXS0108E A1/B1 -> Front US-100 Trig
              ESP32 GPIO 9  <- TXS0108E A2/B2 <- Front US-100 Echo
     - Left:  ESP32 GPIO 10 -> TXS0108E A3/B3 -> Left US-100 Trig
              ESP32 GPIO 11 <- TXS0108E A4/B4 <- Left US-100 Echo
     - Right: ESP32 GPIO 12 -> TXS0108E A5/B5 -> Right US-100 Trig
              ESP32 GPIO 13 <- TXS0108E A6/B6 <- Right US-100 Echo
     - Rear:  ESP32 GPIO 14 -> TXS0108E A7/B7 -> Rear US-100 Trig
              ESP32 GPIO 21 <- TXS0108E A8/B8 <- Rear US-100 Echo

CRITICAL HARDWARE REQUIREMENT:
  The 2-pin jumper on the back of each US-100 module MUST BE REMOVED (OPEN).
  - Jumper REMOVED: Trigger / Echo Pulse Mode (Correct for GPIO pulseIn)
  - Jumper INSTALLED: Serial UART Mode (Sensor will NOT respond to trigger pulses)
==============================================================================
"""

import sys
import os
import time

def print_banner():
    os.system('clear' if os.name == 'posix' else 'cls')
    print("=" * 78)
    print("     NAMMY ROBOT - 4x US-100 ULTRASONIC SENSOR VERIFICATION SUITE      ")
    print("     Author & Sole Developer: Naman Harshwal <namanharshwal@gmail.com> ")
    print("=" * 78)
    print(" HARDWARE CONNECTION MATRIX (via TXS0108E 3.3V <-> 5.0V Level Shifter):")
    print("   • FRONT: Trig -> ESP32 GPIO 8  (A1/B1) | Echo -> ESP32 GPIO 9  (A2/B2)")
    print("   • LEFT:  Trig -> ESP32 GPIO 10 (A3/B3) | Echo -> ESP32 GPIO 11 (A4/B4)")
    print("   • RIGHT: Trig -> ESP32 GPIO 12 (A5/B5) | Echo -> ESP32 GPIO 13 (A6/B6)")
    print("   • REAR:  Trig -> ESP32 GPIO 14 (A7/B7) | Echo -> ESP32 GPIO 21 (A8/B8)")
    print("   • LEVEL SHIFTER: VCCA = 3.3V, OE = 3.3V, VCCB = 5.0V, GND = Star Ground")
    print("   • US-100 JUMPERS: MUST BE REMOVED (OPEN) FOR TRIGGER/ECHO PULSE MODE")
    print("=" * 78)

def format_bar(val: float, width: int = 28) -> str:
    if val is None or val > 4.5:
        return "[" + " " * width + "]"
    fill = int(min(max(val / 4.5, 0.0), 1.0) * width)
    if val <= 0.15:
        color = "\033[91;1m" # Bold Red
    elif val <= 0.50:
        color = "\033[93;1m" # Bold Yellow
    else:
        color = "\033[92;1m" # Bold Green
    return "[" + color + "#" * max(1, fill) + "\033[0m" + " " * (width - max(1, fill)) + "]"

def run_ros_verifier():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Range
    except ImportError:
        return False

    rclpy.init(args=None)

    class UltrasonicMonitorNode(Node):
        def __init__(self):
            super().__init__('ultrasonic_verifier')
            self.ranges = {
                'front': {'val': None, 'time': 0, 'verified': False},
                'left':  {'val': None, 'time': 0, 'verified': False},
                'right': {'val': None, 'time': 0, 'verified': False},
                'rear':  {'val': None, 'time': 0, 'verified': False}
            }
            self.create_subscription(Range, '/sonar/front', lambda m: self.cb('front', m), 10)
            self.create_subscription(Range, '/sonar/left',  lambda m: self.cb('left',  m), 10)
            self.create_subscription(Range, '/sonar/right', lambda m: self.cb('right', m), 10)
            self.create_subscription(Range, '/sonar/rear',  lambda m: self.cb('rear',  m), 10)

        def cb(self, sensor_name, msg):
            v = msg.range
            self.ranges[sensor_name]['val'] = v
            self.ranges[sensor_name]['time'] = time.time()
            if v < 4.0:
                self.ranges[sensor_name]['verified'] = True

    node = UltrasonicMonitorNode()
    print_banner()
    print(" [MODE] Active ROS 2 Bridge Stream (/sonar/* topics)")
    print(" Place your hand or an obstacle 10 cm - 80 cm in front of each sensor:")
    print("------------------------------------------------------------------------------")
    print("\n" * 4, end="")

    try:
        last_render = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)
            now = time.time()
            if now - last_render >= 0.1:
                last_render = now
                lines = []
                sensors = [
                    ('front', 'FRONT SONAR', 'Trig 8  / Echo 9 '),
                    ('left',  'LEFT SONAR ', 'Trig 10 / Echo 11'),
                    ('right', 'RIGHT SONAR', 'Trig 12 / Echo 13'),
                    ('rear',  'REAR SONAR ', 'Trig 14 / Echo 21')
                ]
                for name, label, pins in sensors:
                    entry = node.ranges[name]
                    val = entry['val']
                    age = now - entry['time'] if entry['time'] > 0 else 999.0

                    if val is None or age > 2.0:
                        status = "\033[93m[WAITING]\033[0m"
                        bar = "[" + " " * 28 + "]"
                        dist_str = " --.-- m (--.- cm) "
                    elif val >= 4.45:
                        status = "\033[90m[CLEAR / >4.5m]\033[0m"
                        bar = "[" + "=" * 28 + "]"
                        dist_str = f" {val:4.2f} m ({val*100:5.1f} cm)"
                    elif val <= 0.15:
                        status = "\033[91;1m[CRITICAL STOP]\033[0m"
                        bar = format_bar(val)
                        dist_str = f" \033[91m{val:4.2f} m ({val*100:5.1f} cm)\033[0m"
                    elif val <= 0.50:
                        status = "\033[93;1m[NEAR DETECTED]\033[0m"
                        bar = format_bar(val)
                        dist_str = f" \033[93m{val:4.2f} m ({val*100:5.1f} cm)\033[0m"
                    else:
                        status = "\033[92;1m[FAR DETECTED]\033[0m"
                        bar = format_bar(val)
                        dist_str = f" \033[92m{val:4.2f} m ({val*100:5.1f} cm)\033[0m"

                    verified_badge = " \033[92m[✓ VERIFIED]\033[0m" if entry['verified'] else " \033[90m[UNTESTED]\033[0m"
                    lines.append(f" {label} [{pins}]: {bar} {dist_str}  {status:<24}{verified_badge}")

                print("\033[F\033[K" * 4, end="")
                for line in lines:
                    print(line)
                sys.stdout.flush()

    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    return True

def run_serial_verifier():
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'nammy_base'))
    import serial
    from nammy_base.packet_protocol import PacketParser, pack_heartbeat, unpack_telemetry, MSG_TELEMETRY

    port = '/dev/nammy_esp32'
    if not os.path.exists(port):
        print(f"[!] Device node {port} does not exist.")
        return False

    print(f"[i] Connecting directly to {port} @ 115200 baud...")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"[!] Cannot open {port}: {e}")
        return False

    parser = PacketParser()
    print_banner()
    print(" [MODE] Direct Serial Connection to ESP32-S3 (/dev/nammy_esp32)")
    print(" Place your hand or an obstacle 10 cm - 80 cm in front of each sensor:")
    print("------------------------------------------------------------------------------")
    print("\n" * 4, end="")

    verified = {'front': False, 'left': False, 'right': False, 'rear': False}
    last_render = 0
    t = None

    try:
        while True:
            ser.write(pack_heartbeat())
            while ser.in_waiting:
                b = ser.read(1)
                res = parser.process_byte(b[0])
                if res:
                    msg_type, payload = res
                    if msg_type == MSG_TELEMETRY:
                        t = unpack_telemetry(payload)
                        if t:
                            if t.sonar_front_m < 4.0: verified['front'] = True
                            if t.sonar_left_m  < 4.0: verified['left']  = True
                            if t.sonar_right_m < 4.0: verified['right'] = True
                            if t.sonar_rear_m  < 4.0: verified['rear']  = True

            now = time.time()
            if now - last_render >= 0.1 and t:
                last_render = now
                lines = []
                sensors = [
                    ('front', 'FRONT SONAR', 'Trig 8  / Echo 9 ', t.sonar_front_m),
                    ('left',  'LEFT SONAR ', 'Trig 10 / Echo 11', t.sonar_left_m),
                    ('right', 'RIGHT SONAR', 'Trig 12 / Echo 13', t.sonar_right_m),
                    ('rear',  'REAR SONAR ', 'Trig 14 / Echo 21', t.sonar_rear_m)
                ]
                for key, label, pins, val in sensors:
                    if val >= 4.45:
                        status = "\033[90m[CLEAR / >4.5m]\033[0m"
                        bar = "[" + "=" * 28 + "]"
                        dist_str = f" {val:4.2f} m ({val*100:5.1f} cm)"
                    elif val <= 0.15:
                        status = "\033[91;1m[CRITICAL STOP]\033[0m"
                        bar = format_bar(val)
                        dist_str = f" \033[91m{val:4.2f} m ({val*100:5.1f} cm)\033[0m"
                    elif val <= 0.50:
                        status = "\033[93;1m[NEAR DETECTED]\033[0m"
                        bar = format_bar(val)
                        dist_str = f" \033[93m{val:4.2f} m ({val*100:5.1f} cm)\033[0m"
                    else:
                        status = "\033[92;1m[FAR DETECTED]\033[0m"
                        bar = format_bar(val)
                        dist_str = f" \033[92m{val:4.2f} m ({val*100:5.1f} cm)\033[0m"

                    verified_badge = " \033[92m[✓ VERIFIED]\033[0m" if verified[key] else " \033[90m[UNTESTED]\033[0m"
                    lines.append(f" {label} [{pins}]: {bar} {dist_str}  {status:<24}{verified_badge}")

                print("\033[F\033[K" * 4, end="")
                for line in lines:
                    print(line)
                sys.stdout.flush()

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    return True

def run_quick_check():
    """Sample sensors for 2 seconds and print a formatted diagnostic table."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Range
    except ImportError:
        print("[!] ROS 2 Humble python modules not found.")
        return False

    rclpy.init(args=None)
    data = {'front': [], 'left': [], 'right': [], 'rear': []}

    class Checker(Node):
        def __init__(self):
            super().__init__('ultrasonic_checker')
            self.create_subscription(Range, '/sonar/front', lambda m: data['front'].append(m.range), 10)
            self.create_subscription(Range, '/sonar/left',  lambda m: data['left'].append(m.range),  10)
            self.create_subscription(Range, '/sonar/right', lambda m: data['right'].append(m.range), 10)
            self.create_subscription(Range, '/sonar/rear',  lambda m: data['rear'].append(m.range),  10)

    node = Checker()
    print_banner()
    print(" [DIAGNOSTIC] Sampling all 4 ultrasonic channels for 2.0 seconds...\n")
    t0 = time.time()
    while time.time() - t0 < 2.5:
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(len(samples) >= 10 for samples in data.values()):
            break

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print("+" + "-"*76 + "+")
    print(f"| {'SENSOR':<12} | {'PINS (TRIG/ECHO)':<18} | {'AVG DIST':<12} | {'SAMPLES':<8} | {'STATUS':<15} |")
    print("+" + "-"*76 + "+")

    sensor_info = [
        ('FRONT SONAR', 'GPIO 8 / GPIO 9',   'front'),
        ('LEFT SONAR',  'GPIO 10 / GPIO 11', 'left'),
        ('RIGHT SONAR', 'GPIO 12 / GPIO 13', 'right'),
        ('REAR SONAR',  'GPIO 14 / GPIO 21', 'rear'),
    ]

    for label, pins, key in sensor_info:
        samples = data[key]
        if not samples:
            dist_str = "NO DATA"
            status = "\033[91mDISCONNECTED\033[0m"
            cnt = "0"
        else:
            avg_dist = sum(samples) / len(samples)
            dist_str = f"{avg_dist:.2f} m ({avg_dist*100:.0f}cm)"
            cnt = str(len(samples))
            if avg_dist >= 4.45:
                status = "\033[93mIDLE / CLEAR\033[0m"
            elif avg_dist <= 0.15:
                status = "\033[91mCRITICAL STOP\033[0m"
            else:
                status = "\033[92mDETECTING\033[0m"

        print(f"| {label:<12} | {pins:<18} | {dist_str:<12} | {cnt:<8} | {status:<24} |")

    print("+" + "-"*76 + "+")
    return True

def main():
    if '--check' in sys.argv:
        run_quick_check()
        return

    has_ros = False
    try:
        import rclpy
        has_ros = True
    except ImportError:
        pass

    if has_ros:
        run_ros_verifier()
    else:
        run_serial_verifier()

    print("\n\n" + "=" * 78)
    print(" ULTRASONIC SENSOR TROUBLESHOOTING CHECKLIST:")
    print(" 1. US-100 JUMPERS:")
    print("    - Verify the 2-pin jumper on the back of all 4 sensors is REMOVED.")
    print("    - If jumper is ON, sensor operates in UART mode and will ALWAYS timeout (4.5m).")
    print(" 2. LEVEL SHIFTER (TXS0108E):")
    print("    - VCCA must receive 3.3V from ESP32 3V3 pin.")
    print("    - OE pin MUST be tied to 3.3V (VCCA). If left floating, outputs are tri-stated.")
    print("    - VCCB must receive 5.0V from DC-DC Buck converter.")
    print(" 3. COMMON GROUND:")
    print("    - TXS0108E GND, ESP32 GND, and US-100 GNDs must all share system star ground.")
    print("=" * 78)

if __name__ == '__main__':
    main()
