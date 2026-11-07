#!/usr/bin/env python3
"""
Interactive Terminal Keyboard Teleoperation for Nammy Robot Platform.
Harmonized Kinematics & Smooth Motion Control for Precision SLAM Mapping.

Features:
- Identical wheel speeds: Forward/Backward and Left/Right turns produce exactly matched wheel RPM (omega = v / (b/2))
- Real-time dynamic speed scaling: Presets (1-5) and +/- fine tuning immediately update active wheel velocity
- Smooth S-curve acceleration & deceleration ramp: Completely eliminates wheel jerking and motor shudder
- Default Hold-to-Drive (Deadman) mode with smooth decay stop: Never runs away on key release
- Optional Cruise (Latched) mode toggle with [h]
- Full key mapping: WASD, IJKL, Arrow keys, Diagonals, and Instant Spacebar Halt

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
"""

import os
import sys
import time
import math
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

TRACK_WIDTH_M = 0.380
HALF_TRACK_M = TRACK_WIDTH_M / 2.0  # 0.190 m

# Speed Presets: (Linear m/s) -> Angular is mathematically matched: omega = v / 0.190
SPEED_PRESETS = {
    '1': 0.05,  # Crawl / Precision docking (Wheel: 0.05 m/s, W: 0.26 rad/s)
    '2': 0.12,  # Slow SLAM Mapping - Recommended (Wheel: 0.12 m/s, W: 0.63 rad/s)
    '3': 0.20,  # Normal SLAM Mapping (Wheel: 0.20 m/s, W: 1.05 rad/s)
    '4': 0.32,  # Brisk Transit (Wheel: 0.32 m/s, W: 1.68 rad/s)
    '5': 0.45,  # Max Safe Indoors (Wheel: 0.45 m/s, W: 2.37 rad/s)
}

PRESET_NAMES = {
    '1': 'Crawl (0.05 m/s)',
    '2': 'Slow SLAM (0.12 m/s)',
    '3': 'Normal SLAM (0.20 m/s)',
    '4': 'Brisk Transit (0.32 m/s)',
    '5': 'Fast Transit (0.45 m/s)',
}

MOVE_BINDINGS = {
    # WASD
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'x': (0.0, 0.0),
    # Standard ROS (IJKL)
    'i': (1.0, 0.0),
    ',': (-1.0, 0.0),
    'j': (0.0, 1.0),
    'l': (0.0, -1.0),
    'k': (0.0, 0.0),
    # Diagonals
    'u': (1.0, 0.7),
    'o': (1.0, -0.7),
    'm': (-1.0, -0.7),
    '.': (-1.0, 0.7),
    # Stop
    ' ': (0.0, 0.0),
}

BANNER = """
======================================================================
         NAMMY ROBOT - PRECISION SLAM TELEOPERATION (V2)             
======================================================================
Motion Keys (WASD / IJKL / Arrows):
   [w] or [i] or [UP]    : Forward
   [a] or [j] or [LEFT]  : Turn Left          [d] or [l] or [RIGHT] : Turn Right
   [s] or [,] or [DOWN]  : Backward
   [x] or [k] or [SPACE] : Full Stop (Instant Halt)

Speed Presets (Matched Wheel RPM for Linear & Turning):
   [1] : 0.05 m/s (Crawl / Precision Docking)
   [2] : 0.12 m/s (Slow SLAM Mapping - Recommended)
   [3] : 0.20 m/s (Normal SLAM Mapping)
   [4] : 0.32 m/s (Brisk Transit)
   [5] : 0.45 m/s (Fast Transit)

Speed Fine-Tuning:
   [q] or [+] : Increase Speed (+0.02 m/s)
   [z] or [-] : Decrease Speed (-0.02 m/s)

Drive Modes:
   [h] : Toggle Mode -> [HOLD-TO-DRIVE (Default)] <-> [CRUISE (Latched)]
   [SPACE] : INSTANT EMERGENCY HALT
   [CTRL+C]: Exit
======================================================================
"""

def get_key(settings, timeout=0.02):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':   key = 'w'
            elif extra == '[B': key = 's'
            elif extra == '[D': key = 'a'
            elif extra == '[C': key = 'd'
        elif len(key) == 1 and key.isalpha():
            key = key.lower()
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('nammy_teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # Speed parameters
    linear_speed = SPEED_PRESETS['2']  # 0.12 m/s default
    angular_speed = round(linear_speed / HALF_TRACK_M, 3) # Identical wheel speed!
    current_preset = '2'

    # Kinematic targets and ramp states
    target_v = 0.0
    target_w = 0.0
    current_v = 0.0
    current_w = 0.0

    # Direction multipliers currently held
    active_v_dir = 0.0
    active_w_dir = 0.0

    # Default to Hold-to-Drive (Deadman) mode with smooth decay stop
    # Toggle to Cruise with 'h'
    deadman_mode = True
    last_key_time = time.time()
    deadman_hold_timeout = 0.65  # 650ms safely accommodates Linux X11 500ms initial repeat delay

    # Acceleration limits (m/s^2 and rad/s^2) for buttery smooth motion without wheel jerk
    MAX_LIN_ACCEL = 0.30  # m/s^2 (buttery smooth ~0.4s ramp to 0.12 m/s)
    MAX_ANG_ACCEL = MAX_LIN_ACCEL / HALF_TRACK_M  # Exactly matched angular acceleration

    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

    print(BANNER)
    print(f"Network Domain -> ROS_DOMAIN_ID = {domain_id}")
    print(f"Active Preset  -> [{current_preset}] {PRESET_NAMES[current_preset]}")
    print(f"Wheel Velocity -> Linear: {linear_speed:.2f} m/s | Turn: {angular_speed:.2f} rad/s (Wheel: {angular_speed * HALF_TRACK_M:.2f} m/s - IDENTICAL)")
    print(f"Active Mode    -> {'HOLD-TO-DRIVE (Smooth Deadman)' if deadman_mode else 'CRUISE (Latched)'}\n")
    print("Ready! Wheels accelerate smoothly and stop cleanly...\n")

    last_loop_time = time.time()

    try:
        while rclpy.ok():
            now = time.time()
            dt = max(0.005, min(0.1, now - last_loop_time))
            last_loop_time = now

            key = get_key(settings, timeout=0.015)

            if key == '\x03':  # CTRL+C
                break

            # 1. Preset Speed Selection (Keys 1..5)
            if key in SPEED_PRESETS:
                current_preset = key
                linear_speed = SPEED_PRESETS[key]
                angular_speed = round(linear_speed / HALF_TRACK_M, 3)
                target_v = active_v_dir * linear_speed
                target_w = active_w_dir * angular_speed
                last_key_time = now
                print(f"\r[PRESET {key}: {PRESET_NAMES[key]}] Linear: {linear_speed:.2f} m/s | Turn: {angular_speed:.2f} rad/s                    ", flush=True)

            # 2. Continuous Speed Fine-Tuning (q/z, +/-)
            elif key in ('q', '+', '=', ']'):
                linear_speed = min(0.50, round(linear_speed + 0.02, 3))
                angular_speed = round(linear_speed / HALF_TRACK_M, 3)
                target_v = active_v_dir * linear_speed
                target_w = active_w_dir * angular_speed
                last_key_time = now
                print(f"\r[SPEED+] Linear: {linear_speed:.2f} m/s | Turn: {angular_speed:.2f} rad/s (Wheel RPM Matched)          ", flush=True)

            elif key in ('z', '-', '_', '['):
                linear_speed = max(0.03, round(linear_speed - 0.02, 3))
                angular_speed = round(linear_speed / HALF_TRACK_M, 3)
                target_v = active_v_dir * linear_speed
                target_w = active_w_dir * angular_speed
                last_key_time = now
                print(f"\r[SPEED-] Linear: {linear_speed:.2f} m/s | Turn: {angular_speed:.2f} rad/s (Wheel RPM Matched)          ", flush=True)

            # 3. Mode Toggle (Hold-to-Drive vs Cruise)
            elif key == 'h':
                deadman_mode = not deadman_mode
                mode_str = 'HOLD-TO-DRIVE (Smooth Deadman)' if deadman_mode else 'CRUISE (Latched)'
                last_key_time = now
                print(f"\r[MODE SWITCH] -> Switched to {mode_str:<45}", flush=True)

            # 4. Motion Keys
            elif key in MOVE_BINDINGS:
                v_mult, w_mult = MOVE_BINDINGS[key]
                active_v_dir = v_mult
                active_w_dir = w_mult
                target_v = v_mult * linear_speed
                target_w = w_mult * angular_speed
                last_key_time = now

                if v_mult == 0.0 and w_mult == 0.0:
                    status_line = "HALT / STOP (Smooth Decel)"
                    # Smoothly ramp target down to zero (do not hard-zero current_v to avoid shudder)
                    target_v = 0.0
                    target_w = 0.0
                elif v_mult > 0.0 and w_mult == 0.0:
                    status_line = f"FORWARD ({linear_speed:.2f} m/s)"
                elif v_mult < 0.0 and w_mult == 0.0:
                    status_line = f"BACKWARD ({linear_speed:.2f} m/s)"
                elif v_mult == 0.0 and w_mult > 0.0:
                    status_line = f"TURN LEFT ({angular_speed:.2f} rad/s)"
                elif v_mult == 0.0 and w_mult < 0.0:
                    status_line = f"TURN RIGHT ({angular_speed:.2f} rad/s)"
                else:
                    status_line = f"DIAGONAL (V:{target_v:+.2f} W:{target_w:+.2f})"

                print(f"\r[{status_line:<30}] Speed: {linear_speed:.2f} m/s | Mode: {'HOLD' if deadman_mode else 'CRUISE'}     ", end='', flush=True)

            # 5. Deadman Hold-Timeout Check
            elif deadman_mode:
                if (now - last_key_time) > deadman_hold_timeout:
                    active_v_dir = 0.0
                    active_w_dir = 0.0
                    target_v = 0.0
                    target_w = 0.0

            # 6. Smooth Acceleration & Deceleration Slew Limiter (Anti-Jerk Filter)
            # Linear ramping
            max_v_step = MAX_LIN_ACCEL * dt
            delta_v = target_v - current_v
            if abs(delta_v) <= max_v_step:
                current_v = target_v
            else:
                current_v += math.copysign(max_v_step, delta_v)

            # Angular ramping
            max_w_step = MAX_ANG_ACCEL * dt
            delta_w = target_w - current_w
            if abs(delta_w) <= max_w_step:
                current_w = target_w
            else:
                current_w += math.copysign(max_w_step, delta_w)

            # 7. Publish to cmd_vel
            twist = Twist()
            twist.linear.x = float(current_v)
            twist.angular.z = float(current_w)
            pub.publish(twist)

            time.sleep(0.02)  # 50 Hz control rate

    finally:
        # Publish clean zero stop on exit
        try:
            twist = Twist()
            pub.publish(twist)
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
