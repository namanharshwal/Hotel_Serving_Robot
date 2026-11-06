#!/usr/bin/env python3
"""
Interactive Terminal Keyboard Teleoperation for Nammy Robot Platform.
Optimized for Manual SLAM Mapping with Instant Wheel Response and Matched Linear/Angular Speeds.

Features:
- Instantaneous wheel response (<20ms latency, zero sluggish software ramp delay)
- Harmonized linear and angular speeds so left/right turning matches forward/backward speed
- Presets (1-5) spanning general SLAM mobile robot velocities (0.05 m/s to 0.50 m/s)
- Continuous fine-tuning with q/z or +/- from lowest (0.03 m/s) to highest (0.60 m/s)
- Supports WASD, standard ROS (IJKL), and Arrow keys
- Dual Operational Modes: CRUISE (Latched - default) and DEADMAN (Hold key to drive)

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
"""

import os
import sys
import time
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
======================================================================
         NAMMY ROBOT - HIGH-RESPONSIVENESS SLAM TELEOPERATION         
======================================================================
Motion Keys (WASD / IJKL / Arrows):
   [w] or [i] or [UP]    : Forward
   [a] or [j] or [LEFT]  : Turn Left          [d] or [l] or [RIGHT] : Turn Right
   [s] or [,] or [DOWN]  : Backward
   [x] or [k] or [SPACE] : Full Stop (Instant Halt)

Compound Diagonal Motion:
   [u] : Fwd + Left        [o] : Fwd + Right
   [m] : Rev + Left        [.] : Rev + Right

Standard SLAM Speed Presets (Mobile Robots):
   [1] : 0.05 m/s (Crawl / Tight Passages / Precision SLAM)
   [2] : 0.12 m/s (Slow SLAM Mapping - Recommended)
   [3] : 0.20 m/s (Normal SLAM Mapping)
   [4] : 0.35 m/s (Brisk Transit)
   [5] : 0.50 m/s (High Speed Transit)

Speed Fine-Tuning:
   [q] or [+] : Increase Speed (+0.02 m/s)
   [z] or [-] : Decrease Speed (-0.02 m/s)
   [e] / [c]  : Fine-tune Angular Speed Only (+/- 0.05 rad/s)

Operational Modes:
   [h] : Toggle Mode -> [CRUISE (Latched - Default)] <-> [DEADMAN (Hold Key)]
   [SPACE] : INSTANT STOP
   [CTRL+C]: Exit
======================================================================
"""

# Preset configurations (Linear m/s, Angular rad/s)
SPEED_PRESETS = {
    '1': (0.05, 0.12),   # Ultra Slow / Crawl (~7 deg/s)
    '2': (0.12, 0.25),   # Slow SLAM Mapping (~14 deg/s) - Matched to linear travel
    '3': (0.20, 0.40),   # Medium SLAM (~23 deg/s)
    '4': (0.35, 0.60),   # Brisk (~34 deg/s)
    '5': (0.50, 0.80),   # High Speed (~46 deg/s)
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
    'u': (1.0, 1.0),
    'o': (1.0, -1.0),
    'm': (-1.0, -1.0),
    '.': (-1.0, 1.0),
    # Stop
    ' ': (0.0, 0.0),
}

def get_key(settings, timeout=0.03):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        # Handle 3-byte ANSI arrow key escape sequences (ESC [ A/B/C/D)
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':   # Up arrow
                key = 'w'
            elif extra == '[B': # Down arrow
                key = 's'
            elif extra == '[D': # Left arrow
                key = 'a'
            elif extra == '[C': # Right arrow
                key = 'd'
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('nammy_teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # Initial speed set to Preset 2 (Slow SLAM Mapping)
    linear_speed = 0.12    # m/s
    angular_speed = 0.25   # rad/s - perfectly harmonized with linear speed!
    current_preset = '2'

    target_v = 0.0
    target_w = 0.0
    last_key_time = 0.0
    deadman_mode = False   # False = Latched Cruise (Default), True = Hold Key
    deadman_timeout = 0.65 # Seconds (immune to 500ms OS repeat delay)
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

    print(BANNER)
    print(f"Network Domain -> ROS_DOMAIN_ID = {domain_id}")
    print(f"Active Preset  -> [{current_preset}] Slow SLAM Mapping (Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s)")
    print(f"Active Mode    -> {'DEADMAN (Hold Key)' if deadman_mode else 'CRUISE (Latched - tap to drive, k/Space to stop)'}\n")
    print("Ready! Wheels respond immediately to keystrokes...")

    status_line = "IDLE / STOPPED"

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.03)
            now = time.time()

            if key == '\x03':  # CTRL+C
                break

            if key in SPEED_PRESETS:
                current_preset = key
                linear_speed, angular_speed = SPEED_PRESETS[key]
                preset_names = {'1': 'Crawl', '2': 'Slow SLAM', '3': 'Medium SLAM', '4': 'Brisk', '5': 'Max'}
                print(f"\r[PRESET {key}: {preset_names.get(key, '')}] V: {linear_speed:.2f} m/s | W: {angular_speed:.2f} rad/s                          ", flush=True)

            elif key in ('q', '+', '='):
                linear_speed = min(0.60, round(linear_speed + 0.02, 3))
                angular_speed = min(1.20, round(linear_speed * 2.1, 3))
                print(f"\r[SPEED+] Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s                                 ", flush=True)

            elif key in ('z', '-', '_'):
                linear_speed = max(0.03, round(linear_speed - 0.02, 3))
                angular_speed = max(0.08, round(linear_speed * 2.1, 3))
                print(f"\r[SPEED-] Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s                                 ", flush=True)

            elif key == 'e':
                angular_speed = min(1.50, round(angular_speed + 0.05, 3))
                print(f"\r[ANGULAR+] Angular: {angular_speed:.2f} rad/s (Linear: {linear_speed:.2f} m/s)                             ", flush=True)

            elif key == 'c':
                angular_speed = max(0.08, round(angular_speed - 0.05, 3))
                print(f"\r[ANGULAR-] Angular: {angular_speed:.2f} rad/s (Linear: {linear_speed:.2f} m/s)                             ", flush=True)

            elif key == 'h':  # Toggle drive mode
                deadman_mode = not deadman_mode
                mode_str = 'DEADMAN (Hold Key)' if deadman_mode else 'CRUISE (Latched - Tap)'
                print(f"\r[MODE SWITCH] -> Switched to {mode_str:<40}", flush=True)

            elif key in MOVE_BINDINGS:
                v_mult, w_mult = MOVE_BINDINGS[key]
                target_v = v_mult * linear_speed
                target_w = w_mult * angular_speed
                last_key_time = now

                if v_mult == 0.0 and w_mult == 0.0:
                    status_line = "HALT / STOPPED"
                elif v_mult > 0.0 and w_mult == 0.0:
                    status_line = f"FORWARD ({target_v:+.2f} m/s)"
                elif v_mult < 0.0 and w_mult == 0.0:
                    status_line = f"BACKWARD ({target_v:+.2f} m/s)"
                elif v_mult == 0.0 and w_mult > 0.0:
                    status_line = f"TURN LEFT ({target_w:+.2f} rad/s)"
                elif v_mult == 0.0 and w_mult < 0.0:
                    status_line = f"TURN RIGHT ({target_w:+.2f} rad/s)"
                else:
                    status_line = f"DIAGONAL (V:{target_v:+.2f} W:{target_w:+.2f})"

                print(f"\r[{status_line:<28}] Mode: {'HOLD' if deadman_mode else 'CRUISE'} | Speed: {linear_speed:.2f} m/s   ", end='', flush=True)

            elif deadman_mode:
                # In Deadman mode only: key release after timeout stops target immediately
                if now - last_key_time > deadman_timeout and (target_v != 0.0 or target_w != 0.0):
                    target_v = 0.0
                    target_w = 0.0
                    print(f"\r[HOLD-RELEASE] Stopped.                                               ", end='', flush=True)

            # Instant direct publication: Zero software ramp delay!
            twist = Twist()
            twist.linear.x = float(target_v)
            twist.angular.z = float(target_w)
            pub.publish(twist)

            time.sleep(0.02)  # Deterministic 50 Hz teleop command rate

    except Exception as e:
        print(f"\nTeleop exception: {e}")

    finally:
        # Publish final zero stop velocity
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\nTeleoperation stopped safely. Robot halted.")

if __name__ == '__main__':
    main()
