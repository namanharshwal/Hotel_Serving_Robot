#!/usr/bin/env python3
"""
Interactive Terminal Keyboard Teleoperation for Nammy Robot.
Supports continuous driving, speed increments, and instantaneous emergency stop.
Accepts both WASD keys, standard ROS (IJKL) keys, and arrow keys.
Author: Naman Harshwal <namanharshwal@gmail.com>
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
               NAMMY ROBOT - KEYBOARD TELEOPERATION                  
======================================================================
Control Keys (WASD or IJKL or Arrows):
   [w] or [i] or [UP]    : Drive Forward
   [a] or [j] or [LEFT]  : Turn Left          [d] or [l] or [RIGHT] : Turn Right
   [s] or [,] or [DOWN]  : Drive Backward
   [x] or [k] or [SPACE] : Full Stop (Smooth Brake)

Diagonal / Compound Motion:
   [u] : Forward + Left           [o] : Forward + Right
   [m] : Reverse + Left           [.] : Reverse + Right

Drive Modes:
   [h] : Toggle Mode -> [CRUISE (Latched - Default)] <-> [DEADMAN (Hold Key)]

Speed Modifiers:
   [q] : Increase Linear Speed (+0.05 m/s)   [z] : Decrease Linear Speed (-0.05 m/s)
   [e] : Increase Angular Speed (+0.10 rad/s) [c] : Decrease Angular Speed (-0.10 rad/s)

Safety:
   [SPACE] : STOP IMMEDIATELY
   [CTRL+C]: Exit Teleoperation
======================================================================
"""

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

SPEED_BINDINGS = {
    'q': (0.05, 0.0),
    'z': (-0.05, 0.0),
    'e': (0.0, 0.10),
    'c': (0.0, -0.10),
}

def get_key(settings, timeout=0.05):
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

    linear_speed = 0.35   # m/s
    angular_speed = 0.75  # rad/s
    target_v = 0.0
    target_w = 0.0
    current_v = 0.0
    current_w = 0.0
    accel_v = 0.50        # m/s^2 smooth acceleration profile
    accel_w = 1.00        # rad/s^2 smooth angular acceleration profile
    last_key_time = 0.0
    last_loop_time = time.time()
    deadman_mode = False  # False = Latched Cruise (Standard ROS), True = Hold Key
    deadman_timeout = 0.85 # Seconds (immune to 500ms OS repeat delay)
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

    print(BANNER)
    print(f"Network Domain -> ROS_DOMAIN_ID = {domain_id}")
    print(f"Initial Speeds -> Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s")
    print(f"Active Mode    -> {'DEADMAN (Hold Key)' if deadman_mode else 'CRUISE (Latched - tap to drive, k/Space to stop)'}\n")
    print("Ready! Driving is enabled...")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.04)
            now = time.time()
            dt = max(0.001, min(0.10, now - last_loop_time))
            last_loop_time = now

            if key == '\x03':  # CTRL+C
                break

            if key == 'h':  # Toggle drive mode
                deadman_mode = not deadman_mode
                mode_str = 'DEADMAN (Hold Key)' if deadman_mode else 'CRUISE (Latched - Tap)'
                print(f"\r[MODE SWITCH] -> Switched to {mode_str:<40}", flush=True)

            elif key in MOVE_BINDINGS:
                v_mult, w_mult = MOVE_BINDINGS[key]
                target_v = v_mult * linear_speed
                target_w = w_mult * angular_speed
                last_key_time = now
                if v_mult == 0.0 and w_mult == 0.0:
                    status_text = "HALT / STOP"
                else:
                    status_text = f"CMD V:{target_v:+.2f}m/s W:{target_w:+.2f}rad/s"
                print(f"\r[{status_text}] Act V:{current_v:+.2f} W:{current_w:+.2f} | Mode: {'HOLD' if deadman_mode else 'CRUISE'}     ", end='', flush=True)

            elif key in SPEED_BINDINGS:
                dv, dw = SPEED_BINDINGS[key]
                linear_speed = max(0.05, min(1.20, linear_speed + dv))
                angular_speed = max(0.10, min(2.50, angular_speed + dw))
                print(f"\r[CONFIG] Speeds -> Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s         ", end='', flush=True)

            elif deadman_mode:
                # In Deadman mode only: key release after timeout decelerates target to zero
                if now - last_key_time > deadman_timeout and (target_v != 0.0 or target_w != 0.0):
                    target_v = 0.0
                    target_w = 0.0
                    print(f"\r[HOLD-RELEASE] Smoothly decelerating to zero...                           ", end='', flush=True)

            # Smooth acceleration and deceleration slew rate limiter
            dv_max = accel_v * dt
            dw_max = accel_w * dt

            if target_v > current_v:
                current_v = min(target_v, current_v + dv_max)
            elif target_v < current_v:
                current_v = max(target_v, current_v - dv_max)

            if target_w > current_w:
                current_w = min(target_w, current_w + dw_max)
            elif target_w < current_w:
                current_w = max(target_w, current_w - dw_max)

            # Snap to exact zero when near zero
            if abs(current_v) < 0.005:
                current_v = 0.0
            if abs(current_w) < 0.005:
                current_w = 0.0

            # Continually publish cmd_vel at deterministic rate
            twist = Twist()
            twist.linear.x = float(current_v)
            twist.angular.z = float(current_w)
            pub.publish(twist)

            time.sleep(0.01)

    except Exception as e:
        print(f"\nTeleop exception: {e}")

    finally:
        # Publish final zero stop velocity
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\nTeleoperation stopped safely.")

if __name__ == '__main__':
    main()
