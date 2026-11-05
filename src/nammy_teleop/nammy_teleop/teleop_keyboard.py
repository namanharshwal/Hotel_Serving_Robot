#!/usr/bin/env python3
"""
Interactive Terminal Keyboard Teleoperation for Nammy Robot.
Supports continuous driving, speed increments, and instantaneous emergency stop.
Accepts both WASD keys, standard ROS (IJKL) keys, and arrow keys.
"""

import os
import sys
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
   [w] or [i] : Drive Forward
   [a] or [j] : Turn Left          [d] or [l] : Turn Right
   [s] or [,] : Drive Backward
   [x] or [k] : Full Stop (Brake)

Diagonal / Compound Motion:
   [u] : Forward + Left           [o] : Forward + Right
   [m] : Reverse + Left           [.] : Reverse + Right

Speed Modifiers:
   [q] : Increase Max Linear Speed (+0.05 m/s)
   [z] : Decrease Max Linear Speed (-0.05 m/s)
   [e] : Increase Max Angular Speed (+0.10 rad/s)
   [c] : Decrease Max Angular Speed (-0.10 rad/s)

Safety:
   [SPACE] : EMERGENCY STOP (Brake immediately)
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

def get_key(settings, timeout=0.1):
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
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

    print(BANNER)
    print(f"Network Domain -> ROS_DOMAIN_ID = {domain_id}")
    print(f"Current Speeds -> Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s\n")
    print("Ready! Press motion keys to drive...")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.08)
            if key == '\x03':  # CTRL+C
                break

            if key in MOVE_BINDINGS:
                v_mult, w_mult = MOVE_BINDINGS[key]
                target_v = v_mult * linear_speed
                target_w = w_mult * angular_speed
                print(f"\r[CMD] Linear: {target_v:+.2f} m/s | Angular: {target_w:+.2f} rad/s     ", end='', flush=True)

            elif key in SPEED_BINDINGS:
                dv, dw = SPEED_BINDINGS[key]
                linear_speed = max(0.05, min(1.20, linear_speed + dv))
                angular_speed = max(0.10, min(2.50, angular_speed + dw))
                print(f"\r[SETTINGS] Speed updated -> Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s  ", end='', flush=True)

            twist = Twist()
            twist.linear.x = float(target_v)
            twist.angular.z = float(target_w)
            pub.publish(twist)

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
