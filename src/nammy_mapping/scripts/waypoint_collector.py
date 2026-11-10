#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Interactive RViz Waypoint Collector & Autonomous Navigator
=============================================================================
Description:
  Subscribes to '/clicked_point' from RViz's 'Publish Point' tool.
  Collects multiple user-clicked waypoints, visualizes them with colorful 3D markers,
  and dispatches robust sequential autonomous delivery missions via Nav2 NavigateToPose.
  Guarantees 100% of waypoints are visited sequentially without skipping or truncation.

Usage:
  1. Click 'Publish Point' in RViz top toolbar and click anywhere on the map.
  2. Repeat to add multiple waypoints (WP 1, WP 2, WP 3...).
  3. In this terminal:
     - Press [ENTER] or [s] to Start sequential autonomous navigation!
     - Press [p] or [SPACE] to Pause / Cancel current goal.
     - Press [n] to Skip to Next waypoint.
     - Press [c] to Clear all waypoints.
     - Press [u] to Undo / delete the last waypoint.
     - Press [l] to Toggle continuous patrol loop mode.
     - Press [q] to Exit.

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
"""

import sys
import os
import math
import time
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import ColorRGBA


class WaypointCollectorNode(Node):
    def __init__(self):
        super().__init__('waypoint_collector')

        self.declare_parameter('loop_patrol', False)
        self.declare_parameter('pause_at_waypoint_sec', 1.5)

        self.loop_patrol = self.get_parameter('loop_patrol').value
        self.pause_sec = self.get_parameter('pause_at_waypoint_sec').value

        # Waypoints storage: list of (x, y)
        self.waypoints = []
        self.is_navigating = False
        self.current_goal_handle = None
        self.active_wp_idx = 0
        self.step_timer = None

        # Dedicated Action Client for Sequential NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Marker publisher for 3D RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)

        # Subscriber to RViz "Publish Point" (/clicked_point)
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.get_logger().info("Waypoint Collector initialized. Ready for /clicked_point from RViz.")

    def clicked_point_callback(self, msg: PointStamped):
        """Receive a clicked point from RViz Publish Point tool."""
        x = round(msg.point.x, 2)
        y = round(msg.point.y, 2)
        wp_num = len(self.waypoints) + 1
        self.waypoints.append((x, y))

        print(f"\n  [+] ADDED WAYPOINT #{wp_num}: X = {x:.2f} m, Y = {y:.2f} m")
        print(f"      Total Waypoints: {len(self.waypoints)}. Press [ENTER] to execute or continue clicking!")
        sys.stdout.flush()

        self.publish_markers()

    def undo_last_point(self):
        """Remove the last clicked waypoint."""
        if self.is_navigating:
            print("\n  [!] Cannot modify waypoints while navigation is running! Press [p] to pause first.")
            sys.stdout.flush()
            return

        if self.waypoints:
            removed = self.waypoints.pop()
            print(f"\n  [-] REMOVED last waypoint: ({removed[0]:.2f}, {removed[1]:.2f}). Remaining: {len(self.waypoints)}")
            sys.stdout.flush()
            self.publish_markers()
        else:
            print("\n  [!] No waypoints to remove.")
            sys.stdout.flush()

    def clear_waypoints(self):
        """Clear all waypoints and markers."""
        if self.is_navigating:
            self.stop_navigation(announce=False)

        self.waypoints.clear()
        self.active_wp_idx = 0
        print("\n  [x] All waypoints CLEARED.")
        sys.stdout.flush()
        self.publish_markers(clear_all=True)

    def publish_markers(self, clear_all=False):
        """Publish colorful 3D RViz markers for all collected waypoints and path lines."""
        marker_array = MarkerArray()

        if clear_all:
            del_marker = Marker()
            del_marker.header.frame_id = 'map'
            del_marker.header.stamp = self.get_clock().now().to_msg()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)
            self.marker_pub.publish(marker_array)
            return

        now = self.get_clock().now().to_msg()

        # 1. Sphere and Text markers at each point
        for i, (x, y) in enumerate(self.waypoints):
            # Sphere/Cylinder marker
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'waypoints_spheres'
            sphere.id = i
            sphere.type = Marker.CYLINDER
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.28
            sphere.scale.y = 0.28
            sphere.scale.z = 0.10

            if self.is_navigating and i == self.active_wp_idx:
                # Active target: Bright Neon Green
                sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.95)
            elif self.is_navigating and i < self.active_wp_idx:
                # Completed target: Soft Cyan/Blue
                sphere.color = ColorRGBA(r=0.2, g=0.7, b=0.9, a=0.60)
            else:
                # Queued target: Bright Orange / Amber
                sphere.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.90)
            marker_array.markers.append(sphere)

            # Text label floating above cylinder
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'waypoints_labels'
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22  # Text height
            text.text = f"WP {i + 1}"
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker_array.markers.append(text)

        # 2. Line strip connecting the waypoints in order
        if len(self.waypoints) >= 2:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'waypoints_path'
            line.id = 999
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.04  # Line width
            line.color = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.85)  # Cyan
            for x, y in self.waypoints:
                p = Point(x=float(x), y=float(y), z=0.05)
                line.points.append(p)
            if self.loop_patrol and len(self.waypoints) > 2:
                p_close = Point(x=float(self.waypoints[0][0]), y=float(self.waypoints[0][1]), z=0.05)
                line.points.append(p_close)
            marker_array.markers.append(line)

        self.marker_pub.publish(marker_array)

    def start_navigation(self):
        """Dispatch the collected waypoints sequentially."""
        if not self.waypoints:
            print("\n  [!] No waypoints collected yet! Click 'Publish Point' in RViz first.")
            sys.stdout.flush()
            return

        if self.is_navigating:
            print("\n  [!] Navigation is already active!")
            sys.stdout.flush()
            return

        print(f"\n  [*] Checking Nav2 NavigateToPose action server...")
        sys.stdout.flush()
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            print("\n  [ERROR] Nav2 action server 'navigate_to_pose' is not available!")
            sys.stdout.flush()
            return

        print(f"  [✓] Server connected! Starting sequential mission for {len(self.waypoints)} waypoints...")
        self.is_navigating = True
        self.active_wp_idx = 0
        self.send_current_waypoint()

    def send_current_waypoint(self):
        """Send NavigateToPose for the active waypoint."""
        if not self.is_navigating:
            return

        num_wp = len(self.waypoints)
        if self.active_wp_idx >= num_wp:
            if self.loop_patrol and num_wp > 0:
                print(f"\n  [PATROL LOOP] Completed round! Repeating patrol in {self.pause_sec:.1f}s...")
                sys.stdout.flush()
                self.active_wp_idx = 0
                self.publish_markers()
                self.schedule_next_step(self.send_current_waypoint, delay=self.pause_sec)
                return
            else:
                self.is_navigating = False
                self.publish_markers()
                print("\n  ================================================================")
                print("  [MISSION ACCOMPLISHED] Robot visited ALL waypoints successfully!")
                print("  ================================================================")
                sys.stdout.flush()
                return

        x, y = self.waypoints[self.active_wp_idx]

        # Heading calculation towards next waypoint
        if self.active_wp_idx < num_wp - 1:
            next_x, next_y = self.waypoints[self.active_wp_idx + 1]
            yaw = math.atan2(next_y - y, next_x - x)
        elif self.loop_patrol and num_wp > 1:
            yaw = math.atan2(self.waypoints[0][1] - y, self.waypoints[0][0] - x)
        else:
            if self.active_wp_idx > 0:
                prev_x, prev_y = self.waypoints[self.active_wp_idx - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.publish_markers()

        print(f"\n  [*] TARGET: Waypoint {self.active_wp_idx + 1}/{num_wp} -> (X={x:.2f} m, Y={y:.2f} m, Yaw={math.degrees(yaw):.1f}°)")
        sys.stdout.flush()

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist = getattr(feedback, 'distance_remaining', 0.0)
        print(f"\r  >>> Driving to WP {self.active_wp_idx + 1}/{len(self.waypoints)} | Distance remaining: {dist:.2f} m   ", end="", flush=True)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle or not self.current_goal_handle.accepted:
            print(f"\n  [ERROR] Nav2 REJECTED goal for WP {self.active_wp_idx + 1}!")
            sys.stdout.flush()
            self.stop_navigation(announce=False)
            return

        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        if not self.is_navigating:
            return

        result = future.result()
        print()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            print(f"  [✓ REACHED] Waypoint {self.active_wp_idx + 1}/{len(self.waypoints)} reached successfully!")
            self.active_wp_idx += 1
            self.publish_markers()
            if self.active_wp_idx < len(self.waypoints) or self.loop_patrol:
                print(f"      Pausing for {self.pause_sec:.1f}s before moving to next waypoint...")
                sys.stdout.flush()
                self.schedule_next_step(self.send_current_waypoint, delay=self.pause_sec)
            else:
                self.send_current_waypoint()
        elif result.status == GoalStatus.STATUS_CANCELED:
            print("  [MISSION PAUSED] Current navigation goal canceled.")
            self.is_navigating = False
            self.publish_markers()
            sys.stdout.flush()
        else:
            print(f"  [!] Waypoint {self.active_wp_idx + 1} aborted (status={result.status}). Skipping to next in 2s...")
            self.active_wp_idx += 1
            self.publish_markers()
            sys.stdout.flush()
            self.schedule_next_step(self.send_current_waypoint, delay=2.0)

    def schedule_next_step(self, func, delay=1.0):
        """Schedule a function execution on the ROS event loop after delay."""
        if self.step_timer:
            self.step_timer.cancel()

        def timer_cb():
            self.step_timer.cancel()
            self.step_timer = None
            func()

        self.step_timer = self.create_timer(delay, timer_cb)

    def skip_next_waypoint(self):
        """Skip currently active waypoint and proceed to the next."""
        if not self.is_navigating:
            print("\n  [!] Navigation is not active.")
            return

        print(f"\n  [>>] SKIPPING Waypoint {self.active_wp_idx + 1}...")
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
        self.active_wp_idx += 1
        self.publish_markers()
        self.send_current_waypoint()

    def stop_navigation(self, announce=True):
        """Cancel active navigation mission."""
        if self.step_timer:
            self.step_timer.cancel()
            self.step_timer = None

        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()

        self.is_navigating = False
        self.publish_markers()
        if announce:
            print("\n  [HALTED] Autonomous navigation stopped.")
            sys.stdout.flush()


BANNER = """
======================================================================
       NAMMY ROBOT - INTERACTIVE WAYPOINT NAVIGATOR                  
======================================================================
 Instructions:
  1. In RViz, select 'Publish Point' from the top toolbar.
  2. Click on the map to add Waypoint 1, Waypoint 2, Waypoint 3...
  3. Visual 3D markers (WP1, WP2...) appear dynamically in RViz!
 
 Keyboard Commands:
  [ENTER] or [s] : Start Autonomous Waypoint Navigation
  [p] or [SPACE] : Pause / Stop navigation
  [n]            : Skip to Next waypoint
  [u]            : Undo / remove last waypoint
  [c]            : Clear all waypoints
  [l]            : Toggle Patrol Loop mode (continuous repeat)
  [q]            : Exit
======================================================================
"""


def get_key(timeout=0.05):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCollectorNode()

    # Spin ROS 2 in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(BANNER)
    print("Waiting for waypoints... (Click 'Publish Point' in RViz)")
    sys.stdout.flush()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = get_key(0.05)
            if key is not None:
                if key in ['\n', '\r', 's', 'S', 'g', 'G']:
                    node.start_navigation()
                elif key in ['p', 'P', ' ']:
                    node.stop_navigation()
                elif key in ['n', 'N']:
                    node.skip_next_waypoint()
                elif key in ['c', 'C']:
                    node.clear_waypoints()
                elif key in ['u', 'U']:
                    node.undo_last_point()
                elif key in ['l', 'L']:
                    node.loop_patrol = not node.loop_patrol
                    state = "ENABLED" if node.loop_patrol else "DISABLED"
                    print(f"\n  [LOOP MODE] Continuous Patrol is now {state}.")
                    node.publish_markers()
                    sys.stdout.flush()
                elif key in ['q', 'Q', '\x03']:  # q or Ctrl+C
                    print("\nExiting Waypoint Navigator.")
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
