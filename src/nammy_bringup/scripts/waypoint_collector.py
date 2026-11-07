#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Interactive RViz Waypoint Collector & Autonomous Navigator
=============================================================================
Description:
  Subscribes to '/clicked_point' from RViz's 'Publish Point' tool.
  Collects multiple user-clicked waypoints, visualizes them with 3D markers,
  and dispatches sequential autonomous missions via Nav2 NavigateThroughPoses.

Usage:
  1. Click 'Publish Point' in RViz top toolbar and click anywhere on the map.
  2. Repeat to add multiple waypoints (WP 1, WP 2, WP 3...).
  3. In this terminal:
     - Press [ENTER] or [s] to Start autonomous navigation!
     - Press [c] to Clear all waypoints.
     - Press [u] to Undo / delete the last waypoint.
     - Press [l] to Toggle continuous patrol loop mode.

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
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
from std_msgs.msg import ColorRGBA


class WaypointCollectorNode(Node):
    def __init__(self):
        super().__init__('waypoint_collector')

        self.declare_parameter('loop_patrol', False)
        self.declare_parameter('pause_at_waypoint_sec', 1.0)

        self.loop_patrol = self.get_parameter('loop_patrol').value
        self.pause_sec = self.get_parameter('pause_at_waypoint_sec').value

        # Waypoints storage: list of (x, y)
        self.waypoints = []
        self.is_navigating = False
        self.current_goal_handle = None
        self.active_wp_idx = 0

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

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
        x = msg.point.x
        y = msg.point.y
        wp_num = len(self.waypoints) + 1
        self.waypoints.append((x, y))

        print(f"\n  [+] ADDED WAYPOINT #{wp_num}: X = {x:.2f} m, Y = {y:.2f} m")
        print(f"      Total Waypoints: {len(self.waypoints)}. Press [ENTER] to execute or continue clicking!")
        sys.stdout.flush()

        self.publish_markers()

    def undo_last_point(self):
        """Remove the last clicked waypoint."""
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
        if self.is_navigating and self.current_goal_handle:
            print("\n  [!] Cancelling active navigation mission...")
            self.current_goal_handle.cancel_goal_async()
            self.is_navigating = False

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
            # Sphere marker
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'waypoints_spheres'
            sphere.id = i
            sphere.type = Marker.CYLINDER
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.28
            sphere.scale.y = 0.28
            sphere.scale.z = 0.10

            if self.is_navigating and i == self.active_wp_idx:
                # Active target: Bright Green
                sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.90)
            elif self.is_navigating and i < self.active_wp_idx:
                # Completed target: Slate Blue
                sphere.color = ColorRGBA(r=0.3, g=0.5, b=0.9, a=0.60)
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
            text.pose.position.x = x
            text.pose.position.y = y
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
            line.scale.x = 0.05  # Line width
            line.color = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.85)  # Cyan
            for x, y in self.waypoints:
                p = Point(x=x, y=y, z=0.05)
                line.points.append(p)
            if self.loop_patrol and len(self.waypoints) > 2:
                # Close the loop
                p_close = Point(x=self.waypoints[0][0], y=self.waypoints[0][1], z=0.05)
                line.points.append(p_close)
            marker_array.markers.append(line)

        self.marker_pub.publish(marker_array)

    def start_navigation(self):
        """Dispatch the collected waypoints to Nav2 NavigateThroughPoses."""
        if not self.waypoints:
            print("\n  [!] No waypoints collected yet! Click 'Publish Point' in RViz first.")
            sys.stdout.flush()
            return

        if self.is_navigating:
            print("\n  [!] Navigation is already in progress!")
            sys.stdout.flush()
            return

        print(f"\n  [*] Connecting to Nav2 NavigateThroughPoses action server...")
        sys.stdout.flush()
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            print("\n  [ERROR] Nav2 action server 'navigate_through_poses' is not available!")
            sys.stdout.flush()
            return

        goal_msg = NavigateThroughPoses.Goal()
        now = self.get_clock().now().to_msg()
        num_wp = len(self.waypoints)

        for i in range(num_wp):
            x, y = self.waypoints[i]
            # Calculate heading pointing towards the next waypoint
            if i < num_wp - 1:
                next_x, next_y = self.waypoints[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif self.loop_patrol and num_wp > 1:
                # Final waypoint in loop points towards WP 1
                yaw = math.atan2(self.waypoints[0][1] - y, self.waypoints[0][0] - x)
            else:
                # Single or final waypoint: preserve orientation of approach
                if i > 0:
                    prev_x, prev_y = self.waypoints[i - 1]
                    yaw = math.atan2(y - prev_y, x - prev_x)
                else:
                    yaw = 0.0

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal_msg.poses.append(pose)

        print(f"  [*] Dispatching {num_wp} waypoints to Nav2...")
        self.is_navigating = True
        self.active_wp_idx = 0
        self.publish_markers()

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        curr_idx = getattr(feedback, 'current_pose', 0)
        dist = getattr(feedback, 'distance_remaining', 0.0)
        if curr_idx != self.active_wp_idx:
            self.active_wp_idx = curr_idx
            self.publish_markers()

        print(f"\r  >>> Navigating: Waypoint {curr_idx + 1}/{len(self.waypoints)} | Remaining: {dist:.2f} m   ", end="", flush=True)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            print("\n  [ERROR] Nav2 REJECTED the waypoint goal!")
            self.is_navigating = False
            return

        print("\n  [SUCCESS] Goal ACCEPTED by Nav2! Autonomous driving in progress...")
        sys.stdout.flush()
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.is_navigating = False
        print()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            print("  ================================================================")
            print("  [MISSION ACCOMPLISHED] Robot reached all waypoints successfully!")
            print("  ================================================================")
            if self.loop_patrol and self.waypoints:
                print(f"  Loop mode active: restarting patrol in {self.pause_sec:.1f}s...")
                time.sleep(self.pause_sec)
                self.start_navigation()
        elif result.status == GoalStatus.STATUS_CANCELED:
            print("  [MISSION CANCELED] Navigation was canceled.")
        else:
            print(f"  [MISSION TERMINATED] Navigation finished with status code {result.status}.")

        self.publish_markers()
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
  [u]            : Undo / remove last waypoint
  [c]            : Clear all waypoints
  [l]            : Toggle Patrol Loop mode (repeat continuously)
  [q]            : Exit
======================================================================
"""


def get_key(timeout=0.1):
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
