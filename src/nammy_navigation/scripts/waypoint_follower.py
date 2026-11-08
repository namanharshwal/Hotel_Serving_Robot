#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Autonomous Waypoint Navigation & Patrol
Interfaces with Nav2 NavigateThroughPoses and NavigateToPose action servers.
Enables sequential multi-waypoint navigation and continuous patrol loops.

Author: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from action_msgs.msg import GoalStatus


class NammyWaypointNavigator(Node):
    def __init__(self):
        super().__init__('nammy_waypoint_navigator')

        # Parameters
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('waypoint_pause_sec', 2.0)
        self.declare_parameter('action_server', 'navigate_through_poses')

        self.loop = self.get_parameter('loop_waypoints').value
        self.pause_duration = self.get_parameter('waypoint_pause_sec').value
        self.action_server_name = self.get_parameter('action_server').value

        self.get_logger().info("==================================================================")
        self.get_logger().info("NAMMY ROBOT - NAV2 WAYPOINT NAVIGATION SYSTEM")
        self.get_logger().info(f"Target Action: {self.action_server_name} | Loop Mode: {self.loop}")
        self.get_logger().info("==================================================================")

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateThroughPoses, self.action_server_name)
        
        # Room Waypoints for raj_sir_home_lab_room:
        # User can customize or provide via parameters
        self.waypoints = [
            (0.50, 0.00, 0.0),    # Waypoint 1: Forward into center
            (1.00, 0.20, 1.57),   # Waypoint 2: Upper quadrant
            (0.50, 0.50, 3.14),   # Waypoint 3: Left quadrant
            (0.00, 0.00, 0.0),    # Waypoint 4: Return to Home Dock
        ]

        self.current_waypoint_idx = 0
        self.goal_handle = None

    def start_navigation(self):
        """Wait for Nav2 action server and dispatch waypoints."""
        self.get_logger().info("Waiting for Nav2 NavigateThroughPoses action server...")
        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Nav2 action server not available! Make sure Nav2 is active.")
            return False

        self.get_logger().info(f"Nav2 server ready! Dispatching {len(self.waypoints)} waypoints...")
        goal_msg = NavigateThroughPoses.Goal()

        now = self.get_clock().now().to_msg()
        for idx, (x, y, yaw) in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # Convert Euler yaw to quaternion
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            goal_msg.poses.append(pose)
            self.get_logger().info(f"  [WP {idx + 1}] Target: X={x:.2f}m, Y={y:.2f}m, Yaw={math.degrees(yaw):.1f}°")

        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        wp_idx = getattr(feedback, 'current_pose', 0)
        dist = getattr(feedback, 'distance_remaining', 0.0)
        self.get_logger().info(
            f"Navigating -> Waypoint {wp_idx + 1}/{len(self.waypoints)} (Distance remaining: {dist:.2f}m)",
            throttle_duration_sec=3.0
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Waypoint navigation goal was REJECTED by Nav2!")
            return

        self.get_logger().info("Waypoint goal ACCEPTED! Autonomous transit in progress...")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("==================================================================")
            self.get_logger().info("[MISSION SUCCESS] All waypoints successfully reached!")
            self.get_logger().info("==================================================================")
            if self.loop:
                self.get_logger().info(f"Loop mode active: restarting patrol in {self.pause_duration:.1f}s...")
                time.sleep(self.pause_duration)
                self.start_navigation()
            else:
                self.get_logger().info("Mission complete. Standing by.")
        else:
            self.get_logger().warn(f"Navigation ended with status code: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = NammyWaypointNavigator()
    success = node.start_navigation()
    if success:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Waypoint navigation interrupted by user.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
