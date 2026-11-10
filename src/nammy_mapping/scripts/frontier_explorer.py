#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Autonomous Frontier Exploration Node (nammy_mapping)
=============================================================================
Description:
  Fully autonomous frontier-based exploration and room mapping for the
  Nammy Robot Platform.
  
  Implements Yamauchi's frontier detection algorithm optimized for 2D LiDAR
  and ultrasonic sensory fusion:
    1. Subscribes to dynamic SLAM occupancy grid map (/map).
    2. Identifies frontier cells at the boundary of safe free space and unknown space.
    3. Inflates obstacles by robot footprint clearance to ensure collision-free goals.
    4. Clusters contiguous frontier cells using scipy.ndimage connected components.
    5. Calculates frontier centroids and directional orientation pointing into unknown space.
    6. Scores frontier candidates by distance, information gain, and reachability.
    7. Dispatches autonomous navigation goals to Nav2 NavigateToPose action server.
    8. Handles dynamic replanning, blacklisting unreachable targets, and automatic
       return-to-dock / home upon 100% room exploration completion.
    9. Publishes vibrant 3D visualization markers to RViz (/exploration/frontiers).

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
"""

import sys
import os
import math
import time
import threading
import select
import termios
import tty
from typing import List, Tuple, Optional, Dict

import numpy as np
import scipy.ndimage

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, ColorRGBA


class FrontierCandidate:
    def __init__(self, centroid: Tuple[float, float], yaw: float, size: int, points: List[Tuple[float, float]]):
        self.x = centroid[0]
        self.y = centroid[1]
        self.yaw = yaw
        self.size = size
        self.points = points
        self.score = 0.0


class AutonomousFrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('min_frontier_size', 5)         # Minimum cells in a cluster (~25 cm)
        self.declare_parameter('robot_radius_cells', 5)        # 5 cells * 0.05m = 0.25m obstacle inflation
        self.declare_parameter('min_goal_distance', 0.40)      # Don't pick frontiers closer than 40cm to current pose
        self.declare_parameter('max_goal_distance', 15.0)      # Maximum exploration radius in meters
        self.declare_parameter('blacklist_duration_sec', 45.0) # Blacklist timeout for unreachable targets
        self.declare_parameter('replan_interval_sec', 3.0)     # Periodic frontier check interval
        self.declare_parameter('auto_start', True)             # Automatically start exploration on launch
        self.declare_parameter('return_home_on_finish', True)  # Return to starting dock when all frontiers mapped
        self.declare_parameter('map_save_name', 'raj_sir_lab_room_1')  # Target map filename

        self.min_size = self.get_parameter('min_frontier_size').value
        self.robot_radius_cells = self.get_parameter('robot_radius_cells').value
        self.min_dist = self.get_parameter('min_goal_distance').value
        self.max_dist = self.get_parameter('max_goal_distance').value
        self.blacklist_duration = self.get_parameter('blacklist_duration_sec').value
        self.replan_interval = self.get_parameter('replan_interval_sec').value
        self.auto_start = self.get_parameter('auto_start').value
        self.return_home = self.get_parameter('return_home_on_finish').value
        self.map_save_name = self.get_parameter('map_save_name').value

        # TF Buffer and Listener for robot localization
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State Variables
        self.current_map: Optional[OccupancyGrid] = None
        self.frontiers: List[FrontierCandidate] = []
        self.active_target: Optional[FrontierCandidate] = None
        self.home_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.blacklisted_targets: Dict[Tuple[float, float], float] = {}  # (x,y) -> timestamp
        
        self.state = "INITIALIZING"  # INITIALIZING, IDLE, EXPLORING, NAVIGATING, PAUSED, RETURNING_HOME, COMPLETED
        self.is_exploring = False
        self.nav_goal_handle = None
        self.mapped_area_sqm = 0.0
        self.goals_completed = 0
        self.last_goal_time = 0.0

        # Subscriptions & Publishers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        self.status_pub = self.create_publisher(String, '/exploration/status', 10)

        # Exploration Decision Timer (1 Hz)
        self.timer = self.create_timer(1.0, self.exploration_cycle)

        self.get_logger().info("Autonomous Frontier Explorer initialized. Waiting for /map & Nav2...")

    def map_callback(self, msg: OccupancyGrid):
        """Cache incoming occupancy grid map from SLAM Toolbox."""
        self.current_map = msg
        # Estimate mapped area
        data = np.array(msg.data)
        free_cells = np.count_nonzero((data >= 0) & (data < 50))
        self.mapped_area_sqm = free_cells * (msg.info.resolution ** 2)

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Get current robot pose (x, y, yaw) in map frame via TF."""
        try:
            now = rclpy.time.Time()
            tf_stamped = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                now,
                timeout=Duration(seconds=0.2)
            )
            trans = tf_stamped.transform.translation
            rot = tf_stamped.transform.rotation

            # Compute yaw from quaternion
            siny_cosp = 2.0 * (rot.w * rot.z + rot.x * rot.y)
            cosy_cosp = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return (trans.x, trans.y, yaw)
        except Exception:
            return None

    def detect_frontiers(self) -> List[FrontierCandidate]:
        """Yamauchi frontier detection with obstacle inflation and cluster extraction."""
        if self.current_map is None:
            return []

        info = self.current_map.info
        w = info.width
        h = info.height
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        if w < 10 or h < 10:
            return []

        grid = np.array(self.current_map.data, dtype=np.int8).reshape((h, w))

        # Binary masks
        unknown = (grid == -1)
        free = (grid >= 0) & (grid < 25)
        obstacles = (grid >= 50)

        # Inflate obstacles by robot clearance
        if self.robot_radius_cells > 0:
            dilated_obstacles = scipy.ndimage.binary_dilation(
                obstacles, iterations=self.robot_radius_cells
            )
        else:
            dilated_obstacles = obstacles

        safe_free = free & (~dilated_obstacles)

        # Frontier cells: in safe free space with at least one 8-neighbor in unknown space
        unknown_dilated = scipy.ndimage.binary_dilation(unknown, structure=np.ones((3, 3)))
        frontier_mask = safe_free & unknown_dilated

        # Connected component labeling for clustering
        labeled_mask, num_features = scipy.ndimage.label(frontier_mask, structure=np.ones((3, 3)))

        candidates: List[FrontierCandidate] = []
        now = time.time()

        for label in range(1, num_features + 1):
            cluster_indices = np.argwhere(labeled_mask == label)
            if len(cluster_indices) < self.min_size:
                continue

            # World coordinates of cluster cells: row -> y, col -> x
            xs = cluster_indices[:, 1] * res + origin_x
            ys = cluster_indices[:, 0] * res + origin_y

            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            # Check if this centroid is blacklisted
            is_blacklisted = False
            for (bx, by), btime in list(self.blacklisted_targets.items()):
                if now - btime > self.blacklist_duration:
                    del self.blacklisted_targets[(bx, by)]
                elif math.hypot(cx - bx, cy - by) < 0.45:
                    is_blacklisted = True
                    break

            if is_blacklisted:
                continue

            # Find cell in cluster closest to centroid that is safe
            dists_sq = (xs - cx) ** 2 + (ys - cy) ** 2
            best_idx = np.argmin(dists_sq)
            safe_cx = float(xs[best_idx])
            safe_cy = float(ys[best_idx])

            # Determine vector pointing towards unknown space for LiDAR sweep
            best_r = cluster_indices[best_idx, 0]
            best_c = cluster_indices[best_idx, 1]

            # Sample 5x5 window around candidate cell to find center of unknown cells
            r_min = max(0, best_r - 2)
            r_max = min(h, best_r + 3)
            c_min = max(0, best_c - 2)
            c_max = min(w, best_c + 3)

            unk_patch = unknown[r_min:r_max, c_min:c_max]
            unk_coords = np.argwhere(unk_patch)
            if len(unk_coords) > 0:
                avg_unk_r = np.mean(unk_coords[:, 0]) - 2.0
                avg_unk_c = np.mean(unk_coords[:, 1]) - 2.0
                yaw = math.atan2(avg_unk_r, avg_unk_c)
            else:
                yaw = 0.0

            pts = list(zip(xs.tolist(), ys.tolist()))
            candidate = FrontierCandidate(centroid=(safe_cx, safe_cy), yaw=yaw, size=len(cluster_indices), points=pts)
            candidates.append(candidate)

        return candidates

    def rank_frontiers(self, candidates: List[FrontierCandidate], robot_pose: Tuple[float, float, float]) -> List[FrontierCandidate]:
        """Score frontiers by size, distance, and angular alignment."""
        rx, ry, ryaw = robot_pose

        for c in candidates:
            dist = math.hypot(c.x - rx, c.y - ry)
            if dist < self.min_dist or dist > self.max_dist:
                c.score = -1.0
                continue

            # Heading difference
            angle_to_goal = math.atan2(c.y - ry, c.x - rx)
            angle_diff = abs(math.atan2(math.sin(angle_to_goal - ryaw), math.cos(angle_to_goal - ryaw)))

            # Score formula: Information gain divided by travel cost
            # Prioritizes nearby clusters with substantial unknown coverage
            c.score = (c.size * 2.0) / (1.0 + 1.5 * dist + 0.5 * angle_diff)

        # Filter out invalid distances and sort descending
        valid = [c for c in candidates if c.score > 0.0]
        valid.sort(key=lambda c: c.score, reverse=True)
        return valid

    def exploration_cycle(self):
        """Main autonomous exploration decision loop."""
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        # Store initial starting pose as dock / home position
        if self.home_pose is None:
            self.home_pose = robot_pose
            self.get_logger().info(f"Home Dock pose recorded: ({self.home_pose[0]:.2f}, {self.home_pose[1]:.2f})")

        # Auto-start exploration if enabled
        if self.auto_start and not self.is_exploring and self.state == "INITIALIZING":
            self.is_exploring = True
            self.state = "EXPLORING"
            self.get_logger().info("Autonomous Exploration initiated automatically!")

        if not self.is_exploring:
            return

        # Detect frontiers
        self.frontiers = self.detect_frontiers()
        ranked = self.rank_frontiers(self.frontiers, robot_pose)
        self.publish_markers(self.frontiers, self.active_target)

        # Publish status string
        status_msg = String()
        status_msg.data = f"State: {self.state} | Frontiers: {len(ranked)} | Mapped: {self.mapped_area_sqm:.1f}m2"
        self.status_pub.publish(status_msg)

        # If currently navigating to a goal, check if we arrived or need replanning
        if self.state == "NAVIGATING":
            if self.active_target:
                dist_to_active = math.hypot(self.active_target.x - robot_pose[0], self.active_target.y - robot_pose[1])
                # If robot is already within 0.35m of target, consider goal achieved
                if dist_to_active < 0.35:
                    self.get_logger().info(f"Frontier reached within proximity ({dist_to_active:.2f}m)!")
                    self.on_goal_succeeded()
            return

        # Check for completion: No frontiers left
        if len(ranked) == 0:
            if self.goals_completed > 0:
                self.get_logger().info("=" * 60)
                self.get_logger().info("EXPLORATION COMPLETE: All reachable frontiers fully mapped!")
                self.get_logger().info("=" * 60)
                self.state = "COMPLETED"
                self.is_exploring = False
                self.save_map()

                if self.return_home and self.home_pose:
                    self.get_logger().info("Returning autonomously to Home Dock...")
                    self.dispatch_goal(self.home_pose[0], self.home_pose[1], self.home_pose[2], is_home=True)
            return

        # Pick highest scoring candidate
        best = ranked[0]
        self.active_target = best
        self.dispatch_goal(best.x, best.y, best.yaw)

    def dispatch_goal(self, x: float, y: float, yaw: float, is_home: bool = False):
        """Send 2D goal pose to Nav2 NavigateToPose action server."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 NavigateToPose action server not available. Retrying...")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Orientation quaternion from yaw
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.state = "RETURNING_HOME" if is_home else "NAVIGATING"
        target_name = "Home Dock" if is_home else f"Frontier at ({x:.2f}, {y:.2f})"
        self.get_logger().info(f"Dispatching autonomous goal to {target_name} (yaw: {math.degrees(yaw):.1f}°)")

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance by Nav2 server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected frontier goal. Selecting alternative candidate...")
            self.on_goal_failed()
            return

        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle goal completion status from Nav2."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.on_goal_succeeded()
        else:
            self.on_goal_failed()

    def on_goal_succeeded(self):
        """Handle successful arrival at frontier or return home."""
        if self.state == "RETURNING_HOME":
            self.state = "DOCK_REACHED"
            self.is_exploring = False
            self.active_target = None
            self.get_logger().info("=" * 60)
            self.get_logger().info("MISSION SUCCESS: Robot has returned to its initial position!")
            self.get_logger().info("=" * 60)
            self.save_map()
            return

        self.goals_completed += 1
        self.last_goal_time = time.time()
        self.active_target = None
        self.state = "EXPLORING"
        self.get_logger().info(f"Target successfully explored! Total goals completed: {self.goals_completed}")

    def save_map(self):
        """Save the current SLAM map directly to maps/ directory."""
        import subprocess
        maps_dir = os.path.expanduser('~/Documents/nammy_ws/maps')
        os.makedirs(maps_dir, exist_ok=True)
        name = self.get_parameter('map_save_name').value
        map_path = os.path.join(maps_dir, name)
        self.get_logger().info(f"[MAP SAVER] Saving map to: {map_path}.yaml / .pgm")
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path,
            '--ros-args',
            '-p', 'save_map_timeout:=15.0',
            '-p', 'free_thresh_default:=0.25',
            '-p', 'occupied_thresh_default:=0.65'
        ]
        try:
            subprocess.Popen(cmd)
            self.get_logger().info(f"[MAP SAVER] Successfully dispatched map save for '{name}'!")
        except Exception as e:
            self.get_logger().error(f"[MAP SAVER] Error triggering map saver: {e}")

    def on_goal_failed(self):
        """Blacklist failed frontier and select alternative."""
        if self.state == "RETURNING_HOME":
            self.get_logger().warn("Return to dock goal failed. Holding current position.")
            self.state = "COMPLETED"
            self.is_exploring = False
            return

        if self.active_target:
            pos = (round(self.active_target.x, 2), round(self.active_target.y, 2))
            self.blacklisted_targets[pos] = time.time()
            self.get_logger().warn(f"Frontier at {pos} unreachable. Blacklisted for {self.blacklist_duration:.0f}s.")
        self.active_target = None
        self.state = "EXPLORING"

    def publish_markers(self, frontiers: List[FrontierCandidate], active: Optional[FrontierCandidate]):
        """Publish colorful 3D visualization markers to RViz."""
        marker_array = MarkerArray()

        # Delete all previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # 1. Frontier Centroid Spheres
        for idx, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = idx + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.x
            m.pose.position.y = f.y
            m.pose.position.z = 0.15

            # Size scale based on information gain
            scale = min(0.35, max(0.12, f.size * 0.015))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            # Color: Cyan / Yellow for candidate frontiers
            m.color.r = 0.0
            m.color.g = 0.85
            m.color.b = 1.0
            m.color.a = 0.75
            marker_array.markers.append(m)

        # 2. Active Target Marker (Bright Green Sphere + Arrow)
        if active:
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'active_target'
            target_marker.id = 1000
            target_marker.type = Marker.CYLINDER
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = active.x
            target_marker.pose.position.y = active.y
            target_marker.pose.position.z = 0.25
            target_marker.scale.x = 0.35
            target_marker.scale.y = 0.35
            target_marker.scale.z = 0.50
            target_marker.color.r = 0.0
            target_marker.color.g = 1.0
            target_marker.color.b = 0.2
            target_marker.color.a = 0.90
            marker_array.markers.append(target_marker)

            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'active_heading'
            arrow.id = 1001
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = active.x
            arrow.pose.position.y = active.y
            arrow.pose.position.z = 0.25
            arrow.pose.orientation.z = math.sin(active.yaw / 2.0)
            arrow.pose.orientation.w = math.cos(active.yaw / 2.0)
            arrow.scale.x = 0.50
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color.r = 1.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 0.90
            marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)


def run_interactive_ui(node: AutonomousFrontierExplorer):
    """Terminal interactive dashboard for operator control."""
    def get_key():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    time.sleep(2.0)
    last_print = 0.0

    while rclpy.ok():
        now = time.time()
        if now - last_print >= 0.5:
            last_print = now
            robot_pose = node.get_robot_pose()
            rx = robot_pose[0] if robot_pose else 0.0
            ry = robot_pose[1] if robot_pose else 0.0

            os.system('clear' if os.name == 'posix' else 'cls')
            print("\033[96;1m" + "=" * 74 + "\033[0m")
            print("\033[93;1m      NAMMY ROBOT - FULLY AUTONOMOUS FRONTIER EXPLORATION SUITE       \033[0m")
            print("\033[90m        Maintainer: Naman Harshwal <namanharshwal@gmail.com>          \033[0m")
            print("\033[96;1m" + "=" * 74 + "\033[0m")

            state_color = "\033[92;1m" if node.state in ("EXPLORING", "NAVIGATING") else "\033[93;1m"
            print(f" • \033[1mEXPLORATION STATE:\033[0m  {state_color}{node.state:<16}\033[0m  • \033[1mAUTO-START:\033[0m \033[92mENABLED\033[0m")
            print(f" • \033[1mROBOT POSE (MAP):\033[0m   X: {rx:+5.2f} m | Y: {ry:+5.2f} m")
            print(f" • \033[1mMAPPED AREA:\033[0m        \033[92;1m{node.mapped_area_sqm:5.1f} m²\033[0m  (Free traversable space)")
            print(f" • \033[1mACTIVE FRONTIERS:\033[0m   \033[93;1m{len(node.frontiers)} clusters detected\033[0m")
            print(f" • \033[1mGOALS COMPLETED:\033[0m    \033[92;1m{node.goals_completed}\033[0m goals visited")

            if node.active_target:
                tgt = node.active_target
                d = math.hypot(tgt.x - rx, tgt.y - ry)
                print(f" • \033[1mCURRENT TARGET:\033[0m     \033[92mX: {tgt.x:+5.2f}m, Y: {tgt.y:+5.2f}m (dist: {d:.2f}m, yaw: {math.degrees(tgt.yaw):+.0f}°)\033[0m")
            else:
                print(f" • \033[1mCURRENT TARGET:\033[0m     \033[90mSearching for highest-gain frontier...\033[0m")

            if len(node.blacklisted_targets) > 0:
                print(f" • \033[1mBLACKLISTED TARGETS:\033[0m\033[91m{len(node.blacklisted_targets)} unreachable points temporarily bypassed\033[0m")

            print("\033[96;1m" + "-" * 74 + "\033[0m")
            print(" \033[1mKEYBOARD CONTROLS:\033[0m")
            print("   [\033[92;1ms\033[0m] Start / Resume Autonomous Exploration")
            print("   [\033[93;1mp\033[0m] Pause Exploration (Hold Position)")
            print("   [\033[94;1mr\033[0m] Return to Starting Home Dock Pose")
            print(f"   [\033[92;1mm\033[0m] Save Map Now -> \033[1m{node.map_save_name}.yaml\033[0m")
            print("   [\033[95;1mc\033[0m] Clear Blacklist & Re-evaluate Frontiers")
            print("   [\033[91;1mq\033[0m] Quit Exploration Node")
            print("\033[96;1m" + "=" * 74 + "\033[0m")

        key = get_key()
        if key == 's':
            node.is_exploring = True
            node.state = "EXPLORING"
        elif key == 'p':
            node.is_exploring = False
            node.state = "PAUSED"
            if node.nav_goal_handle:
                node.nav_goal_handle.cancel_goal_async()
        elif key == 'r':
            node.is_exploring = False
            if node.home_pose:
                node.dispatch_goal(node.home_pose[0], node.home_pose[1], node.home_pose[2], is_home=True)
        elif key == 'm':
            node.save_map()
        elif key == 'c':
            node.blacklisted_targets.clear()
        elif key == 'q':
            if node.nav_goal_handle:
                node.nav_goal_handle.cancel_goal_async()
            rclpy.shutdown()
            break

        time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousFrontierExplorer()

    # Run terminal UI in a daemon thread
    ui_thread = threading.Thread(target=run_interactive_ui, args=(node,), daemon=True)
    ui_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
