#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - SLAM Map Saver Utility (nammy_mapping)
=============================================================================
Description:
  Saves the current occupancy grid map from the /map topic to disk.
  Generates both the YAML metadata and PGM image files into the workspace maps/ directory.

Usage:
  ros2 run nammy_mapping save_map.py [map_name]
  Example:
    ros2 run nammy_mapping save_map.py room_map

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
"""

import sys
import os
import subprocess
from datetime import datetime


def main():
    home_dir = os.path.expanduser('~')
    maps_dir = os.path.join(home_dir, 'Documents', 'nammy_ws', 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        map_name = sys.argv[1]
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_name = f"map_{timestamp}"

    map_path = os.path.join(maps_dir, map_name)

    print("======================================================================")
    print("               NAMMY ROBOT - SLAM MAP SAVER UTILITY                   ")
    print("======================================================================")
    print(f" Saving active /map to: {map_path}.yaml / .pgm")
    print("======================================================================")

    cmd = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', map_path,
        '--ros-args',
        '-p', 'save_map_timeout:=15.0',
        '-p', 'free_thresh_default:=0.25',
        '-p', 'occupied_thresh_default:=0.65'
    ]

    try:
        res = subprocess.run(cmd, check=True)
        if res.returncode == 0:
            print("")
            print(f"[✓] SUCCESS: Map saved successfully to:")
            print(f"    - {map_path}.yaml")
            print(f"    - {map_path}.pgm")
    except subprocess.CalledProcessError as e:
        print(f"[!] ERROR: Failed to save map (exit code {e.returncode}). Is SLAM running and publishing /map?")
        sys.exit(e.returncode)
    except FileNotFoundError:
        print("[!] ERROR: 'ros2' command not found. Please ensure ROS 2 Humble is sourced.")
        sys.exit(1)


if __name__ == '__main__':
    main()
