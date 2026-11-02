# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/plant_pace_talker.py

Criterion 3 for P11, inside a ROS container on the desktop (outside the ROS
wall, as ros_pace.py): a stand-in for nav2's controller and the bridge's
scans, which `ros_pace.py` records and `scorers.controller_rate()` /
`scorers.scan_lag()` read.

* `/follow_path/_action/status`: one goal EXECUTING for the run, then
  SUCCEEDED;
* `cmd_vel/nav` at 20 Hz (a 50 ms loop, as `controller_frequency`), each
  cycle `PLANT` s longer (100 ms planted: a 150 ms period);
* `/scan` at 10 Hz, stamped with its capture time and published `PLANT` s
  after it (the lag planted).

    python3 plant_pace_talker.py <seconds> <plant_s>
"""

import sys
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp(t):
    return Time(sec=int(t), nanosec=int((t % 1) * 1e9))


def main():
    seconds, plant = float(sys.argv[1]), float(sys.argv[2])
    rclpy.init()
    n = Node("plant_pace_talker")
    cmd = n.create_publisher(Twist, "cmd_vel/nav", 50)
    fp = n.create_publisher(GoalStatusArray, "/follow_path/_action/status", 10)
    scan = n.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    gid = list(uuid.uuid4().bytes)

    def status(s):
        g = GoalStatus(status=s)
        g.goal_info.goal_id.uuid = gid
        fp.publish(GoalStatusArray(status_list=[g]))

    time.sleep(2.0)                     # discovery
    status(GoalStatus.STATUS_EXECUTING)
    end = time.time() + seconds

    def scans():
        while time.time() < end:
            t = time.time()
            m = LaserScan()
            m.header.frame_id = "laser"
            m.header.stamp = stamp(t)
            time.sleep(plant)
            scan.publish(m)
            time.sleep(max(0.0, 0.1 - (time.time() - t)))
    th = threading.Thread(target=scans, daemon=True)
    th.start()
    nxt = time.time()
    while time.time() < end:
        cmd.publish(Twist())
        nxt += 0.05 + plant
        time.sleep(max(0.0, nxt - time.time()))
    th.join(timeout=2)
    status(GoalStatus.STATUS_SUCCEEDED)
    time.sleep(1.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
