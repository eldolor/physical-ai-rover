# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/ros_pace.py

3.136.B's P11 recorder. Runs INSIDE the ROS container (copied in with
`docker cp`, started with `docker exec` under the image's ROS setup), so the
ROS wall (tests/test_ros_containment.py) holds: evaluations/ is outside the
system and nothing imports this file. It only listens; it publishes nothing.

One JSON line per message, receipt time `t` from the host's wall clock (the
container runs on the host network and clock):

| k     | topic                                 | fields            | for |
|-------|---------------------------------------|-------------------|-----|
| cmd   | `cmd_vel/nav` (controller, behaviours)| t                 | P11 controller rate |
| fp    | `/follow_path/_action/status`         | t, goals [[id, s]]| P11: when the controller is executing |
| nav   | `/navigate_to_pose/_action/status`    | t, goals [[id, s]]| validity: nav2's goals' ends |
| scan  | `/scan`                               | t, stamp          | P11 scan lag (receipt - header stamp) |
| map   | `/map`                                | t, stamp          | P4: a SLAM map update |

    python3 ros_pace.py <out.jsonl> [seconds]

Ends on SIGTERM/SIGINT or after `seconds`.
"""

import json
import signal
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan


def _stamp(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _gid(g) -> str:
    return bytes(g.goal_info.goal_id.uuid).hex()[:12]


class Pace(Node):
    def __init__(self, out):
        super().__init__("perf_3136_pace")
        self.out = out
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Twist, "cmd_vel/nav", lambda m: self.row({"k": "cmd"}), 50)
        self.create_subscription(GoalStatusArray, "/follow_path/_action/status",
                                 lambda m: self.row({"k": "fp", "goals": [[_gid(g), g.status]
                                                                          for g in m.status_list]}),
                                 status_qos)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 lambda m: self.row({"k": "nav", "goals": [[_gid(g), g.status]
                                                                           for g in m.status_list]}),
                                 status_qos)
        self.create_subscription(LaserScan, "/scan",
                                 lambda m: self.row({"k": "scan", "stamp": _stamp(m.header)}),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map",
                                 lambda m: self.row({"k": "map", "stamp": _stamp(m.header)}),
                                 map_qos)

    def row(self, r):
        r["t"] = round(time.time(), 6)
        self.out.write(json.dumps(r) + "\n")


def main():
    path = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6 * 3600
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *a: stop.update(now=True))
    rclpy.init()
    with open(path, "a", buffering=1) as out:
        node = Pace(out)
        end = time.time() + seconds
        while not stop["now"] and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
