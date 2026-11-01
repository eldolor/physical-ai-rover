"""Inside the ROS container (3.50.D5, criterion 6): for SECS seconds, every
/pose slam_toolbox publishes -- one per processed scan, stamped with that
scan's capture time (slam_toolbox_common.cpp publishPose) -- as receipt
minus stamp, to OUT (JSON). Run by d5_runs.py with `docker exec`."""
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

SECS = float(sys.argv[1])
OUT = sys.argv[2]


class Rec(Node):
    def __init__(self):
        super().__init__("d5_rec")
        self.lags = []
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, 50)

    def on_pose(self, p):
        stamp = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
        self.lags.append((time.time(), time.time() - stamp))


rclpy.init()
n = Rec()
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(OUT, "w") as f:
    json.dump({"lags": n.lags}, f)
print("poses", len(n.lags))
