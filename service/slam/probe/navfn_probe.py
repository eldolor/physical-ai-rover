# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""service/slam/probe/navfn_probe.py -- 3.211.A: ask nav2's planner_server for
paths between fixed start/goal pairs and record each answer. Nothing moves:
`compute_path_to_pose` with `use_start` plans only.

Runs INSIDE the ROS container (it imports rclpy, so it lives in service/slam,
CLAUDE.md rule 3); the host driver is evaluations/navfn-3211/probe.py, which
copies this file in with `docker cp` and runs it with `docker exec`.

    python3 navfn_probe.py draw  PAIRS.json SEED N MIN_M DOCK_X DOCK_Y STAGE_X STAGE_Y TRIPS
    python3 navfn_probe.py plan  PAIRS.json OUT.jsonl

`draw` reads the static map (/map) and draws N pairs of free cells at least
MIN_M apart, each also reversed, plus TRIPS trip-home pairs (a free cell to
the staging point, then to the dock, alternating). A cell is free when it and
every cell within CLEAR_M are known free on the map: clear of the chassis'
circumscribed radius (0.151 m), so no start or goal is in inscribed cost by
construction. `plan` asks for each pair and writes one row a pair: ok, the
path's length, nav2's own planning time, and the wall times of the call (the
driver matches planner log lines to pairs by them).
"""

import json
import math
import random
import sys
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

CLEAR_M = 0.20
CALL_TIMEOUT_S = 30.0


def grab_map(node):
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", got.append, qos)
    end = time.time() + 30
    while not got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not got:
        raise SystemExit("no /map in 30 s")
    return got[-1]


def free_cells(m):
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    d = m.data
    r = int(math.ceil(CLEAR_M / res))
    blocked = [False] * (w * h)
    for i, v in enumerate(d):
        if v != 0:                      # occupied or unknown blocks its neighbourhood
            cx, cy = i % w, i // w
            for yy in range(max(0, cy - r), min(h, cy + r + 1)):
                base = yy * w
                for xx in range(max(0, cx - r), min(w, cx + r + 1)):
                    blocked[base + xx] = True
    out = []
    for i in range(w * h):
        if not blocked[i]:
            out.append((ox + (i % w + 0.5) * res, oy + (i // w + 0.5) * res))
    return out


def draw(path, seed, n, min_m, dock, stage, trips):
    rclpy.init()
    node = Node("navfn_probe_draw")
    cells = free_cells(grab_map(node))
    rng = random.Random(seed)
    pairs = []
    while len(pairs) < 2 * n:
        a, b = rng.choice(cells), rng.choice(cells)
        if math.hypot(a[0] - b[0], a[1] - b[1]) < min_m:
            continue
        pairs.append({"kind": "pair", "start": a, "goal": b})
        pairs.append({"kind": "reverse", "start": b, "goal": a})
    for i in range(trips):
        a = rng.choice(cells)
        to = stage if i % 2 == 0 else dock
        pairs.append({"kind": "staging" if i % 2 == 0 else "dock", "start": a, "goal": list(to)})
    for i, p in enumerate(pairs):
        p["i"] = i
    with open(path, "w") as f:
        json.dump({"seed": seed, "free_cells": len(cells), "pairs": pairs}, f)
    print(f"{len(pairs)} pairs from {len(cells)} free cells")


def pose(xy):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = float(xy[0]), float(xy[1])
    p.pose.orientation.w = 1.0
    return p


def plan(path, out):
    rclpy.init()
    node = Node("navfn_probe_plan")
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=30):
        raise SystemExit("no compute_path_to_pose server")
    pairs = json.load(open(path))["pairs"]
    with open(out, "w") as f:
        for p in pairs:
            goal = ComputePathToPose.Goal()
            goal.start, goal.goal = pose(p["start"]), pose(p["goal"])
            goal.planner_id, goal.use_start = "GridBased", True
            t0 = time.time()
            fut = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=CALL_TIMEOUT_S)
            row = {"i": p["i"], "kind": p["kind"], "t0": t0}
            gh = fut.result() if fut.done() else None
            if gh is None or not gh.accepted:
                row.update(ok=False, status="not_accepted")
            else:
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(node, rf, timeout_sec=CALL_TIMEOUT_S)
                res = rf.result() if rf.done() else None
                if res is None:
                    row.update(ok=False, status="timeout")
                else:
                    poses = res.result.path.poses
                    ok = res.status == 4 and len(poses) > 0          # GoalStatus.STATUS_SUCCEEDED
                    length = sum(math.hypot(poses[k + 1].pose.position.x - poses[k].pose.position.x,
                                            poses[k + 1].pose.position.y - poses[k].pose.position.y)
                                 for k in range(len(poses) - 1))
                    pt = res.result.planning_time
                    row.update(ok=ok, status=int(res.status), length_m=round(length, 3) if ok else None,
                               planning_ms=round(pt.sec * 1e3 + pt.nanosec / 1e6, 1))
            row["t1"] = time.time()
            f.write(json.dumps(row) + "\n")
            f.flush()
    print("done")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "draw":
        draw(a[1], int(a[2]), int(a[3]), float(a[4]), (float(a[5]), float(a[6])),
             (float(a[7]), float(a[8])), int(a[9]))
    elif a and a[0] == "plan":
        plan(a[1], a[2])
    else:
        sys.exit(__doc__)
