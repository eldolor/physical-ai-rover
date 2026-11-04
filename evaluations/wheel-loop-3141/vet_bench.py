# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.A: what one wheel-loop vet costs (`SafetyController.vet_wheel_velocity`)
on a 360-bin scan (the D500's, `robot/lidar_ld19.py`), by case: driving
straight in the open, straight toward a wall, turning in the open, and
turning with a corner close enough that `pivot_scale()` searches.

    python evaluations/wheel-loop-3141/vet_bench.py [repeats]
"""
import math
import statistics
import sys
import time
from pathlib import Path

# The repo root; on the board the file is copied to /tmp and run from the checkout.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]) if len(Path(__file__).resolve().parents) > 2
                else ".")

from robot.safety import SafetyController  # noqa: E402

R, TRACK = 0.04, 0.17


class Body:
    def __init__(self, ranges, turned):
        self.ranges, self.turned, self.lr = ranges, turned, (0.0, 0.0)

    def get_wheel_state(self):
        return {"usable": True, "wheel_radius_m": R, "track_width_m": TRACK,
                "left": {"velocity_rad_s": self.lr[0]}, "right": {"velocity_rad_s": self.lr[1]}}

    def get_scan(self, max_range_m=None):
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "ranges_m": list(self.ranges)}

    def get_depth_grid(self):
        return {"usable": False}

    def sensor_age_s(self):
        return 0.08

    def turn_since_scan(self):
        return self.turned


def box(half_x, half_y):
    """Ranges from the lidar (4 cm ahead of centre) to a rectangle of walls."""
    out = []
    for i in range(360):
        a = math.radians(-180 + i)
        c, s = math.cos(a), math.sin(a)
        t = min((half_x - 0.04) / c if c > 1e-9 else math.inf,
                (-half_x - 0.04) / c if c < -1e-9 else math.inf,
                half_y / abs(s) if abs(s) > 1e-9 else math.inf)
        out.append(round(t, 3))
    return out


CASES = {
    "straight, open":          (box(3.0, 3.0), 0.0, (10.0, 10.0)),
    "straight, wall 25 cm":    (box(0.40, 3.0), 0.0, (10.0, 10.0)),
    "turn, open":              (box(3.0, 3.0), 0.05, (-8.0, 8.0)),
    "turn, corner 1 cm off":   (box(0.137, 0.126), 0.05, (-8.0, 8.0)),
}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    for name, (ranges, turned, lr) in CASES.items():
        body = Body(ranges, turned)
        body.lr = lr
        s = SafetyController(body)
        ts = []
        for _ in range(n):
            t0 = time.perf_counter()
            out = s.vet_wheel_velocity(*lr)
            ts.append((time.perf_counter() - t0) * 1000)
        ts.sort()
        print(f"{name:24s} p50 {statistics.median(ts):6.2f} ms  p90 {ts[int(0.9 * n)]:6.2f}  "
              f"max {ts[-1]:6.2f}  -> {out[2]}")


if __name__ == "__main__":
    main()
