# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""convert -- the bridge's conversions at the wall, as plain Python.

Kept out of bridge.py (which imports rclpy) so they are unit-tested on a
laptop (tests/test_bridge_convert.py) rather than only through the live
container. These are the two places a picture can come out mirrored:

  * the project's bearings are CLOCKWISE-positive and ROS's are
    counter-clockwise (REP-103);
  * a beam with no return is None on our side and +inf on ROS's (REP-117).

And one thing is taken out on the way (3.221): the returns from the robot's
own body that the safety layer's self box names (the Rover's mast, 3.218)
go to ROS as -inf, "too close to measure" (REP-117) -- below range_min, so
slam_toolbox, the costmaps and collision_monitor neither mark nor clear on
them. Never +inf: that is "no return", which reads as clear.
"""

import math

# The lidar ahead of base_link: the xacro's laser_x and robot/safety.py's
# LIDAR_X_M (tests/test_wall_linters.py keeps the three equal).
LASER_X_M = 0.040


# Two capture stamps closer than this are one revolution read twice (3.221).
# The robot server fixes a revolution's stamp once, so a repeat is equal; the
# D500 turns at 10 Hz, so distinct revolutions are ~100 ms apart.
SAME_REVOLUTION_S = 0.03


def same_revolution(stamp, last):
    """Whether a capture `stamp` is not a new scan after the one last sent at
    `last` (None: nothing sent yet): the same revolution read again, or an
    older stamp (a clock stepped back -- NTP on a board with no RTC), which
    would reach slam_toolbox out of order. Up to a second back is held off
    until the clock catches up; further back is a new start (review, 3.221:
    publishing small steps back fed SLAM scans out of order)."""
    if last is None:
        return False
    return -1.0 < stamp - last < SAME_REVOLUTION_S


def self_box(value):
    """The bridge's reading of the robot server's `lidar_self_box_cm`: a
    tuple of four finite numbers with x_min < x_max and y_min < y_max, else
    None (no box: nothing masked). The robot server validates its box; this
    only keeps a malformed one from raising inside the scan timer."""
    try:
        box = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        return None
    if len(box) != 4 or not all(math.isfinite(v) for v in box) \
            or not (box[0] < box[1] and box[2] < box[3]):
        return None
    return box


def in_self_box(r, bearing_deg, box_cm):
    """Whether a return `r` metres away at our (clockwise) `bearing_deg` lies
    inside `box_cm` = (x_min, x_max, y_min, y_max), cm, body frame (x ahead,
    y left), edges included. The same arithmetic, in the same order, as
    robot/safety.py's `scan_points_cm()` and `_in_self_box()`, so the beams
    ROS loses are exactly the ones the veto calls the body's own."""
    a = math.radians(bearing_deg)
    x = r * 100.0 * math.cos(a) + LASER_X_M * 100.0
    y = -r * 100.0 * math.sin(a)
    x0, x1, y0, y1 = box_cm
    return x0 <= x <= x1 and y0 <= y <= y1


def ros_ranges(ours, start_deg, inc_deg, self_box_cm=None):
    """Our clockwise scan -> the ranges of a ROS LaserScan with the SAME
    angle_min / angle_increment, counter-clockwise.

    ROS beam j sits at start + j*inc, CCW-positive; the same direction in
    our clockwise convention is its negation, which is our beam
    ((-(start + j*inc)) - start) / inc, modulo the beam count.

    `self_box_cm` (the robot server's `lidar_self_box_cm`, 3.221): a return
    inside it is the robot's own body and becomes -inf. None: nothing is.
    """
    n = len(ours)
    box = self_box(self_box_cm) if self_box_cm else None
    out = []
    for j in range(n):
        ours_deg = -(start_deg + j * inc_deg)
        i = int(round((ours_deg - start_deg) / inc_deg)) % n
        r = ours[i]
        if r is None:
            out.append(math.inf)
        elif box is not None and in_self_box(r, start_deg + i * inc_deg, box):
            out.append(-math.inf)
        else:
            out.append(float(r))
    return out


def yaw_pitch_roll(x, y, z, w):
    """A unit quaternion's ZYX Euler angles, radians (yaw CCW-positive)."""
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll


def odometry_zero_in_house(truth, odom):
    """Where the robot stood, in the house's frame, when its odometry read
    zero -- from the truth and the odometry taken at the SAME instant
    (PLAN-ros-alignment.md 3.36). This is the anchor world/ros_world.py lays
    SLAM's frame on: `{"x_m", "y_m", "heading_deg"}`, house frame (x east,
    y south, compass heading clockwise from north), with odometry zero
    facing compass 90 in its own frame.

    It used to be the truth read at session start on the assumption the
    robot had not moved; when the container restarts while a person drives
    the fallback (3.24 G3), or the first reads come late on a slow board,
    it had, and every house-frame goal was converted ~20 deg off. Composing
    with the odometry at the same moment needs no such assumption.

    `truth`: {"x_m", "y_m", "heading_deg"}; `odom`: ROS odom frame
    {"x_m", "y_m", "yaw_rad"} (x forward, y left, yaw CCW)."""
    ox, oy = odom["x_m"], -odom["y_m"]                       # ROS -> ours, same frame
    oh = (90.0 - math.degrees(odom["yaw_rad"])) % 360.0
    alpha = (truth["heading_deg"] - oh + 180.0) % 360.0 - 180.0
    c, s = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
    return {"x_m": truth["x_m"] - (c * ox - s * oy),
            "y_m": truth["y_m"] - (s * ox + c * oy),
            "heading_deg": (90.0 + alpha) % 360.0}


def odometry_at(odom, t):
    """`odom` ({x_m, y_m, yaw_rad, linear_m_s, angular_rad_s, at}, ROS odom
    frame) carried by its own body velocities to time `t` -- a constant
    twist over the gap (3.76 H1-4, R4). The start truth is sampled inside a
    request; the newest odometry is up to one controller period older, and
    mid-turn that gap alone is degrees of heading."""
    dt = t - odom["at"]
    v, w = odom.get("linear_m_s") or 0.0, odom.get("angular_rad_s") or 0.0
    yaw = odom["yaw_rad"]
    mid = yaw + w * dt / 2
    return {"x_m": odom["x_m"] + v * dt * math.cos(mid),
            "y_m": odom["y_m"] + v * dt * math.sin(mid),
            "yaw_rad": yaw + w * dt}
