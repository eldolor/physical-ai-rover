# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""evaluations/rover-slam-3221/measure.py -- 3.221.B's instruments.

Run on the board, beside the robot server and the ROS container, while the
user drives the Rover by hand. Reads only; never moves anything.

    python measure.py stamps [--n 200]        # criterion 6b
    python measure.py rest [--secs 60]        # criteria 1b, 4, 6c
    python measure.py track --out FILE        # a 2 Hz trajectory until Ctrl-C
    python measure.py judge --track FILE      # criteria 1c and 5, the map's free area
    python measure.py length X1 Y1 X2 Y2      # criterion 3: map-frame distance (m)
    python measure.py render --out FILE.pgm   # the map, to look at on the board

The trajectory and the map are a home's floor plan: they stay on the board
(the user's rule); only the numbers this prints go into the section file.

Env: PICAR_ROBOT_URL (default :8000), PICAR_BRIDGE_URL (default :8090), and
the secret from APP_SHARED_SECRET or LOCAL_SECRET (else
~/.vision-picar-local-secrets).
"""

import argparse
import json
import math
import os
import re
import statistics
import time
import urllib.request
from pathlib import Path

ROBOT = os.environ.get("PICAR_ROBOT_URL", "http://127.0.0.1:8000")
BRIDGE = os.environ.get("PICAR_BRIDGE_URL", "http://127.0.0.1:8090")
LASER_X_M = 0.040
HALF_L_M, HALF_W_M = 0.1265, 0.1155        # the footprint (nav2.yaml)
OCCUPIED = 65                               # nav2's occupied threshold on a 0-100 grid


def _secret():
    s = os.environ.get("APP_SHARED_SECRET") or os.environ.get("LOCAL_SECRET")
    if s:
        return s
    p = Path.home() / ".vision-picar-local-secrets"
    if p.exists():
        m = re.search(r"LOCAL_SECRET=['\"]?([^'\"\n]+)", p.read_text())
        if m:
            return m.group(1)
    return ""


SECRET = _secret()


def get(base, path):
    req = urllib.request.Request(base + path, headers={"x-app-secret": SECRET})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def _wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


def stamps(n):
    """6b: every reply `capture`; reply time minus stamp in [0, 250] ms;
    the median gap between successive distinct stamps 90-110 ms."""
    rows = []
    for _ in range(n):
        t0 = time.time()
        s = get(ROBOT, "/scan")
        t1 = time.time()
        rows.append((t0, t1, s.get("stamp_unix"), s.get("stamp_source")))
    capture = sum(1 for r in rows if r[3] == "capture")
    lag = [(t1 - st) * 1000 for _, t1, st, _ in rows if st is not None]
    distinct = sorted({st for *_, st, _ in rows if st is not None})
    gaps = [(b - a) * 1000 for a, b in zip(distinct, distinct[1:])]
    out = {"replies": n, "capture": capture,
           "lag_ms_min": round(min(lag), 1), "lag_ms_median": round(statistics.median(lag), 1),
           "lag_ms_max": round(max(lag), 1), "distinct_stamps": len(distinct),
           "gap_ms_median": round(statistics.median(gaps), 1) if gaps else None,
           "met": capture == n and min(lag) >= 0 and max(lag) <= 250
                  and bool(gaps) and 90 <= statistics.median(gaps) <= 110}
    print(json.dumps(out, indent=1))


def _box_hits(scan, box_cm):
    """Finite returns of the bridge's last scan (ROS convention: CCW) inside
    the self box -- the published ranges, with +/-inf read as None."""
    if not scan.get("ranges_m"):
        return None
    x0, x1, y0, y1 = box_cm
    a0, inc, hits = scan["angle_min_rad"], scan["angle_increment_rad"], 0
    for j, r in enumerate(scan["ranges_m"]):
        if r is None:
            continue
        a = a0 + j * inc
        x, y = (r * math.cos(a) + LASER_X_M) * 100.0, r * math.sin(a) * 100.0
        hits += x0 <= x <= x1 and y0 <= y <= y1
    return hits


def rest(secs):
    """1b, 4 and 6c with the Rover stationary."""
    box = get(ROBOT, "/health").get("lidar_self_box_cm")
    pub0 = get(BRIDGE, "/health").get("scans_published")
    rev0 = (get(ROBOT, "/health").get("lidar") or {}).get("revolutions")
    poses, hits, seen = [], [], None
    end = time.time() + secs
    while time.time() < end:
        p = get(BRIDGE, "/slam/pose").get("map")
        if p:
            poses.append(p)
        s = get(BRIDGE, "/scan")
        if box and s.get("at") != seen:
            seen = s.get("at")
            h = _box_hits(s, box)
            if h is not None:
                hits.append(h)
        time.sleep(0.5)
    pub1 = get(BRIDGE, "/health").get("scans_published")
    rev1 = (get(ROBOT, "/health").get("lidar") or {}).get("revolutions")
    out = {"secs": secs, "box": box, "scans_checked": len(hits),
           "finite_in_box_max": max(hits) if hits else None}
    if poses:
        mx = statistics.median(p["x_m"] for p in poses)
        my = statistics.median(p["y_m"] for p in poses)
        myaw = statistics.median(math.degrees(p["yaw_rad"]) for p in poses)
        out.update(poses=len(poses),
                   excursion_cm=round(max(math.hypot(p["x_m"] - mx, p["y_m"] - my)
                                          for p in poses) * 100, 2),
                   excursion_deg=round(max(abs(_wrap_deg(math.degrees(p["yaw_rad"]) - myaw))
                                           for p in poses), 3))
    if None not in (pub0, pub1, rev0, rev1):
        out.update(scans_published=pub1 - pub0, revolutions=rev1 - rev0,
                   repeats=get(BRIDGE, "/health").get("scans_repeated"))
    print(json.dumps(out, indent=1))


def track(path):
    """A 2 Hz trajectory (map and odom poses) until Ctrl-C."""
    n = 0
    with open(path, "a") as f:
        try:
            while True:
                p = get(BRIDGE, "/slam/pose")
                f.write(json.dumps({"t": time.time(), "map": p.get("map"),
                                    "odom": p.get("odom")}) + "\n")
                f.flush()
                n += 1
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
    print(f"{n} samples in {path}")


def _occupied_in_footprint(m, pose, shrink_m):
    res, w = m["resolution_m"], m["width"]
    hl, hw = HALF_L_M - shrink_m, HALF_W_M - shrink_m
    c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
    n, step = 0, res / 2.0
    seen = set()
    x = -hl
    while x <= hl + 1e-9:
        y = -hw
        while y <= hw + 1e-9:
            wx = pose["x_m"] + c * x - s * y - m["origin_x_m"]
            wy = pose["y_m"] + s * x + c * y - m["origin_y_m"]
            i, j = int(math.floor(wx / res)), int(math.floor(wy / res))
            if (i, j) not in seen and 0 <= i < w and 0 <= j < m["height"]:
                seen.add((i, j))
                n += m["data"][j * w + i] >= OCCUPIED
            y += step
        x += step
    return n


def _median_pose(rows, key):
    ps = [r[key] for r in rows if r.get(key)]
    return {"x_m": statistics.median(p["x_m"] for p in ps),
            "y_m": statistics.median(p["y_m"] for p in ps),
            "yaw_deg": statistics.median(math.degrees(p["yaw_rad"]) for p in ps)}


def _delta(a, b):
    return {"cm": round(math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]) * 100, 1),
            "deg": round(abs(_wrap_deg(b["yaw_deg"] - a["yaw_deg"])), 2)}


def judge(path, ends):
    """1c (footprint cells), 5 (loop), the map's free area and the path length."""
    rows = [json.loads(line) for line in open(path) if line.strip()]
    m = get(BRIDGE, "/slam/map")
    res = m["resolution_m"]
    with_map = [r for r in rows if r.get("map")]
    bad = sum(1 for r in with_map if _occupied_in_footprint(m, r["map"], res) > 0)
    length = sum(math.hypot(b["odom"]["x_m"] - a["odom"]["x_m"], b["odom"]["y_m"] - a["odom"]["y_m"])
                 for a, b in zip(rows, rows[1:]) if a.get("odom") and b.get("odom"))
    out = {"samples": len(rows), "with_map_pose": len(with_map),
           "footprint_occupied_poses": bad,
           "footprint_occupied_share": round(bad / len(with_map), 4) if with_map else None,
           "free_m2": round(sum(1 for v in m["data"] if v == 0) * res * res, 2),
           "path_m_odom": round(length, 2)}
    if len(rows) >= 2 * ends:
        out["loop_map"] = _delta(_median_pose(rows[:ends], "map"), _median_pose(rows[-ends:], "map"))
        out["loop_odom"] = _delta(_median_pose(rows[:ends], "odom"), _median_pose(rows[-ends:], "odom"))
    print(json.dumps(out, indent=1))


def render(path):
    """The map as a PGM (white free, black occupied, grey unknown), row 0 at
    the top = the map's largest y. Stays on the board."""
    m = get(BRIDGE, "/slam/map")
    w, h = m["width"], m["height"]
    px = bytearray()
    for j in range(h - 1, -1, -1):
        for i in range(w):
            v = m["data"][j * w + i]
            px.append(205 if v < 0 else (0 if v >= OCCUPIED else 254))
    with open(path, "wb") as f:
        f.write(f"P5 {w} {h} 255\n".encode() + bytes(px))
    print(json.dumps({"path": path, "width": w, "height": h, "resolution_m": m["resolution_m"],
                      "origin": [m["origin_x_m"], m["origin_y_m"]]}))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("stamps").add_argument("--n", type=int, default=200)
    sub.add_parser("rest").add_argument("--secs", type=float, default=60.0)
    sub.add_parser("track").add_argument("--out", required=True)
    j = sub.add_parser("judge")
    j.add_argument("--track", required=True)
    j.add_argument("--ends", type=int, default=6)
    ln = sub.add_parser("length")
    for k in ("x1", "y1", "x2", "y2"):
        ln.add_argument(k, type=float)
    sub.add_parser("render").add_argument("--out", required=True)
    a = ap.parse_args()
    if a.cmd == "stamps":
        stamps(a.n)
    elif a.cmd == "rest":
        rest(a.secs)
    elif a.cmd == "track":
        track(a.out)
    elif a.cmd == "judge":
        judge(a.track, a.ends)
    elif a.cmd == "length":
        print(round(math.hypot(a.x2 - a.x1, a.y2 - a.y1), 3))
    elif a.cmd == "render":
        render(a.out)


if __name__ == "__main__":
    main()
