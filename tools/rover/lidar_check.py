# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/rover/lidar_check.py

3.168 / 3.140's L0.6: the D500 on the real port, a 30 s capture and its
score against the row's bars.

    python -m tools.rover.lidar_check capture --port /dev/picar-lidar --out cap.bin [--seconds 30]
    python -m tools.rover.lidar_check score cap.bin --wall-bearing 90 --wall-m 1.00 --box-m 0.60

Bars: under 0.1% CRC failures; 10 Hz +/- 10%; the taped wall within 2 cm; a
box dead ahead at 0 +/- 2 degrees. Distances are taped from the lidar's own
centre; bearings are the body's (`robot/lidar_ld19.LIDAR_YAW_DEG` applied, as
the robot server applies it), so a wrong yaw reads as a box off bearing.
Parsing, CRC and revolutions are the driver's own code.
"""

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

from robot.lidar_ld19 import (LIDAR_YAW_DEG, RANGE_MAX_M, RANGE_MIN_M, TIMESTAMP_WRAP_MS,
                              Assembler, Parser, packet_points)
from tools.rover.serial_ro import LIDAR_BAUD, chunks, open_readonly

CRC_BAR = 0.001
RATE_HZ, RATE_TOLERANCE = 10.0, 0.10
WALL_BAR_M, BOX_BAR_DEG = 0.02, 2.0
WALL_WINDOW_DEG = 2.0
BOX_SEARCH_DEG = 30.0     # keep this window clear of anything nearer than the box
BOX_DEPTH_M = 0.15        # the nearest return must be this near the taped distance
BOX_SURFACE_M = 0.05      # the box's face: within this of its nearest return
BOX_GAP_DEG = 2.0
MIN_CAPTURE_S = 28.0      # L0.6 is a 30 s capture, on the lidar's own clock


def capture(port: str, out: str, seconds: float = 30.0) -> Dict:
    fd = open_readonly(port, LIDAR_BAUD)
    size, started = 0, time.time()
    try:
        with open(out, "wb") as f:
            for _t, data in chunks(fd, seconds):
                f.write(data)
                size += len(data)
    finally:
        os.close(fd)
    meta = {"port": port, "baud": LIDAR_BAUD, "started": started, "seconds": seconds, "bytes": size}
    Path(out + ".json").write_text(json.dumps(meta, indent=1) + "\n")
    return meta


def _body_deg(lidar_deg: float, yaw_deg: float) -> float:
    return (lidar_deg + yaw_deg + 180.0) % 360.0 - 180.0


def revolutions(data: bytes):
    """(parser, revolutions, nominal speeds): times are the lidar's own clock,
    unwrapped, so a slow reader cannot change the measured rate."""
    parser, asm = Parser(), Assembler()
    revs, speeds = [], []
    offset, last_ms, t_prev = 0.0, None, None
    for pkt in parser.feed(data):
        if last_ms is not None and pkt.timestamp_ms < last_ms:
            offset += TIMESTAMP_WRAP_MS / 1000.0
        last_ms = pkt.timestamp_ms
        t = offset + pkt.timestamp_ms / 1000.0
        revs += asm.add(packet_points(pkt, t_prev, t))
        t_prev = t
        speeds.append(pkt.speed_dps)
    return parser, revs, speeds


def _delta(bearing: float, target: float) -> float:
    return (bearing - target + 180.0) % 360.0 - 180.0


def wall_range(revs, bearing: float, yaw_deg: float) -> Optional[float]:
    vals = [mm / 1000.0 for rev in revs for a, mm, _t in rev
            if RANGE_MIN_M * 1000 <= mm <= RANGE_MAX_M * 1000
            and abs(_delta(_body_deg(a, yaw_deg), bearing)) <= WALL_WINDOW_DEG]
    return statistics.median(vals) if vals else None


def box_bearing(revs, box_m: float, yaw_deg: float) -> Optional[float]:
    """The box's centre: per revolution, the nearest return in the search
    window, grown along the bearings still on its face; the median of those
    middles. A wall behind the box is not its face, however near the tape.
    Use a narrow box (15 cm or less) with its face square to the lidar: a
    wide one's edges fall outside BOX_SURFACE_M and its middle reads the face."""
    centres = []
    for rev in revs:
        pts = sorted((_body_deg(a, yaw_deg), mm / 1000.0) for a, mm, _t in rev
                     if RANGE_MIN_M * 1000 <= mm <= RANGE_MAX_M * 1000
                     and abs(_body_deg(a, yaw_deg)) <= BOX_SEARCH_DEG)
        if not pts:
            continue
        i = min(range(len(pts)), key=lambda k: pts[k][1])
        nearest = pts[i][1]
        if abs(nearest - box_m) > BOX_DEPTH_M:
            continue
        lo = hi = i
        while lo > 0 and pts[lo - 1][1] <= nearest + BOX_SURFACE_M and pts[lo][0] - pts[lo - 1][0] <= BOX_GAP_DEG:
            lo -= 1
        while hi < len(pts) - 1 and pts[hi + 1][1] <= nearest + BOX_SURFACE_M \
                and pts[hi + 1][0] - pts[hi][0] <= BOX_GAP_DEG:
            hi += 1
        centres.append((pts[lo][0] + pts[hi][0]) / 2.0)
    return statistics.median(centres) if centres else None


def score(data: bytes, wall_bearing=None, wall_m=None, box_m=None,
          yaw_deg: float = LIDAR_YAW_DEG) -> Dict:
    parser, revs, speeds = revolutions(data)
    seen = parser.packets + parser.crc_failed
    crc_share = parser.crc_failed / seen if seen else None
    times = [rev[0][2] for rev in revs]
    rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 2 and times[-1] > times[0] else None
    total = sum(len(r) for r in revs)
    report = {"packets": parser.packets, "crc_failed": parser.crc_failed, "crc_share": crc_share,
              "bytes_skipped": parser.bytes_skipped, "revolutions": len(revs),
              "scan_rate_hz": None if rate is None else round(rate, 3),
              "nominal_speed_hz": round(statistics.mean(speeds) / 360.0, 3) if speeds else None,
              "no_return_share": round(sum(1 for r in revs for p in r if not p[1]) / total, 4) if total else None,
              "yaw_deg": yaw_deg}
    span = times[-1] - times[0] if len(times) > 1 else 0.0
    report["capture_s"] = round(span, 2)
    rows = {"duration": span >= MIN_CAPTURE_S,
            "crc": crc_share is not None and crc_share < CRC_BAR,
            "rate": rate is not None and abs(rate - RATE_HZ) <= RATE_TOLERANCE * RATE_HZ}
    not_measured = []
    if wall_bearing is None or wall_m is None:
        not_measured.append("wall (--wall-bearing and --wall-m)")
    else:
        got = wall_range(revs, wall_bearing, yaw_deg)
        report["wall_m"] = None if got is None else round(got, 4)
        report["wall_error_m"] = None if got is None else round(got - wall_m, 4)
        rows["wall"] = got is not None and abs(got - wall_m) <= WALL_BAR_M
    if box_m is None:
        not_measured.append("box (--box-m)")
    else:
        got = box_bearing(revs, box_m, yaw_deg)
        report["box_bearing_deg"] = None if got is None else round(got, 3)
        rows["box"] = got is not None and abs(got) <= BOX_BAR_DEG
    report["rows"] = rows
    report["not_measured"] = not_measured
    report["failed"] = sorted(k for k, ok in rows.items() if not ok)
    report["met"] = not report["failed"] and not not_measured
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--port", required=True)
    c.add_argument("--out", required=True)
    c.add_argument("--seconds", type=float, default=30.0)
    s = sub.add_parser("score")
    s.add_argument("capture")
    s.add_argument("--wall-bearing", type=float)
    s.add_argument("--wall-m", type=float)
    s.add_argument("--box-m", type=float)
    s.add_argument("--yaw-deg", type=float, default=LIDAR_YAW_DEG)
    s.add_argument("--out")
    a = ap.parse_args(argv)
    if a.cmd == "capture":
        try:
            print(json.dumps(capture(a.port, a.out, a.seconds), indent=1))
        except OSError as e:
            print(f"refused: {e}", file=sys.stderr)
            return 2
        return 0
    report = score(Path(a.capture).read_bytes(), a.wall_bearing, a.wall_m, a.box_m, a.yaw_deg)
    text = json.dumps(report, indent=1)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")
    return 0 if report["met"] else 1


if __name__ == "__main__":
    sys.exit(main())
