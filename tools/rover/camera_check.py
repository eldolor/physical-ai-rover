# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/rover/camera_check.py

3.168 / 3.140's L0.8: one camera's frame age and staleness through
`robot/camera_stream.py`, the class the robot server serves frames from.

    python -m tools.rover.camera_check --device /dev/picar-camera [--seconds 600]

Bars: frame age p90 at or under 100 ms over 10 minutes (3.90.A: 46 ms on the
Jetson), and no stale frame served. A read the stream refuses as stale is
counted as refused: that is the 503 doing its job, not a stale frame served.
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Callable, Dict

from robot.camera_stream import CameraStale, CameraStream, OpenCvSource, encode_jpeg

AGE_P90_BAR_S = 0.100
READ_PERIOD_S = 0.1       # the rate a tiered mission's fastest loop reads at
MIN_SECONDS = 600.0       # L0.8: over 10 minutes
MAX_REFUSED_SHARE = 0.01  # a camera mostly refused has not delivered


def p90(values) -> float:
    return statistics.quantiles(values, n=10, method="inclusive")[8] if len(values) > 1 else values[0]


def measure(stream: CameraStream, seconds: float, period: float = READ_PERIOD_S,
            clock: Callable[[], float] = time.time, sleep=time.sleep,
            min_seconds: float = MIN_SECONDS) -> Dict:
    """Read like the server does; age is now minus the frame's `captured_at`."""
    ages, seqs, refused, stale_served, reasons = [], set(), 0, 0, {}
    end = time.monotonic() + seconds
    first_served = None
    while time.monotonic() < end:
        try:
            frame = stream.get_camera_frame()
        except CameraStale as e:
            if not ages:          # still opening: not yet a camera that stopped
                sleep(period)
                continue
            refused += 1
            key = str(e).split(":")[1].strip()[:30] if ":" in str(e) else str(e)[:30]
            reasons[key] = reasons.get(key, 0) + 1
        else:
            first_served = time.monotonic() if first_served is None else first_served
            age = clock() - frame["captured_at"]
            ages.append(age)
            seqs.add(frame["metadata"]["seq"])
            stale_served += age > stream.stale_s
        sleep(period)
    report = {"reads": len(ages) + refused, "served": len(ages), "refused_stale": refused,
              "refused_reasons": reasons, "stale_served": stale_served,
              "distinct_frames": len(seqs), "stale_limit_s": stream.stale_s,
              "age_p50_ms": round(statistics.median(ages) * 1000.0, 2) if ages else None,
              "age_p90_ms": round(p90(ages) * 1000.0, 2) if ages else None,
              "age_max_ms": round(max(ages) * 1000.0, 2) if ages else None,
              "stream": stream.status()}
    share = refused / report["reads"] if report["reads"] else 1.0
    report["refused_share"] = round(share, 4)
    # Timed from the first frame: a camera slow to open has not delivered.
    measured = end - first_served if first_served is not None else 0.0
    report["measured_s"] = round(measured, 2)
    report["met"] = bool(ages) and measured >= min_seconds and p90(ages) <= AGE_P90_BAR_S \
        and stale_served == 0 and share <= MAX_REFUSED_SHARE
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    ap.add_argument("--device", required=True, help="a udev name, /dev/videoN or an index")
    ap.add_argument("--seconds", type=float, default=620.0, help="the bar is 600 from the first frame")
    ap.add_argument("--warm-s", type=float, default=2.0, help="let it open before reading")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    stream = CameraStream(lambda: OpenCvSource(a.device), encode=encode_jpeg, name="camera")
    try:
        time.sleep(a.warm_s)
        report = measure(stream, a.seconds)
    finally:
        stream.close()
    report.update(device=a.device, seconds=a.seconds)
    text = json.dumps(report, indent=1)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")
    return 0 if report["met"] else 1


if __name__ == "__main__":
    sys.exit(main())
