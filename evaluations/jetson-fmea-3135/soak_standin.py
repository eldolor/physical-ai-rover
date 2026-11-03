# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/soak_standin.py

A stand-in process for 3.135.C's planted check (`soak_plant.py`): it holds
`--base-mb` and, every `--every-s`, keeps `--leak-mb` more (touched, so it is
resident) -- a planted leak when `--leak-mb` > 0, a steady process at 0.
Its `--name` is what `soak_score.role()` reads.

    python soak_standin.py --name leak --base-mb 50 --leak-mb 25 --every-s 10 --seconds 200
"""

import argparse
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--base-mb", type=int, default=50)
    ap.add_argument("--leak-mb", type=int, default=0)
    ap.add_argument("--every-s", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=200.0)
    a = ap.parse_args()
    held = [bytearray(b"\x01") * (a.base_mb << 20)]
    end, nxt = time.monotonic() + a.seconds, time.monotonic() + a.every_s
    while time.monotonic() < end:
        if a.leak_mb and time.monotonic() >= nxt:
            held.append(bytearray(b"\x01") * (a.leak_mb << 20))
            nxt += a.every_s
        time.sleep(0.2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
