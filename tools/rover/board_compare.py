# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/rover/board_compare.py

3.168 / 3.140's L0.5: the real motor board against `sim/fake_esp32.py`, by
listening only.

    python -m tools.rover.board_compare --port /dev/ttyTHS1 [--seconds 60]
        [--firmware stock|fork] [--gap-s 0.5] [--out report.json] [--keep raw.jsonl]

Press the board's reset during the capture to read the boot text and the
silence after it. That silence is how long the board's OUTPUT stops: whether
its input is deaf for longer needs a write, which this tool cannot make.

Reports every field the fake sends and the board does not (and the reverse),
the units (`L`/`R` m/s, `v` centivolts, integer odometers), the frame rate
beside the fake's, how often `v` changes (M2.7 expects every 10 s, so give it
60 s or more) and every line that is not JSON. Each difference goes into the
fake (L0.5's bar). Turn a wheel by hand during the capture: speeds and
odometers that stayed at zero prove nothing about their units, and the row is
not met while they have (`not_exercised`).
"""

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

from sim.fake_esp32 import FakeEsp32
from tools.rover.serial_ro import BOARD_BAUD, lines, open_readonly

Line = Tuple[float, bytes]
RATE_TOLERANCE = 0.20
WHEEL_MAX_M_S = 2.0       # setGoalSpeed() ignores anything beyond this
PACK_CV = (900, 1300)     # a 3S pack, centivolts: 9.00 - 13.00 V
FEEDBACK = 1001
REFERENCE_S = 2.0


class _StillBody:
    """The least a FakeEsp32 needs: wheels that do not turn."""

    def get_wheel_state(self) -> dict:
        wheel = {"position_rad": 0.0, "velocity_rad_s": 0.0}
        return {"wheel_radius_m": 0.04, "left": dict(wheel), "right": dict(wheel)}

    def set_wheel_velocity(self, _left, _right) -> None:
        pass

    def advance(self, _dt) -> None:
        pass


def capture(port: str, seconds: float, baud: int = BOARD_BAUD) -> List[Line]:
    fd = open_readonly(port, baud)
    try:
        return list(lines(fd, seconds))
    finally:
        os.close(fd)


def parse(captured: List[Line]) -> Dict:
    """Frames (t, dict), other JSON's `T` counts, and the non-JSON text."""
    frames, other, text = [], {}, []
    for t, raw in captured:
        if not raw:
            continue
        try:
            obj = json.loads(raw)
        except ValueError:
            text.append((t, raw.decode("utf-8", "replace")))
            continue
        if isinstance(obj, dict) and obj.get("T") == FEEDBACK:
            frames.append((t, obj))
        elif isinstance(obj, dict):
            key = str(obj.get("T"))
            other[key] = other.get(key, 0) + 1
        else:
            text.append((t, raw.decode("utf-8", "replace")))
    return {"frames": frames, "other": other, "text": text}


def period_ms(frames: List[Tuple[float, dict]], gap_s: float) -> List[float]:
    ts = [t for t, _ in frames]
    return [(b - a) * 1000.0 for a, b in zip(ts, ts[1:]) if b - a < gap_s]


def reference(firmware: str, seconds: float = REFERENCE_S) -> Dict:
    """What the fake really sends, for this comparison: it cannot go stale."""
    fake = FakeEsp32(_StillBody(), firmware=firmware, battery_v=12.0, drain_v_per_s=0.0)
    try:
        got = parse(capture(fake.path, seconds))
    finally:
        fake.close()
    if not got["frames"]:
        raise RuntimeError("the fake sent no frames")
    keys = sorted({k for _, f in got["frames"] for k in f})
    periods = period_ms(got["frames"], 1.0)
    return {"firmware": firmware, "fields": keys,
            "period_p50_ms": statistics.median(periods) if periods else None}


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def units(frames: List[dict]) -> List[str]:
    out = []
    speeds = [abs(f[k]) for f in frames for k in ("L", "R") if _is_num(f.get(k))]
    if speeds and max(speeds) > WHEEL_MAX_M_S:
        out.append(f"L/R reach {max(speeds)}: over {WHEEL_MAX_M_S}, not m/s?")
    # 3.76 K1.4: a failed INA219 read gives 0; counted apart (`v_zero_frames`).
    volts = [f["v"] for f in frames if "v" in f and f["v"] != 0]
    if volts and not all(_is_int(v) for v in volts):
        out.append("v is not an integer (centivolts)")
    elif volts and not (PACK_CV[0] <= min(volts) and max(volts) <= PACK_CV[1]):
        out.append(f"v spans {min(volts)}-{max(volts)}: outside {PACK_CV} centivolts")
    for k in ("odl", "odr", "odlt", "odrt", "ms"):
        vals = [f[k] for f in frames if k in f]
        if vals and not all(_is_int(v) for v in vals):
            out.append(f"{k} is not an integer")
    for cm, tenth in (("odl", "odlt"), ("odr", "odrt")):
        pairs = [(f[cm], f[tenth]) for f in frames if _is_int(f.get(cm)) and _is_int(f.get(tenth))]
        # odlt is tenths of a mm: 100 to the centimetre, and odl truncates.
        if any(abs(t - c * 100) > 100 for c, t in pairs):
            out.append(f"{tenth} does not track {cm} at 100 per cm")
    return out


def not_exercised(frames: List[dict]) -> List[str]:
    """Fields whose units a run of zeros could not show."""
    out = []
    if not any(f.get(k) for f in frames for k in ("L", "R")):
        out.append("L/R (turn a wheel by hand during the capture)")
    if not any(f.get(k) for f in frames for k in ("odl", "odr")):
        out.append("odl/odr (turn a wheel by hand during the capture)")
    return out


def v_changes_s(frames: List[Tuple[float, dict]]) -> List[float]:
    """Seconds between successive changes of `v`."""
    at, last = [], None
    for t, f in frames:
        if "v" in f and f["v"] != last:
            if last is not None:
                at.append(t)
            last = f["v"]
    return [round(b - a, 3) for a, b in zip(at, at[1:])]


def resets(frames: List[Tuple[float, dict]], text, gap_s: float) -> List[Dict]:
    """Each silence of `gap_s` or more in the frames, with the text inside."""
    out = []
    for (a, _), (b, _) in zip(frames, frames[1:]):
        if b - a >= gap_s:
            inside = [(t, s) for t, s in text if a <= t <= b]
            rec = {"output_silent_s": round(b - a, 3), "boot_text": [s for _, s in inside]}
            if inside:
                rec["boot_text_to_first_frame_s"] = round(b - inside[0][0], 3)
            out.append(rec)
    return out


def compare(captured: List[Line], ref: Dict, gap_s: float = 0.5) -> Dict:
    got = parse(captured)
    frames = got["frames"]
    report = {"frames": len(frames), "reference": ref, "other_frames": got["other"],
              "text_lines": [s for _, s in got["text"]][:40],
              "resets": resets(frames, got["text"], gap_s)}
    fields = sorted({k for _, f in frames for k in f})
    report["missing_fields"] = [k for k in ref["fields"] if k not in fields]
    report["extra_fields"] = [k for k in fields if k not in ref["fields"]]
    plain = [f for _, f in frames]
    report["units"] = units(plain)
    report["not_exercised"] = not_exercised(plain)
    report["v_zero_frames"] = sum(1 for f in plain if f.get("v") == 0)
    periods = period_ms(frames, gap_s)
    p50 = statistics.median(periods) if periods else None
    report["period_p50_ms"] = None if p50 is None else round(p50, 2)
    report["rate_ok"] = bool(p50 and ref["period_p50_ms"]
                             and abs(p50 - ref["period_p50_ms"]) <= RATE_TOLERANCE * ref["period_p50_ms"])
    volts = [f["v"] for _, f in frames if "v" in f and f["v"] != 0]
    report["v_range"] = [min(volts), max(volts)] if volts else None
    report["v_change_interval_s"] = v_changes_s(frames)[:40]
    report["met"] = bool(frames and not report["missing_fields"] and not report["units"]
                         and not report["not_exercised"] and report["rate_ok"])
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    ap.add_argument("--port", required=True)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--firmware", choices=("stock", "fork"), default="stock")
    ap.add_argument("--gap-s", type=float, default=0.5, help="a silence this long is a reset")
    ap.add_argument("--out")
    ap.add_argument("--keep", help="write every line heard, with its time, as JSONL")
    a = ap.parse_args(argv)
    started = time.time()
    try:
        captured = capture(a.port, a.seconds)
    except OSError as e:
        print(f"refused: {e}", file=sys.stderr)
        return 2
    report = compare(captured, reference(a.firmware), a.gap_s)
    report.update(port=a.port, seconds=a.seconds, started=started)
    text = json.dumps(report, indent=1)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")
    if a.keep:
        with open(a.keep, "w") as f:
            for t, raw in captured:
                f.write(json.dumps({"t": t, "line": raw.decode("utf-8", "replace")}) + "\n")
    return 0 if report["met"] else 1


if __name__ == "__main__":
    sys.exit(main())
