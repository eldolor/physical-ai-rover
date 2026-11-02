# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/scorers.py

How each 3.136 row reads its number from what the board tools write. The
planted-delay check (criterion 3, `plant.py`) runs exactly these functions,
so a scorer that misreads a delay fails there before any board run.

* P1/P2/P10 -- `perception_ms()`: `perception_loop.py`'s frame lines
  ("HH:MM:SS <ms> <status>"), inside a window, as headroom_identity.sh
  reads them.
* P4 -- `wheel_delta()`: the robot server's `/health` `wheel_loop` read at
  the window's start and end (late ticks are a difference; `max_dt_s` is
  the server's running maximum).
* P8 -- `vlm_latency()`: `bench_local_vlm.py frames`'s output document.
* P12 (map) -- `MapPoller`: the harness times `/world/map` itself (reply
  time and bytes, the ETag/304 path as explore uses it).
* Every row -- `percentile()`: nearest rank, the boards' convention
  (`bench_identity_llama.percentile`, headroom's p90).
"""

from __future__ import annotations

import bisect
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional, Sequence

FRAME_RE = re.compile(r"(\d\d:\d\d:\d\d) (\d+) \w+")
NAV_RE = re.compile(r"goals (\d+)/(\d+) succeeded")


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    s = sorted(values)
    if not s:
        return None
    return s[max(0, min(len(s) - 1, math.ceil(q / 100 * len(s)) - 1))]


def _epoch(day: str, hms: str) -> float:
    return time.mktime(time.strptime(f"{day} {hms}", "%Y-%m-%d %H:%M:%S"))


def _next_day(day: str) -> str:
    # Through noon of the day, so a DST change (2026-11-01) never lands on the wrong date.
    return time.strftime("%Y-%m-%d", time.localtime(_epoch(day, "12:00:00") + 86400))


def _after(day: str, hms: str, ref: float) -> float:
    """A bare HH:MM:SS stamp on `day`, or on the next day when it would sit
    more than 12 h before `ref` (a run crossing midnight)."""
    t = _epoch(day, hms)
    return _epoch(_next_day(day), hms) if t < ref - 43200 else t


def perception_ms(log_text: str, t0: Optional[float] = None, t1: Optional[float] = None,
                  day: Optional[str] = None) -> list:
    """Frame times (ms) from perception_loop.py's log. With a window, only
    frames stamped inside it; `day` (YYYY-MM-DD, the board's) dates the
    HH:MM:SS stamps (default: t0's local day)."""
    if day is None and (t0 is not None or t1 is not None):
        day = time.strftime("%Y-%m-%d", time.localtime(t0 if t0 is not None else t1))
    out = []
    for line in log_text.splitlines():
        m = FRAME_RE.fullmatch(line.strip())
        if not m:
            continue
        if t0 is not None or t1 is not None:
            at = _after(day, m.group(1), t0) if t0 is not None else _epoch(day, m.group(1))
            if (t0 is not None and at < int(t0)) or (t1 is not None and at > t1):
                continue
        out.append(int(m.group(2)))
    return out


def wheel_delta(h0: dict, h1: dict) -> dict:
    """Two `/health` documents (or their `wheel_loop` blocks) -> the window's
    moving and late ticks, and the running max_dt_s at its end."""
    a = h0.get("wheel_loop", h0)
    b = h1.get("wheel_loop", h1)
    return {"moving": b["moving_ticks"] - a["moving_ticks"],
            "late": b["late_ticks"] - a["late_ticks"],
            "max_dt_s": b["max_dt_s"], "max_dt_s_before": a["max_dt_s"]}


def nav_counts(nav_log_text: str) -> Optional[dict]:
    """headroom's nav load prints `goals <ok>/<n> succeeded` last."""
    ms = NAV_RE.findall(nav_log_text)
    if not ms:
        return None
    ok, n = ms[-1]
    return {"ok": int(ok), "n": int(n)}


def headroom_window(run_log_text: str, day: str) -> tuple:
    """headroom_identity.sh's `== window start HH:MM:SS` / `== window end
    HH:MM:SS` lines -> epochs on the board's `day` (None when missing). An
    end stamped before the start crossed midnight."""
    s = re.search(r"== window start (\d\d:\d\d:\d\d)", run_log_text)
    e = re.search(r"== window end (\d\d:\d\d:\d\d)", run_log_text)
    t0 = _epoch(day, s.group(1)) if s else None
    t1 = None
    if e:
        t1 = _epoch(day, e.group(1))
        if t0 is not None and t1 < t0:
            t1 = _epoch(_next_day(day), e.group(1))
    return t0, t1


def vlm_latency(doc: dict) -> dict:
    """bench_local_vlm.py's `latency_ms` (n, p50, p90, max) and statuses."""
    return {**doc["latency_ms"], "statuses": doc.get("statuses")}


class MapPoller:
    """Times `GET /world/map` as explore fetches it (with the last ETag, so
    an unchanged map is a 304), one request every `period_s`, on a thread."""

    def __init__(self, url: str, secret: str = "", period_s: float = 0.5, timeout_s: float = 10.0):
        self.url, self.secret, self.period_s, self.timeout_s = url, secret, period_s, timeout_s
        self.rows: list = []
        self._stop = threading.Event()
        self._t: Optional[threading.Thread] = None
        self._etag: Optional[str] = None

    def one(self) -> dict:
        hdr = {"x-app-secret": self.secret} if self.secret else {}
        if self._etag:
            hdr["If-None-Match"] = self._etag
        req = urllib.request.Request(f"{self.url}/world/map", headers=hdr)
        t0, start = time.perf_counter(), time.time()
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                body = r.read()
                status = r.status
                self._etag = r.headers.get("ETag") or self._etag
        except urllib.error.HTTPError as e:
            body, status = b"", e.code
        except (urllib.error.URLError, OSError) as e:
            body, status = b"", f"error {type(e).__name__}"
        return {"t": round(start, 3), "ms": (time.perf_counter() - t0) * 1000,
                "status": status, "bytes": len(body)}

    def start(self) -> None:
        def loop():
            while not self._stop.is_set():
                self.rows.append(self.one())
                self._stop.wait(self.period_s)
        self._t = threading.Thread(target=loop, daemon=True)
        self._t.start()

    def stop(self) -> list:
        self._stop.set()
        if self._t:
            self._t.join(timeout=self.timeout_s + 2)
        return self.rows


def map_summary(rows: list) -> dict:
    ok = [r["ms"] for r in rows if r["status"] in (200, 304)]
    return {"n": len(rows), "ok": len(ok), "p50_ms": percentile(ok, 50), "p90_ms": percentile(ok, 90),
            "full_replies": sum(r["status"] == 200 for r in rows),
            "bytes_max": max((r["bytes"] for r in rows), default=0)}


def load_json(path: Path) -> dict:
    return json.loads(Path(path).read_text())


# ------------------------------------------------- 3.136.B: P11, P12, P4

EXECUTING = 2                       # action_msgs/GoalStatus.STATUS_EXECUTING
TERMINAL = {4: "succeeded", 5: "canceled", 6: "aborted"}


def jsonl(path) -> list:
    """Rows of a JSON-lines file; a line cut by a kill is skipped."""
    out = []
    p = Path(path)
    if not p.exists():
        return out
    for line in p.read_text().splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


def _within(rows, t0, t1, key="t"):
    return [r for r in rows if (t0 is None or r[key] >= t0) and (t1 is None or r[key] <= t1)]


def executing_spans(pace_rows: list, t0=None, t1=None) -> list:
    """`/follow_path` status rows -> [start, end] spans while some goal is
    EXECUTING (the controller's loop runs only then); clipped to the window."""
    spans, start = [], None
    for r in sorted((r for r in pace_rows if r.get("k") == "fp"), key=lambda r: r["t"]):
        busy = any(s == EXECUTING for _, s in r.get("goals") or [])
        if busy and start is None:
            start = r["t"]
        elif not busy and start is not None:
            spans.append([start, r["t"]])
            start = None
    if start is not None and t1 is not None:
        spans.append([start, t1])
    out = []
    for a, b in spans:
        a, b = max(a, t0) if t0 is not None else a, min(b, t1) if t1 is not None else b
        if b > a:
            out.append([a, b])
    return out


def controller_rate(pace_rows: list, t0=None, t1=None, min_msgs: int = 3) -> dict:
    """P11: nav2's controller output (`cmd_vel/nav`) inside the spans where
    FollowPath is executing. Per span, (messages - 1) / (last - first); the
    pooled rate is the sum of the intervals over the sum of their time, and
    `periods_ms` are the gaps between messages in a span (its p50/p90)."""
    spans = executing_spans(pace_rows, t0, t1)
    cmds = sorted(r["t"] for r in pace_rows if r.get("k") == "cmd")
    n_int, dur, gaps = 0, 0.0, []
    for a, b in spans:
        ts = cmds[bisect.bisect_left(cmds, a):bisect.bisect_right(cmds, b)]
        if len(ts) < min_msgs:
            continue
        n_int += len(ts) - 1
        dur += ts[-1] - ts[0]
        gaps += [(y - x) * 1000 for x, y in zip(ts, ts[1:])]
    rate = n_int / dur if dur > 0 else None
    return {"spans": len(spans), "intervals": n_int, "executing_s": round(dur, 1),
            "rate_hz": round(rate, 3) if rate else None,
            "mean_period_ms": round(1000 / rate, 2) if rate else None,
            "period_p50_ms": percentile(gaps, 50), "period_p90_ms": percentile(gaps, 90)}


def scan_lag(pace_rows: list, t0=None, t1=None) -> dict:
    """P11: a scan's age on arrival at a ROS subscriber (receipt minus its
    header stamp, which the bridge sets to the capture time since 3.40)."""
    lags = [(r["t"] - r["stamp"]) * 1000 for r in _within(pace_rows, t0, t1)
            if r.get("k") == "scan" and r.get("stamp")]
    return {"n": len(lags), "p50_ms": percentile(lags, 50), "p90_ms": percentile(lags, 90),
            "max_ms": max(lags) if lags else None}


def nav_goals(pace_rows: list, t0=None, t1=None) -> dict:
    """Each `/navigate_to_pose` goal's end (succeeded / canceled / aborted),
    for goals that ended inside the window."""
    ended = {}
    for r in sorted((r for r in pace_rows if r.get("k") == "nav"), key=lambda r: r["t"]):
        for gid, s in r.get("goals") or []:
            if s in TERMINAL and gid not in ended:
                ended[gid] = (r["t"], TERMINAL[s])
    inside = [s for t, s in ended.values() if (t0 is None or t >= t0) and (t1 is None or t <= t1)]
    return {"n": len(inside), **{v: inside.count(v) for v in TERMINAL.values()}}


def map_updates(pace_rows: list, t0=None, t1=None) -> list:
    return sorted(r["t"] for r in _within(pace_rows, t0, t1) if r.get("k") == "map")


def shim_spans(shim_rows: list, kind: str, t0=None, t1=None, caller=None) -> list:
    """The shim's [t0, t1] spans of one kind (and calling file, if named)
    that start inside the window."""
    return sorted([r["t0"], r["t1"]] for r in shim_rows
                  if r.get("k") == kind and (caller is None or r.get("caller") == caller)
                  and (t0 is None or r["t0"] >= t0)
                  and (t1 is None or r["t0"] <= t1))


def durations_ms(spans: list) -> list:
    return [(b - a) * 1000 for a, b in spans]


def room_work(shim_rows: list, t0: float, t1: float) -> dict:
    """P12: `segment()` and spacing's walk per call, and the pause they make
    on the mission thread: their summed time over the window's length."""
    def stats(v):
        return {"n": len(v), "p50_ms": percentile(v, 50), "p90_ms": percentile(v, 90),
                "max_ms": max(v) if v else None}
    if t0 is None or t1 is None:
        return {"segment": stats([]), "walk": stats([]), "pause_share": None}
    seg = durations_ms(shim_spans(shim_rows, "segment", t0, t1))
    walk = durations_ms(shim_spans(shim_rows, "walk", t0, t1))
    pause = (sum(seg) + sum(walk)) / 1000 / (t1 - t0) if t1 > t0 else None
    by_caller = {c: stats(durations_ms(shim_spans(shim_rows, "segment", t0, t1, c)))
                 for c in sorted({r.get("caller") for r in shim_rows if r.get("k") == "segment"}, key=str)}
    return {"segment_by_caller": by_caller,
            "segment": {"n": len(seg), "p50_ms": percentile(seg, 50), "p90_ms": percentile(seg, 90),
                        "max_ms": max(seg) if seg else None},
            "walk": {"n": len(walk), "p50_ms": percentile(walk, 50), "p90_ms": percentile(walk, 90),
                     "max_ms": max(walk) if walk else None},
            "pause_share": round(pause, 4) if pause is not None else None}


def _intersect(a: list, b: list) -> list:
    """Two sorted span lists -> the spans where both are in progress."""
    out, i, j = [], 0, 0
    while i < len(a) and j < len(b):
        lo, hi = max(a[i][0], b[j][0]), min(a[i][1], b[j][1])
        if hi > lo:
            out.append([lo, hi])
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def _merge(spans: list) -> list:
    out = []
    for a, b in sorted(spans):
        if out and a <= out[-1][1]:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def overlaps(groups: dict, instants: Sequence[float]) -> dict:
    """P4's count: `groups` maps a name to its spans (explore's map fetch, a
    VE question, a perception frame, the room cut); the spans where ALL are
    in progress, how many there are and their time, and how many of them a
    SLAM map update (`instants`, /map arrivals) lands in."""
    both = None
    for spans in groups.values():
        m = _merge(spans)
        both = m if both is None else _intersect(both, m)
    both = both or []
    inst = sorted(instants)
    hit = sum(bisect.bisect_right(inst, b) > bisect.bisect_left(inst, a) for a, b in both)
    return {"all_spans": len(both), "all_s": round(sum(b - a for a, b in both), 3),
            "all_with_map_update": hit, "per_group": {k: len(v) for k, v in groups.items()}}
