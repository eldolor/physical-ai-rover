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
