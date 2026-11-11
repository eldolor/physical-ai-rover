# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.174.B: the real-mission recorder, an observer on the desktop beside the twin.

    .venv/bin/python evaluations/real-missions-3174/recorder.py --robot http://<board>:8000 \\
        --brain http://<board>:8001 --batch <name> [--missions N] [--interventions <path>]

A client of the robot server and the brain over HTTP, GET requests only, so it can
neither move the robot nor change a mission. While a mission runs it keeps:

* G1, one row in `rows.jsonl`: the end status, the report's counts, `/health` and
  `/battery` at the start and end (refusal, watchdog and wheel-loop deltas, volts),
  the intervention log's new line, and each 3.174 section 1 measure the row cannot
  hold, named `unmeasurable` with its reason;
* G2, the pose track, `track/<mission>.jsonl`, from `/world/pose` at 2 Hz;
* G4, the frames, from `/frame` every 2 s, as `LocalWalkStore` walks under `walks/`
  (split at the 500-frame cap), the layout `walk_eval` and 3.80.U's `make_sets.py` read.

Everything goes to a batch directory outside the repository (default
`~/picar-kept/real-missions/<batch>/`), and one inside a git work tree is refused:
real frames, maps and tracks are the user's home (3.174 section 6). `numbers_only()`
is the row as it may be copied into `evaluations/real-missions-3174/<batch>/`.
The secret is read from `APP_SHARED_SECRET`, never a flag.
"""
import argparse
import base64
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.recording_routes import (MAX_FRAMES_PER_WALK,  # noqa: E402
                                      finish_walk_meta, store_frame)
from control.walk_store import LocalWalkStore  # noqa: E402

TICK_S = 0.5            # the pose's rate, 2 Hz
FRAME_EVERY_S = 2.0     # a frame each 2 s by default (`--frame-every-s`; 0 keeps none)
STATUS_EVERY = 2        # ticks: the status each 1 s
BATTERY_EVERY = 10      # ticks: the pack each 5 s
IDLE_S = 1.0            # between status reads while no mission runs
END_GRACE_S = 30.0      # after `running` turns False: the report and the return land
STATUS_LOST_S = 120.0   # no status of this mission for this long: its row is written
WALK_MAX = 64           # control.recording_routes.WALK_NAME's length

# 3.174 section 1's measures this row cannot hold, and why.
UNMEASURABLE = {
    "found_on_target": "a person at the robot, with found.jpg, in the mission's note",
    "wrong_found": "a person, against the house sheet's targets",
    "contacts": "a person or the phone's video: the Rover has no bumper",
    "min_gap_cm": "the lidar's nearest return is the veto's own reading",
    "falls": "a person or the phone's video",
    "overhang_cm": "no sensor reads it",
    "rooms_named_right": "the user's names (3.160) against rooms_entered",
    "wh": "the board's input power, after 3.136.D",
    "mem_available_min": "the board's /proc/meminfo, copied with the logs (G3)",
}
# A row never carries these: a position, a frame or a map (3.174 section 6).
PRIVATE_KEYS = {"x_m", "y_m", "heading_deg", "pose", "track", "map", "map_id", "frame_id", "image",
                "image_base64", "cells", "rooms_entered"}


def refuse_in_repo(path: Path) -> Path:
    """The batch directory, made; refused if inside a git work tree."""
    p = Path(path).expanduser().resolve()
    for d in (p, *p.parents):
        if (d / ".git").exists():
            raise SystemExit(f"{p} is inside the git work tree {d}: real missions are kept "
                             f"outside any repository (3.174 section 6)")
    p.mkdir(parents=True, exist_ok=True)
    return p


def distance_m(track) -> float:
    """Metres driven along the track, on one map: a sample on another map id
    (a re-map) or another frame id (a session anchored on the saved map, review)
    starts a new leg rather than a jump."""
    d, last = 0.0, None
    for s in track:
        if last is not None and (s.get("map_id"), s.get("frame_id")) == \
                (last.get("map_id"), last.get("frame_id")):
            d += math.hypot(s["x_m"] - last["x_m"], s["y_m"] - last["y_m"])
        last = s
    return d


def rooms_from_track(track, room_at) -> list:
    """The rooms the track's samples stand in; `room_at(x_m, y_m)` names the
    room at a point of the map frame, or None."""
    return sorted({r for s in track if (r := room_at(s["x_m"], s["y_m"]))})


def coverage(times, t0, t1, within_s=1.0) -> float:
    """The share of the mission's whole seconds with a sample within `within_s`."""
    secs = range(int(math.floor(t0)), int(math.ceil(t1)))
    if not secs:
        return 1.0
    ts = sorted(times)
    import bisect
    hit = 0
    for s in secs:
        c = s + 0.5
        k = bisect.bisect_left(ts, c)
        near = [ts[j] for j in (k - 1, k) if 0 <= j < len(ts)]
        hit += any(abs(t - c) <= within_s for t in near)
    return hit / len(secs)


def _delta(a, b) -> dict:
    keys = set(a or {}) | set(b or {})
    return {k: (b or {}).get(k, 0) - (a or {}).get(k, 0) for k in sorted(keys)
            if (b or {}).get(k, 0) != (a or {}).get(k, 0)}


def numbers_only(row: dict) -> dict:
    """The row as it may be committed: no key carrying a position, a frame or a
    map, and no room names (section 6: names only as the user allows)."""
    return {k: v for k, v in row.items() if k not in PRIVATE_KEYS}


class Recorder:
    def __init__(self, robot, brain, batch_dir, interventions=None, room_at=None,
                 frame_every_s=FRAME_EVERY_S, clock=time.time, sleep=time.sleep):
        self.robot, self.brain = robot, brain          # httpx-like: .get(path) -> response
        self.dir = refuse_in_repo(batch_dir)
        self.store = LocalWalkStore(self.dir / "walks")
        (self.dir / "track").mkdir(exist_ok=True)
        self.interventions = Path(interventions) if interventions else None
        self.room_at = room_at
        self.clock, self.sleep = clock, sleep
        # ticks between kept frames; None keeps none
        self.frame_every = max(1, round(frame_every_s / TICK_S)) if frame_every_s > 0 else None

    # -- reads that never raise: a dropped link is a gap, not an end ----------
    def _get(self, client, path):
        try:
            r = client.get(path)
            return r.json() if r.status_code == 200 else None
        except Exception:  # noqa: BLE001 -- the link drops; the gap is counted
            return None

    def _lines(self):
        try:
            if self.interventions is None or not self.interventions.exists():
                return []
            return self.interventions.read_text(errors="replace").splitlines()
        except OSError:
            return []

    @staticmethod
    def _kind(line):
        """An intervention line's kind; a cut or foreign line is `unreadable` (review)."""
        try:
            return (json.loads(line).get("intervention") or {}).get("kind")
        except (ValueError, AttributeError):
            return "unreadable"

    def wait_for_start(self, give_up_s=None):
        t0 = self.clock()
        while give_up_s is None or self.clock() - t0 < give_up_s:
            s = self._get(self.brain, "/mission/status")
            if s and s.get("running") and s.get("mission_id"):
                return s
            self.sleep(IDLE_S)
        return None

    def record(self, first) -> dict:
        """One mission, from a status that shows it running, to its row."""
        mid = first["mission_id"]
        t0 = self.clock()
        h0, b0 = self._get(self.robot, "/health"), self._get(self.robot, "/battery")
        lines0 = len(self._lines())
        walk_base = mid[:WALK_MAX - 4]
        track, frames, part, seq, frame_errors = [], 0, 1, 0, 0
        gap_ticks, row_end, seen_at = 0, "ended", t0
        status, v = first, [b0.get("v")] if b0 and b0.get("v") is not None else []
        t_end, tick = None, 0
        tf = (self.dir / "track" / f"{walk_base}.jsonl").open("a")
        try:
            while True:
                now = self.clock()
                missed = False
                p = self._get(self.robot, "/world/pose")
                if p and p.get("usable"):
                    s = {"t": round(now - t0, 2), "x_m": p["x_m"], "y_m": p["y_m"],
                         "heading_deg": p.get("heading_deg"), "map_id": p.get("map_id"),
                         "frame_id": p.get("frame_id")}
                    track.append(s)
                    tf.write(json.dumps(s) + "\n")
                elif p is None:
                    missed = True
                if tick % STATUS_EVERY == 0:
                    s2 = self._get(self.brain, "/mission/status")
                    if s2 is None:
                        missed = True
                    elif s2.get("mission_id") == mid:
                        status, seen_at = s2, now
                if self.frame_every and tick % self.frame_every == 0 and t_end is None:
                    f = self._get(self.robot, "/frame")
                    if f and f.get("image_base64"):
                        # Review: a write that fails (a full disk, a bad frame) is
                        # counted; it never ends the mission's record.
                        try:
                            if seq >= MAX_FRAMES_PER_WALK:      # the walk's cap: the next part
                                finish_walk_meta(self.store, f"{walk_base}-p{part}",
                                                 target_object=status.get("target_object"))
                                part, seq = part + 1, 0
                            store_frame(self.store, f"{walk_base}-p{part}", seq,
                                        base64.b64decode(f["image_base64"]),
                                        f.get("media_type") or "image/jpeg",
                                        {"t": round(now - t0, 2),
                                         "navigate": {"action": status.get("last_action")}},
                                        count=seq)
                            seq += 1
                            frames += 1
                        except Exception:  # noqa: BLE001 -- counted in the row
                            frame_errors += 1
                    elif f is None:
                        missed = True
                if tick % BATTERY_EVERY == 0:
                    b = self._get(self.robot, "/battery")
                    if b and b.get("v") is not None:
                        v.append(b["v"])
                gap_ticks += missed
                if t_end is None and not status.get("running"):
                    t_end = now
                # `running` stays True through the return home; the report is
                # set once the robot has stopped.
                done = t_end is not None and (status.get("report") is not None
                                              or now - t_end > END_GRACE_S)
                if done:
                    break
                if now - seen_at > STATUS_LOST_S:
                    # Review: no word of this mission for that long (the brain
                    # gone, or its end missed in an outage): its row now, not never.
                    row_end, status = "status_lost", {**status, "outcome": None, "end_state": None}
                    break
                tick += 1
                self.sleep(max(0.0, TICK_S - (self.clock() - now)))
        finally:
            tf.close()
        if seq:
            try:
                finish_walk_meta(self.store, f"{walk_base}-p{part}",
                                 target_object=status.get("target_object"))
            except Exception:  # noqa: BLE001 -- counted in the row
                frame_errors += 1
        h1, b1 = self._get(self.robot, "/health"), self._get(self.robot, "/battery")
        if b1 and b1.get("v") is not None:
            v.append(b1["v"])
        rep = self._get(self.brain, f"/mission/reports/{mid}")
        new_lines = self._lines()[lines0:]
        wl0, wl1 = (h0 or {}).get("wheel_loop") or {}, (h1 or {}).get("wheel_loop") or {}
        ret = status.get("return") or {}
        budget = status.get("budget") or {}
        row = {
            "mission_id": mid, "started_at": round(t0, 2), "job": status.get("job"),
            "policy": status.get("policy"),
            "target": status.get("target_object"),
            "outcome": status.get("outcome"), "end_state": status.get("end_state"),
            "end_reason": (status.get("end_reason") or "")[:200], "error": status.get("error"),
            "seconds": round((t_end or self.clock()) - t0, 1), "elapsed_s": budget.get("elapsed_s"),
            "ret_state": ret.get("state"), "ret_seconds": ret.get("elapsed_s"),
            "camera_failures": status.get("camera_failures"),
            "refusals": _delta((h0 or {}).get("refusal_counts"), (h1 or {}).get("refusal_counts")),
            "goal_lease_lapses": _delta((h0 or {}).get("goal_lease"), (h1 or {}).get("goal_lease"))
            if isinstance((h0 or {}).get("goal_lease"), dict) else None,
            "late_ticks": wl1.get("late_ticks", 0) - wl0.get("late_ticks", 0) if h0 and h1 else None,
            "moving_ticks": wl1.get("moving_ticks", 0) - wl0.get("moving_ticks", 0) if h0 and h1 else None,
            "v_start": v[0] if v else None, "v_end": v[-1] if v else None,
            "v_min": min(v) if v else None,
            "battery_level_end": (b1 or {}).get("level"),
            "distance_m": round(distance_m(track), 2),
            "pose_samples": len(track),
            "pose_coverage": round(coverage([s["t"] for s in track], 0.0,
                                             (t_end or self.clock()) - t0), 4),
            "frames_kept": frames, "walks": [f"{walk_base}-p{k}" for k in range(1, part + 1)] if frames else [],
            "gap_s": round(gap_ticks * TICK_S, 1),
            "report": "kept" if rep else "missing",
            "report_landmarks": len(rep.get("landmarks") or rep.get("things") or []) if rep else None,
            "intervention_lines": len([x for x in new_lines if x.strip()]) if self.interventions else "missing",
            "interventions": [self._kind(x) for x in new_lines if x.strip()]
            if self.interventions else "missing",
            "frame_errors": frame_errors, "row_end": row_end,
            "health": "kept" if h0 and h1 else "missing",
            "battery": "kept" if b0 and b1 else "missing",
            "unmeasurable": UNMEASURABLE,
        }
        if self.room_at is not None:
            row["rooms_entered"] = rooms_from_track(track, self.room_at)
            row["rooms_entered_n"] = len(row["rooms_entered"])
        with (self.dir / "rows.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")
        return row

    def run(self, missions=None):
        n = 0
        while missions is None or n < missions:
            s = self.wait_for_start()
            row = self.record(s)
            print(row["mission_id"], row["outcome"], row["seconds"], "s, frames", row["frames_kept"],
                  "poses", row["pose_samples"], "gap", row["gap_s"], flush=True)
            n += 1


def main(argv=None):
    import httpx
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--brain", required=True)
    ap.add_argument("--batch", required=True)
    ap.add_argument("--root", default="~/picar-kept/real-missions")
    ap.add_argument("--missions", type=int, default=None)
    ap.add_argument("--interventions", default=None)
    ap.add_argument("--frame-every-s", type=float, default=FRAME_EVERY_S,
                    help="seconds between kept frames (0: none); 3.174.B's B6 checks the default")
    a = ap.parse_args(argv)
    headers = {"x-app-secret": os.environ.get("APP_SHARED_SECRET", "")}
    robot = httpx.Client(base_url=a.robot, timeout=5, headers=headers)
    brain = httpx.Client(base_url=a.brain, timeout=5, headers=headers)
    Recorder(robot, brain, Path(a.root).expanduser() / a.batch, a.interventions,
             frame_every_s=a.frame_every_s).run(a.missions)


if __name__ == "__main__":
    main()
