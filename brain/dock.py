# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/dock.py -- 3.156.B: the return to the dock
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, design D1-D6).

* `dock_from_pose()`: the dock is where the mission started (D1's `start`
  source: in the simulator the start pose is the dock stand-in; under ROS a
  reload localises from the saved dock, a fresh map's start is its best
  guess, so it is never called verified).
* `HomeBudget`: D3's return reserve. `T_home = K * L_home / v_obs + T_DOCK_S`,
  `L_home` the path length home on the brain's own map (`brain/frontier.py`'s
  `reach`, from the dock), `v_obs` this mission's own driving speed; the
  battery predicted at the dock from this mission's drain rate.
* `ReturnTrip`: D2/D4's phase, one action a tick: a nav2 goal to a staging
  point `STAGING_M` behind the dock along its heading, then a turn to face
  the dock, a creep on guarded verbs (every one through `robot/safety.py`),
  and a final turn to the dock's heading. Docked: the pose within
  `DOCK_TOL_M` and `DOCK_TOL_DEG`.

Headings are compass bearings (clockwise from north, `get_pose()`'s
`heading_deg`); a robot facing `h` moves along (sin h, -cos h) in (x_m, y_m).
"""

import math
from typing import Callable, Optional

from brain.frontier import CELL_FREE, PASS_CLEARANCE_M, _cell_of, _grid, reach

# D3's design constants (held to the data by C4/C5).
K = 1.5
T_DOCK_S = 30.0
V_FLOOR_M_S = 0.05
V_EARLY_M_S = 0.10
EARLY_DRIVE_S = 30.0
DRAIN_SPAN_S = 60.0
BATTERY_MARGIN_V = 0.1
REPLAN_M = 0.5
REPLAN_S = 30.0
MOVED_M = 0.01              # a tick that moved less did not drive
MAX_GAP_S = 10.0            # one read's share of driving time, at most (a verb runs <= 7 s)
# D4. 0.30 m, not the design's 0.40 (amended before any measurement, the
# section's B notes): the simulator's straight verbs move whole 0.30 m cells
# (`MockRobot._speed_duration_to_cells`), so a 0.40 m creep cannot close
# there; on the car the verbs are continuous and step C may restore it.
STAGING_M = 0.30
STAGING_CLEAR_M = 0.25      # free map around the staging point, or straight to the dock
DOCK_TOL_M = 0.10
DOCK_TOL_DEG = 10.0
TURN_TOL_DEG = 3.0
CREEP_TOL_M = 0.03
CREEP_SPEED = 20            # a straight verb's 0-100 speed for the creep
CREEP_VERB_MAX_S = 1.5
MAX_ACTIONS = 40            # verbs in the final approach, then `not_aligned`
MAX_REFUSED = 3             # consecutive vetoed verbs, then `vetoed`
GOAL_TRIES = 2              # nav2 goals per target (staging, then the dock), a back-off between
BACK_OFF_S = 0.75           # the back-off: one guarded REVERSE at creep speed
RETURN_WAIT_S = 30.0        # D2: nav2's hold never clears -> skipped, no_nav2
GOAL_RETRY_S = 1.0          # between refused goals (through the runner's idle)
POLL_S = 0.5                # between polls of a goal under way
RETURN_MAX_S = 600.0        # a return that never ends is a failed one

DOCKED, FAILED, CANCELLED, SKIPPED = "docked", "failed", "cancelled", "skipped"


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def ahead(heading_deg: float) -> tuple:
    h = math.radians(heading_deg)
    return math.sin(h), -math.cos(h)


def dock_from_pose(pose: Optional[dict], source: str = "start") -> Optional[dict]:
    if not pose or not pose.get("usable"):
        return None
    try:
        return {"x_m": float(pose["x_m"]), "y_m": float(pose["y_m"]),
                "theta": float(pose["heading_deg"]), "source": source}
    except (KeyError, TypeError, ValueError):
        return None


def staging_point(dock: dict, back_m: float = STAGING_M) -> tuple:
    fx, fy = ahead(dock["theta"])
    return dock["x_m"] - back_m * fx, dock["y_m"] - back_m * fy


def pose_error(dock: dict, pose: dict) -> tuple:
    """(metres, degrees) between a pose and the dock."""
    return (math.hypot(pose["x_m"] - dock["x_m"], pose["y_m"] - dock["y_m"]),
            abs(wrap_deg(pose["heading_deg"] - dock["theta"])))


def clear_on_map(m: dict, x_m: float, y_m: float, radius_m: float = STAGING_CLEAR_M) -> bool:
    """Every map cell within `radius_m` of (x, y) is known free."""
    if not m or not m.get("usable") or not m.get("resolution_m"):
        return False
    cells = _grid(m)
    cx, cy = _cell_of(m, x_m, y_m)
    r = int(math.ceil(radius_m / m["resolution_m"]))
    h, w = cells.shape
    if not (r <= cx < w - r and r <= cy < h - r):
        return False
    return bool((cells[cy - r:cy + r + 1, cx - r:cx + r + 1] == CELL_FREE).all())


def home_path_m(m: dict, dock: dict, x_m: float, y_m: float) -> Optional[float]:
    """The path length from (x, y) to the dock on the map (from the dock's
    side: `reach` keeps where a robot can be), or None if the map does not
    connect them."""
    r = reach(m, dock["x_m"], dock["y_m"], clearance_m=PASS_CLEARANCE_M)
    if not len(r):
        return None
    cx, cy = _cell_of(m, x_m, y_m)
    d = r.dense
    best = math.inf
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            x, y = cx + dx, cy + dy
            if 0 <= x < r.w and 0 <= y < r.h and not math.isnan(d[y, x]):
                best = min(best, d[y, x] + math.hypot(dx, dy))
    return None if best == math.inf else best * m["resolution_m"]


class HomeBudget:
    """D3: what the trip home is predicted to cost, and whether the search
    must end now to afford it."""

    def __init__(self, dock: dict):
        self.dock = dock
        self.metres = 0.0
        self.driving_s = 0.0
        self._last = None               # (x, y, t)
        self._samples = []              # distinct (t, v) battery samples
        self._path = None               # (L_home, x, y, t)
        self.last = {}

    def observe_pose(self, pose: dict, t: float) -> None:
        if not pose or not pose.get("usable"):
            return
        x, y = pose["x_m"], pose["y_m"]
        if self._last is not None:
            d = math.hypot(x - self._last[0], y - self._last[1])
            if d >= MOVED_M:
                self.metres += d
                # A pause between reads (a cooldown, a pick) is not driving.
                self.driving_s += min(MAX_GAP_S, max(0.0, t - self._last[2]))
        self._last = (x, y, t)

    def observe_battery(self, v: float, t: float) -> None:
        """A distinct sample (3.137's rule: a repeated reading is the board's
        last one again, not a new measurement)."""
        if self._samples and self._samples[-1][1] == v:
            return
        self._samples.append((t, float(v)))

    def v_obs(self) -> float:
        if self.driving_s < EARLY_DRIVE_S:
            return V_EARLY_M_S
        return max(V_FLOOR_M_S, self.metres / self.driving_s)

    def drain_v_s(self) -> float:
        """V/s over distinct samples at least DRAIN_SPAN_S apart; 0 before."""
        if len(self._samples) < 2:
            return 0.0
        (t0, v0), (t1, v1) = self._samples[0], self._samples[-1]
        if t1 - t0 < DRAIN_SPAN_S:
            return 0.0
        return max(0.0, (v0 - v1) / (t1 - t0))

    def path_m(self, get_map: Callable[[], dict], pose: dict, t: float) -> float:
        x, y = pose["x_m"], pose["y_m"]
        p = self._path
        if p is None or math.hypot(x - p[1], y - p[2]) >= REPLAN_M or t - p[3] >= REPLAN_S:
            L = None
            try:
                m = get_map()
                if m and m.get("usable"):
                    L = home_path_m(m, self.dock, x, y)
            except Exception:  # noqa: BLE001 -- a map read lost, the straight line kept
                L = None
            if L is None:
                L = K * math.hypot(x - self.dock["x_m"], y - self.dock["y_m"])
            self._path = p = (L, x, y, t)
        return p[0]

    def t_home(self, L: float) -> float:
        return K * L / self.v_obs() + T_DOCK_S

    def reserve_reason(self, get_map, pose: dict, t: float, left_s: Optional[float],
                       v_now: Optional[float], floor_v: Optional[float]) -> Optional[str]:
        """Why the search must end now to get home, or None."""
        if not pose or not pose.get("usable"):
            return None
        L = self.path_m(get_map, pose, t)
        if math.hypot(pose["x_m"] - self.dock["x_m"], pose["y_m"] - self.dock["y_m"]) < REPLAN_M:
            self.last = {"l_home_m": round(L, 2), "at_dock": True}
            return None     # at the dock: nothing to drive home (a short limit is not a reserve)
        th = self.t_home(L)
        r = self.drain_v_s()
        self.last = {"l_home_m": round(L, 2), "t_home_s": round(th, 1),
                     "v_obs_m_s": round(self.v_obs(), 3), "drain_v_s": round(r, 5)}
        if left_s is not None and th > left_s:
            return f"time: {left_s:.0f} s left, the trip home {th:.0f} s ({L:.1f} m)"
        if v_now is not None and floor_v:
            at_dock = v_now - K * r * th
            self.last["v_at_dock"] = round(at_dock, 2)
            if at_dock < floor_v + BATTERY_MARGIN_V:
                return (f"battery: {v_now:.2f} V now, {at_dock:.2f} V predicted at the "
                        f"dock (floor {floor_v:g} V + {BATTERY_MARGIN_V:g})")
        return None


class ReturnTrip:
    """D2/D4: the drive home, one action per `step()`. `step()` returns None
    while under way, else (state, reason). Exceptions from the navigator or a
    verb (a person's Stop, a preemption, the battery's Reserve) propagate to
    the runner, which owns what they mean."""

    def __init__(self, dock: dict, navigator, safety, get_pose: Callable[[], dict],
                 clock: Callable[[], float], wait_s: float = RETURN_WAIT_S,
                 max_s: float = RETURN_MAX_S, idle: Optional[Callable[[float], None]] = None,
                 get_map: Optional[Callable[[], dict]] = None):
        self.dock, self.nav, self.safety = dock, navigator, safety
        self.get_pose, self.clock = get_pose, clock
        self.wait_s, self.max_s = wait_s, max_s
        self.idle = idle
        self.get_map = get_map
        self._staging_checked = False
        self.t0 = clock()
        self.state = "goal"
        self.target = staging_point(dock)
        self.direct = False             # the staging point refused: a goal to the dock itself
        self.tries = 0                  # goals sent to the current target
        self.goal_sent_at = None
        self.actions = 0
        self.refused = 0
        self.reversals = 0
        self.metres = 0.0
        self._last_xy = None
        self.goal_id = None
        self.log = []
        self.steps = 0                  # approach steps, acting or not (a loop with no verb ends)
        self._last_along = None         # the creep's remaining distance before its last verb
        self._step_m = None             # what that verb closed

    def summary(self) -> dict:
        return {"phase": self.state, "actions": self.actions, "reversals": self.reversals,
                "metres": round(self.metres, 2), "elapsed_s": round(self.clock() - self.t0, 1),
                "goal_id": self.goal_id, "direct": self.direct}

    def _pose(self) -> dict:
        p = self.get_pose() or {}
        if p.get("usable"):
            xy = (p["x_m"], p["y_m"])
            if self._last_xy is not None:
                self.metres += math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1])
            self._last_xy = xy
        return p

    def step(self) -> Optional[tuple]:
        if self.clock() - self.t0 > self.max_s:
            return FAILED, f"timeout: not docked in {self.max_s:.0f} s"
        if self.nav is None:
            return SKIPPED, "no_nav2"
        if self.state == "goal":
            return self._goal()
        if self.state == "driving":
            return self._driving()
        return self._approach()

    def _goal(self):
        if not self.direct and not self._staging_checked:
            self._staging_checked = True
            try:
                tight = self.get_map is not None and not clear_on_map(self.get_map(), *self.target)
            except Exception:  # noqa: BLE001 -- no map: the staging point is tried as planned
                tight = False
            if tight:
                self._to_dock("the staging point is tight on the map")
        try:
            reply = self.nav.set_goal(*self.target) or {}
        except Exception as e:  # noqa: BLE001 -- a hold or a lost answer; re-raised below if not
            if type(e).__name__ in ("StoppedByPerson", "AutonomyLapsed", "BatteryReserve",
                                    "Preempted", "MissionHalted"):
                raise
            reply = {"accepted": False, "reason": f"error: {e}"}
        if reply.get("accepted"):
            self.tries += 1
            self.goal_id = reply.get("goal_id")
            self.state, self.goal_sent_at = "driving", self.clock()
            self.log.append(f"goal to ({self.target[0]:.2f}, {self.target[1]:.2f}) accepted")
            return None
        if self.clock() - self.t0 >= self.wait_s:
            return SKIPPED, f"no_nav2: goal not taken in {self.wait_s:.0f} s ({reply.get('reason')})"
        if self.idle is not None:
            self.idle(GOAL_RETRY_S)
        return None

    def _driving(self):
        self._pose()                    # metres home are counted on nav2's leg too (C4)
        goal = (self.nav.get_goal() or {}).get("goal") or {}
        st = goal.get("state")
        if st in ("pending", "active", None) and goal:
            if self.idle is not None:
                self.idle(POLL_S)
            return None
        if st == "succeeded":
            self.state = "turn"
            return None
        # aborted, canceled or gone: back off (the runner's 3.31 move, as
        # nav2's own recovery would) and try again; the staging point twice,
        # then the dock itself twice.
        if self.tries >= GOAL_TRIES:
            if self.direct:
                return FAILED, f"nav2_{st or 'lost'}"
            self._to_dock(f"staging goal {st}")
        else:
            self.log.append(f"goal {st}; backing off and trying again")
            self.state = "goal"
        self._act("REVERSE", speed=CREEP_SPEED, duration=BACK_OFF_S)
        self.reversals += 1
        return None

    def _to_dock(self, why: str) -> None:
        self.direct, self.tries, self.refused = True, 0, 0
        self.target, self.state = (self.dock["x_m"], self.dock["y_m"]), "goal"
        self.log.append(f"{why}; a goal to the dock itself")

    def _act(self, action: str, **kw) -> None:
        from robot.safety import SafetyViolation
        self.actions += 1
        try:
            self.safety.check_and_execute(action, **kw)
            self.refused = 0
        except SafetyViolation as e:
            self.refused += 1
            self.log.append(f"{action} refused: {e}")

    def _turn_to(self, pose: dict, bearing: float) -> bool:
        """One turn toward `bearing`; False when already within tolerance."""
        d = wrap_deg(bearing - pose["heading_deg"])
        if abs(d) <= TURN_TOL_DEG:
            return False
        self._act("RIGHT" if d > 0 else "LEFT", angle=max(1, int(round(abs(d)))))
        return True

    def _approach(self):
        self.steps += 1
        if self.actions >= MAX_ACTIONS or self.steps >= 3 * MAX_ACTIONS:
            return FAILED, "not_aligned"
        if self.refused >= MAX_REFUSED:
            if not self.direct:
                # The staging point is too tight to turn in (the pivot guard
                # refused): nav2 goals to the dock itself, where the robot
                # started and so has turned before, then the final turn there.
                self._to_dock("turns refused at the staging point")
                return None
            return FAILED, "vetoed"
        pose = self._pose()
        if not pose.get("usable"):
            return FAILED, "no_pose"
        dx, dy = self.dock["x_m"] - pose["x_m"], self.dock["y_m"] - pose["y_m"]
        dist = math.hypot(dx, dy)
        if self.state == "turn":
            # Face the dock (its bearing from here; its heading once close).
            bearing = (math.degrees(math.atan2(dx, -dy)) % 360.0
                       if dist > CREEP_TOL_M + 0.02 else self.dock["theta"])
            if not self._turn_to(pose, bearing):
                self.state = "creep" if dist > CREEP_TOL_M else "final"
            return None
        if self.state == "creep":
            fx, fy = ahead(pose["heading_deg"])
            along = dx * fx + dy * fy
            if self._last_along is not None:
                self._step_m = abs(self._last_along - along) or self._step_m
            if (dist <= CREEP_TOL_M or (abs(along) <= CREEP_TOL_M and dist <= DOCK_TOL_M)
                    or (self._step_m and abs(along) < 0.5 * self._step_m)):
                # Close enough, or the next verb would overshoot by more than
                # it closes (a body whose verbs move in steps, as the sim's).
                self.state = "final"
                self._last_along = None
                return None
            off = abs(wrap_deg(math.degrees(math.atan2(dx, -dy)) - pose["heading_deg"]))
            if along > 0 and off > 15.0 and dist > DOCK_TOL_M:
                self.state = "turn"
                return None
            from robot.safety import verb_speed_m_s
            v = verb_speed_m_s(CREEP_SPEED)
            dur = round(min(CREEP_VERB_MAX_S, abs(along) / v), 2)
            self._last_along = along
            if along > 0:
                self._act("FORWARD", speed=CREEP_SPEED, duration=dur)
            else:
                self.reversals += 1
                self._act("REVERSE", speed=CREEP_SPEED, duration=dur)
            return None
        # final: the dock's heading, then judge.
        if self._turn_to(pose, self.dock["theta"]):
            return None
        m, deg = pose_error(self.dock, pose)
        if m <= DOCK_TOL_M and deg <= DOCK_TOL_DEG:
            return DOCKED, f"{m:.3f} m, {deg:.1f} deg from the dock"
        if dist > DOCK_TOL_M:
            self.state = "turn"
            return None
        return FAILED, f"not_aligned: {m:.3f} m, {deg:.1f} deg"
