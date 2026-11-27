# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/dock.py -- 3.156.B: the return to the dock
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, design D1-D6).

* `dock_from_pose()`: the saved map's dock when the pose names one (3.156.E,
  D1's `saved_map` source: under ROS a matched reload's `dock_pose`, the same
  point every mission on that map), else where the mission started (`start`:
  in the simulator the start pose is the dock stand-in; a fresh map's start
  is its best guess, so it is never called verified).
* `HomeBudget`: D3's return reserve. `T_home = K * L_home / v_obs + T_DOCK_S`,
  `L_home` the path length home on the brain's own map (`brain/frontier.py`'s
  `reach`, from the dock), `v_obs` this mission's own driving speed; the
  battery predicted at the dock from this mission's drain rate.
* `ReturnTrip`: D2/D4's phase, one action a tick: a nav2 goal to a staging
  point `STAGING_M` behind the dock along its heading, then a turn to face
  the dock, a creep on guarded verbs (every one through `robot/safety.py`),
  and a final turn to the dock's heading. Docked: the pose within
  `DOCK_TOL_M` and `DOCK_TOL_DEG`.
* `Charge`: 3.156.D's wait on the dock (D7d): rising to `resume_v`, or why not.

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
# 3.225: a motor's sag only ever lowers a reading, so the pack's own voltage
# over a window is the window's highest sample.
REST_WINDOW_S = 60.0
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
# 3.172 F2. nav2 driving less than STUCK_M in STUCK_S is a stall: cancel, back
# off (a refused back-off: explore's bounded creep forward), send the goal again.
# STUCK_ROUNDS shifts that leave it within STUCK_PLACE_M end the return at the next
# stall, 3.158.B's trap having waited out RETURN_MAX_S. One creep per STUCK_PLACE_M.
STUCK_S = 45.0
STUCK_M = 0.05
STUCK_PLACE_M = 0.30
STUCK_ROUNDS = 3
NUDGE_SPEED = 5             # 0.03 m/s: the creep bar
NUDGE_S = 3.3               # about 10 cm: brain/explore.py's NUDGE_S, and why
MAX_HOLDS = 20              # polls a move waits for the mission's own cancelled goal to let go
# 3.198 D2. A residual the creep's moves cannot close (under `drive: ros` a
# straight verb is whole 0.30 m moves) is closed by a nav2 goal aimed past the
# dock by how short of its last goal nav2 stopped, at most FINE_AIM_MAX_M; at
# most FINE_TRIES of them, then `not_aligned` with its numbers.
FINE_TRIES = 2
FINE_AIM_MAX_M = 0.20
# A creep that moved this much more than it asked shows a body whose moves come
# whole; on such a body a docked error over FINE_ABOVE_M is corrected too, so
# the dock is not passed by margin (3.195: 0.088-0.103 m, against 0.10).
QUANTUM_EXCESS_M = 0.05
FINE_ABOVE_M = 0.05
_NOT_OURS = ("StoppedByPerson", "AutonomyLapsed", "BatteryReserve", "Preempted", "MissionHalted")
# 3.156.D (D7d). `RESUME_V` is a placeholder until 3.137.E.
RESUME_V = 11.8
WALL_LIMIT_S = 4 * 3600.0   # the whole mission, charges and all
CHARGE_STALL_S = 300.0      # no rise of CHARGE_RISE_V in this long: no charger
CHARGE_RISE_V = 0.05
CHARGE_SAMPLE_S = 10.0      # a repeated reading this old is a new sample (3.137)
LEFT_DOCK_M = 0.25
RESUME_MIN_S = 120.0        # less of the limit left on arrival: not worth a charge
MAX_CHARGES = 3
CHARGE_POLL_S = 1.0
CHARGE_PROBE_S = 2.0        # D7f: how often a person's Stop or D-pad is looked for

DOCKED, FAILED, CANCELLED, SKIPPED = "docked", "failed", "cancelled", "skipped"


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def ahead(heading_deg: float) -> tuple:
    h = math.radians(heading_deg)
    return math.sin(h), -math.cos(h)


SAVED_MAP, START = "saved_map", "start"


def dock_from_pose(pose: Optional[dict], source: str = START) -> Optional[dict]:
    """The dock, from the pose at the mission's start: its `dock` (3.156.E)
    when it carries a readable one, else the pose itself as `source`."""
    if not pose or not pose.get("usable"):
        return None
    saved = pose.get("dock")
    if isinstance(saved, dict):
        try:
            d = {"x_m": float(saved["x_m"]), "y_m": float(saved["y_m"]),
                 "theta": float(saved["heading_deg"]), "source": SAVED_MAP}
            if all(math.isfinite(d[k]) for k in ("x_m", "y_m", "theta")):
                return d
        except (KeyError, TypeError, ValueError):
            pass                        # unreadable: the start pose, as before
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
        self._samples = []              # distinct (t, v, at_rest) battery samples
        self._path = None               # (L_home, x, y, t)
        self.last = {}
        self._prior_r = 0.0             # 3.156.D: the rate known before a charge

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

    def observe_battery(self, v: float, t: float, at_rest: Optional[bool] = None) -> None:
        """A distinct sample (3.137's rule: a repeated reading is the board's
        last one again, not a new measurement). `at_rest` (3.225): the robot
        server's word that no wheel turned for 0.5 s before it was read;
        None from a body that does not say. A repeat that arrives at rest is
        still dropped: it is the board's earlier measurement, which may have
        been taken while the wheels turned."""
        if self._samples and self._samples[-1][1] == v:
            return
        self._samples.append((t, float(v), at_rest))

    def v_obs(self) -> float:
        if self.driving_s < EARLY_DRIVE_S:
            return V_EARLY_M_S
        return max(V_FLOOR_M_S, self.metres / self.driving_s)

    @staticmethod
    def _highest(samples):
        """The highest sample, the earliest of equals: a sag only lowers a
        reading, so this is the nearest to the pack's own voltage."""
        return max(samples, key=lambda s: (s[1], -s[0]))

    def _ends(self, pool, now: float):
        """`a`, the highest of `pool`'s first REST_WINDOW_S, and `b`, the
        highest of its last REST_WINDOW_S before `now` (its latest, when none
        is that recent)."""
        t0 = pool[0][0]
        a = self._highest([s for s in pool if s[0] < t0 + REST_WINDOW_S])
        recent = [s for s in pool if s[0] > now - REST_WINDOW_S]
        return a, (self._highest(recent) if recent else pool[-1])

    def _pools(self, t: Optional[float]):
        """3.225: (pools to measure a drain within, `now` for each window).
        A body that reports `at_rest`: the readings at rest, and apart from
        them the readings while moving -- a drain is measured only within
        one kind (rest against rest, loaded against loaded), never across,
        where the sag would read as drain. A body that does not say: every
        reading, its windows ending at the last one (a repeat dropped as the
        same measurement is no new time)."""
        flagged = any(s[2] is not None for s in self._samples)
        if not flagged:
            return [self._samples], (self._samples[-1][0] if self._samples else None)
        now = max(t, self._samples[-1][0]) if t is not None else self._samples[-1][0]
        return ([[s for s in self._samples if s[2] is True],
                 [s for s in self._samples if s[2] is False]], now)

    def drain_v_s(self, t: Optional[float] = None) -> float:
        """V/s between two of one kind's readings (3.225), `a` and `b` of
        `_ends()`, at least DRAIN_SPAN_S apart; of the kinds that have one,
        the rate whose `b` is the more recent. Before any, the rate known
        before the last charge (none: 0). Raw first-to-last read a motor's
        sag as drain (3.140's two first surveys)."""
        if len(self._samples) < 2:
            return self._prior_r
        pools, now = self._pools(t)
        best = None
        for pool in pools:
            if len(pool) < 2:
                continue
            a, b = self._ends(pool, now)
            if b[0] - a[0] < DRAIN_SPAN_S:
                continue
            if best is None or b[0] > best[0]:
                best = (b[0], max(0.0, (a[1] - b[1]) / (b[0] - a[0])))
        return self._prior_r if best is None else best[1]

    def rested_v(self, t: Optional[float] = None) -> Optional[float]:
        """3.225: the pack's voltage at `t`. With readings at rest, the
        latest rested one carried forward by the drain; otherwise (none at
        rest yet, or a body that does not say) the window's highest reading,
        unprojected: projecting a sagged one would read its sag twice, and
        it errs low. None before any sample."""
        if not self._samples:
            return None
        pools, now = self._pools(t)
        rest = pools[0]
        if len(pools) == 2 and rest:
            _, b = self._ends(rest, now)
            return b[1] - self.drain_v_s(t) * max(0.0, now - b[0])
        pool = pools[0] if len(pools) == 1 else pools[1]
        return self._ends(pool, now)[1][1]

    def recharged(self) -> None:
        """3.156.D7g: the charge is not a drain; the rate stands until a new
        span exists. The drive so far and its speed are kept."""
        self._prior_r = self.drain_v_s()
        self._samples = []
        self._last = None
        self._path = None

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
        r = self.drain_v_s(t)
        self.last = {"l_home_m": round(L, 2), "t_home_s": round(th, 1),
                     "v_obs_m_s": round(self.v_obs(), 3), "drain_v_s": round(r, 5)}
        if left_s is not None and th > left_s:
            return f"time: {left_s:.0f} s left, the trip home {th:.0f} s ({L:.1f} m)"
        if v_now is not None and floor_v:
            rested = self.rested_v(t)
            v_now = rested if rested is not None else v_now     # 3.225: not a sagged reading
            at_dock = v_now - K * r * th
            self.last["v_at_dock"] = round(at_dock, 2)
            if at_dock < floor_v + BATTERY_MARGIN_V:
                return (f"battery: {v_now:.2f} V now, {at_dock:.2f} V predicted at the "
                        f"dock (floor {floor_v:g} V + {BATTERY_MARGIN_V:g})")
        return None


class Charge:
    """3.156.D7d: waiting on the dock. `observe()` returns None while
    charging, else (state, reason): `resumed`, `no_charge` or `left_dock`."""

    def __init__(self, dock: dict, resume_v: float, t0: float, stall_s: float = CHARGE_STALL_S):
        self.dock, self.resume_v, self.t0, self.stall_s = dock, resume_v, t0, stall_s
        self.from_v = None
        self.v = None
        self._at = None
        self._anchor_v = None
        self._rose_at = t0
        self.over = 0

    def observe(self, v: Optional[float], pose: Optional[dict], now: float) -> Optional[tuple]:
        if pose and pose.get("usable") and pose_error(self.dock, pose)[0] > LEFT_DOCK_M:
            return "left_dock", f"moved {pose_error(self.dock, pose)[0]:.2f} m from the dock"
        if v is not None and (v != self.v or now - self._at >= CHARGE_SAMPLE_S):
            self.v, self._at = v, now
            if self.from_v is None:
                self.from_v = self._anchor_v = v
            if v >= self._anchor_v + CHARGE_RISE_V:
                self._anchor_v, self._rose_at = v, now
            self.over = self.over + 1 if v >= self.resume_v else 0
            if self.over >= 2:
                return "resumed", f"{v:.2f} V, resume at {self.resume_v:g} V"
        if now - self._rose_at >= self.stall_s:
            return "no_charge", f"no rise of {CHARGE_RISE_V:g} V in {self.stall_s:.0f} s"
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
        self._moves = []                # 3.172: "back" or "nudge", one a step
        self._nudged = []               # where each creep was made from
        self._holds = 0                 # polls in a row the cancelled goal held a move
        self._wait_from = self.t0       # 3.172: when this wait for nav2 to take a goal began
        self._still = None              # (xy, t) the pose last moved from
        self._stuck_at = None           # where the current run of stalls began
        self.stuck_rounds = 0
        self.fine = False               # 3.198 D2: the current goal closes the last residual
        self.fine_tries = 0
        self._fine_refused = 0          # refusals of the current correcting goal on an unmoving clock
        self._fine_clock = None
        self._nav_stop_m = None         # how far from its goal nav2 ended the last one
        self._asked_m = None            # what the last creep asked for
        self._quantum_m = None          # a creep's whole move, once one moved more than asked

    def summary(self) -> dict:
        return {"phase": self.state, "actions": self.actions, "reversals": self.reversals,
                "metres": round(self.metres, 2), "elapsed_s": round(self.clock() - self.t0, 1),
                "goal_id": self.goal_id, "direct": self.direct, "fine_goals": self.fine_tries}

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
        if self._moves:
            if self._moves.pop(0) == "back":
                self._back_off()
            else:
                self._move("nudge", "FORWARD", speed=NUDGE_SPEED, duration=NUDGE_S)
            return None
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
            if type(e).__name__ in _NOT_OURS:
                raise
            reply = {"accepted": False, "reason": f"error: {e}"}
        if reply.get("accepted"):
            self.tries += 1
            self.goal_id = reply.get("goal_id")
            self.state, self.goal_sent_at = "driving", self.clock()
            # 3.172: this goal's own stall clock; a refusal before it says nothing of the turns after.
            self._still, self.refused, self._holds, self._wait_from = None, 0, 0, self.clock()
            self.log.append(f"goal to ({self.target[0]:.2f}, {self.target[1]:.2f}) accepted")
            return None
        if self.fine:
            # Counted only while the clock stands still: such a clock must not
            # hold the approach forever (a test's, or an idle that never waits).
            now = self.clock()
            self._fine_refused = self._fine_refused + 1 if now == self._fine_clock else 0
            self._fine_clock = now
        if (self.clock() - self._wait_from >= self.wait_s
                or self._fine_refused > self.wait_s / GOAL_RETRY_S):
            if self.fine:
                # The trip home ran: the judge decides on the pose as it is
                # (docked if within the tolerance), the correcting goals spent.
                self.log.append(f"correcting goal not taken in {self.wait_s:.0f} s "
                                f"({reply.get('reason')})")
                self.fine, self.fine_tries, self.state = False, FINE_TRIES, "final"
                return None
            return SKIPPED, f"no_nav2: goal not taken in {self.wait_s:.0f} s ({reply.get('reason')})"
        if self.idle is not None:
            self.idle(GOAL_RETRY_S)
        return None

    def _driving(self):
        pose = self._pose()             # metres home are counted on nav2's leg too (C4)
        goal = (self.nav.get_goal() or {}).get("goal") or {}
        st = goal.get("state")
        if st in ("pending", "active", None) and goal:
            if self._stalled(pose):
                if self.fine:
                    # No back-off beside the dock: the goal is given up, the judge decides.
                    try:
                        self.nav.cancel_goal()
                    except Exception as e:  # noqa: BLE001 -- the judge runs regardless
                        if type(e).__name__ in _NOT_OURS:
                            raise
                    self.log.append("correcting goal stalled")
                    self.fine, self.state = False, "final"
                    return None
                return self._unstick(pose)
            if self.idle is not None:
                self.idle(POLL_S)
            return None
        fine, self.fine = self.fine, False
        if st == "succeeded":
            end = self._pose()          # read after the goal's state: where nav2 ended it
            if end.get("usable") and not fine:      # a trip goal's: nav2's own stop error
                self._nav_stop_m = math.hypot(end["x_m"] - self.target[0], end["y_m"] - self.target[1])
            # 3.198: after the correcting goal, the dock's heading and the judge.
            self.state = "final" if fine else "turn"
            return None
        if fine:
            # Not a trip to recover (no back-off beside the dock): the judge decides.
            self.log.append(f"correcting goal {st or 'lost'}")
            self.state = "final"
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
        self._back_off()
        return None

    def _back_off(self) -> None:
        before = self.refused
        if not self._move("back", "REVERSE", speed=CREEP_SPEED, duration=BACK_OFF_S):
            return
        self.reversals += 1
        if NUDGE_S > 0 and self.refused > before and self._nudge_here():
            self._moves.append("nudge")

    def _move(self, name: str, action: str, **kw) -> bool:
        """False when the mission's own just-cancelled goal still holds the
        robot: that is a wait (`is_own_goal_refusal`, as explore's `_verb`), not a
        person's preemption and not a refusal; the move is queued again, for a while."""
        from brain.explore import is_own_goal_refusal
        from robot.interface import Preempted
        # After a verb the brain holds the robot until it lapses, so nav2's
        # next goal can come back `preempted`: that wait starts here (A6, live).
        self._wait_from = self.clock()
        try:
            self._act(action, **kw)
        except Preempted as e:
            if not is_own_goal_refusal(e):
                raise
            self.actions -= 1
            self._holds += 1
            if self._holds <= MAX_HOLDS:
                self._moves.insert(0, name)
                if self.idle is not None:
                    self.idle(POLL_S)
            return False
        self._holds = 0
        return True

    def _nudge_here(self) -> bool:
        """One creep per place: repeated, it would walk the robot on into
        the far end of the gap (brain/explore.py's `_nudge_here`)."""
        p = self._pose()
        if not p.get("usable"):
            return False
        xy = (p["x_m"], p["y_m"])
        if any(math.hypot(xy[0] - x, xy[1] - y) < STUCK_PLACE_M for x, y in self._nudged):
            return False
        self._nudged.append(xy)
        return True

    def _stalled(self, pose: dict) -> bool:
        """3.172 F2: nav2 has held a goal for STUCK_S with the robot not
        going anywhere. Translation only: a pivot that swings and is refused
        (3.158.B's trap) looks busy."""
        if STUCK_S <= 0 or not pose.get("usable"):
            self._still = None
            return False
        xy, now = (pose["x_m"], pose["y_m"]), self.clock()
        if self._still is None or math.hypot(xy[0] - self._still[0][0], xy[1] - self._still[0][1]) > STUCK_M:
            self._still = (xy, now)
            return False
        return now - self._still[1] >= STUCK_S

    def _unstick(self, pose: dict) -> Optional[tuple]:
        """Give up the goal and shift the robot, or end the return."""
        xy = (pose["x_m"], pose["y_m"])
        if (self._stuck_at is None
                or math.hypot(xy[0] - self._stuck_at[0], xy[1] - self._stuck_at[1]) > STUCK_PLACE_M):
            self._stuck_at, self.stuck_rounds = xy, 0
        self.stuck_rounds += 1
        if self.stuck_rounds > STUCK_ROUNDS:
            return FAILED, f"no_motion: still stalled at one place after {STUCK_ROUNDS} shifts"
        try:
            self.nav.cancel_goal()
        except Exception as e:  # noqa: BLE001 -- the back-off is refused while it holds; the next round retries
            if type(e).__name__ in _NOT_OURS:
                raise
        self.log.append(f"stalled {STUCK_S:.0f} s at ({xy[0]:.2f}, {xy[1]:.2f}); shifting and trying again")
        self.state, self.tries, self._still = "goal", 0, None
        self._back_off()
        return None

    def _to_dock(self, why: str) -> None:
        self.direct, self.tries, self.refused, self.fine = True, 0, 0, False
        self._wait_from = self.clock()  # the turns before it can outlast the wait
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
        """Thermos 3: a verb refused because the mission's own goal (just
        cancelled or ended) still holds the robot is a wait, as `_move`'s:
        at most MAX_HOLDS polls, not counted as an action."""
        from brain.explore import is_own_goal_refusal
        from robot.interface import Preempted
        try:
            out = self._approach_step()
        except Preempted as e:
            if not is_own_goal_refusal(e) or self._holds >= MAX_HOLDS:
                raise
            self.actions -= 1
            self._holds += 1
            if self.idle is not None:
                self.idle(POLL_S)
            return None
        self._holds = 0
        return out

    def _approach_step(self):
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
                moved = abs(self._last_along - along)
                self._step_m = moved or self._step_m
                if self._asked_m is not None and moved > self._asked_m + QUANTUM_EXCESS_M:
                    self._quantum_m = moved             # 3.198: moves come whole
            self._asked_m = None
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
            before = self.refused
            if along > 0:
                self._act("FORWARD", speed=CREEP_SPEED, duration=dur)
            else:
                self.reversals += 1
                self._act("REVERSE", speed=CREEP_SPEED, duration=dur)
            # Judged at the next creep only, and only for a creep that ran.
            self._asked_m = v * dur if self.refused == before else None
            return None
        # final: the dock's heading, then judge.
        if self._turn_to(pose, self.dock["theta"]):
            return None
        m, deg = pose_error(self.dock, pose)
        # Whole moves seen, and the residual under half the last one (a move
        # cut short or doubled is still the size the next one may take).
        whole = bool(self._quantum_m) and bool(self._step_m) and dist < 0.5 * self._step_m
        if whole and m > FINE_ABOVE_M and self.fine_tries < FINE_TRIES:
            # 3.198 D2: under whole moves the creep cannot close this; nav2 can.
            return self._fine(pose, dist, m, deg)
        if m <= DOCK_TOL_M and deg <= DOCK_TOL_DEG:
            return DOCKED, f"{m:.3f} m, {deg:.1f} deg from the dock"
        if dist > DOCK_TOL_M:
            if self._step_m and dist < 0.5 * self._step_m:
                # 3.198 D1: the creep's guard would skip the move (it would
                # overshoot by more than it closes), so turning back to the
                # dock only pivots at one place: c-1's 40 verbs.
                if whole:
                    return self._fine(pose, dist, m, deg)   # D2, and once spent, the end
                # Moves that come as asked: the step was the last creep's
                # size, not the body's; forget it and creep what is left.
                self._step_m = None
            self.state = "turn"
            return None
        return FAILED, f"not_aligned: {m:.3f} m, {deg:.1f} deg"

    def _error(self) -> str:
        p = self.get_pose() or {}
        if not p.get("usable"):
            return "no pose"
        m, deg = pose_error(self.dock, p)
        return f"{m:.3f} m, {deg:.1f} deg"

    def _fine(self, pose: dict, dist: float, m: float, deg: float) -> Optional[tuple]:
        """3.198 D2: one nav2 goal past the dock, on the line from the robot
        through it, by how short of its last goal nav2 stopped: its goal
        checker then ends the goal on the dock. A nav2 goal like the trip's
        own (arbitrated, refused while the brain's last verb holds the robot,
        retried), never a motion of its own."""
        if self.fine_tries >= FINE_TRIES:
            return FAILED, f"not_aligned: {m:.3f} m, {deg:.1f} deg"
        self.fine_tries += 1
        aim = min(max(self._nav_stop_m or 0.0, 0.0), FINE_AIM_MAX_M)
        ux, uy = (self.dock["x_m"] - pose["x_m"]) / dist, (self.dock["y_m"] - pose["y_m"]) / dist
        self.target = (self.dock["x_m"] + aim * ux, self.dock["y_m"] + aim * uy)
        self.fine, self.tries, self.state, self._fine_refused = True, 0, "goal", 0
        self._wait_from, self._still, self._last_along = self.clock(), None, None
        self.log.append(f"{m:.3f} m off, under one move: a goal {aim:.3f} m past the dock "
                        f"({self.fine_tries} of {FINE_TRIES})")
        return None
