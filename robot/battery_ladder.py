"""The battery ladder's decisions -- PLAN-ros-alignment.md 3.137.

Pure logic, no I/O: `robot/server.py` feeds it each battery read and acts on
what it answers. Kept apart so every rule below is tested on planted samples,
not only through a draining battery.

Three levels on the pack's volts (the board's `v`), placeholders until 3.137.E
measures the Rover's discharge curve:

* **Low** (10.5 V): reported only. The brain's own floor (`battery_floor_v`,
  3.70 E4) ends a mission there, and a mission may not start below it.
* **Reserve** (10.2 V): latched. The robot server refuses all autonomy; a
  person may still drive at reduced speed, to bring the robot to its charger.
* **Power-off** (10.0 V, or Reserve held for `reserve_max_s`): latched. A
  clean power-off nobody may override.

Each level is crossed only on **two distinct board samples** under it (3.76
M2.7's rule, extended): a read counts again only if its value changed since
the last one counted, or the same value is still there after `sample_s` (the
Rover's board measures its battery every 10 s and repeats it in every frame).

**At Reserve and Power-off the wheels must be at rest** (the plan's ladder):
a motor's inrush sags the pack, so if either sample was read while the wheels
turned, the answer is CHECK -- the server stops the wheels and holds autonomy
-- and the next distinct sample decides: at or over the level (at rest or
not: a sag only lowers a reading) it was a sag and the check clears; under it
and read at rest, it fires. A check that sees no sample at rest within
`check_max_s` (a person driving on) fires on the moving samples if the last
two are still under: the safe side.

Known limit: a repeated value counted after `sample_s` may have been measured
by the board before the wheels stopped (its refresh instant is not reported).
"""

from typing import List, Optional, Tuple

OK = "ok"
LOW = "low"
RESERVE = "reserve"
POWEROFF = "poweroff"
LEVELS = (OK, LOW, RESERVE, POWEROFF)

# What `observe()` / `tick()` answer, beside a level reached.
CHECK = "check"         # stop the wheels and read again at rest
CLEAR = "clear"         # the check was a sag: autonomy may drive again

# Two distinct samples (M2.7, the user's "two distinct samples").
READINGS = 2
# The Rover's board refreshes its battery every 10 s, plus slack (M2.7).
SAMPLE_S = 12.0


class BatteryLadder:
    def __init__(self, low_v: float = 10.5, reserve_v: float = 10.2,
                 poweroff_v: float = 10.0, reserve_max_s: float = 120.0,
                 sample_s: float = SAMPLE_S, check_max_s: Optional[float] = None):
        if not (low_v > reserve_v > poweroff_v > 0):
            raise ValueError("battery levels must fall: low > reserve > power-off > 0 "
                             f"(got {low_v}, {reserve_v}, {poweroff_v})")
        if reserve_max_s <= 0:
            raise ValueError("reserve_max_s must be > 0")
        self.low_v = float(low_v)
        self.reserve_v = float(reserve_v)
        self.poweroff_v = float(poweroff_v)
        self.reserve_max_s = float(reserve_max_s)
        self.sample_s = float(sample_s)
        # Long enough for one more board refresh after the wheels stop.
        self.check_max_s = float(check_max_s if check_max_s is not None else sample_s + 3.0)
        self.level = OK
        self.reserve_at: Optional[float] = None
        self.poweroff_at: Optional[float] = None
        self.poweroff_why: Optional[str] = None
        # The distinct samples, newest last: (volts, read at rest, when).
        self.samples: List[Tuple[float, bool, float]] = []
        self.last_v: Optional[float] = None
        # A check in progress: the level it is for and when it began.
        self.check_level: Optional[str] = None
        self.check_at: Optional[float] = None
        self.checks = 0                 # how many began (a sag is one that cleared)
        self.sags = 0

    # ---------- reads ----------

    def _distinct(self, v: float, now: float) -> bool:
        if not self.samples:
            return True
        last_v, _, last_at = self.samples[-1]
        return v != last_v or now - last_at >= self.sample_s

    def _target(self) -> Optional[str]:
        """The highest latched level the last READINGS samples are all under,
        if it is above the current level."""
        if len(self.samples) < READINGS:
            return None
        recent = [s[0] for s in self.samples[-READINGS:]]
        for level, bar in ((POWEROFF, self.poweroff_v), (RESERVE, self.reserve_v)):
            if all(v < bar for v in recent) and LEVELS.index(level) > LEVELS.index(self.level):
                return level
        return None

    def _under(self, v: float) -> Optional[str]:
        for level, bar in ((POWEROFF, self.poweroff_v), (RESERVE, self.reserve_v)):
            if v < bar and LEVELS.index(level) > LEVELS.index(self.level):
                return level
        return None

    def observe(self, v: Optional[float], now: float, at_rest: bool) -> Optional[str]:
        """One battery read: `v` volts (None: no reading, which never moves a
        level), read at `now`, with the wheels at rest or not. Answers the
        latched level newly reached (RESERVE or POWEROFF), CHECK, CLEAR or
        None. Low moves silently: read it from `level`."""
        if v is None:
            return self.tick(now)
        v = float(v)
        self.last_v = v
        if self.level == POWEROFF:
            return None
        if not self._distinct(v, now):
            return self.tick(now)
        self.samples.append((v, bool(at_rest), now))
        del self.samples[:-READINGS]

        # Low: reported, never latched (a charger plugged in clears it).
        if self.level in (OK, LOW):
            recent = [s[0] for s in self.samples]
            if len(recent) >= READINGS and all(x < self.low_v for x in recent):
                self.level = LOW
            elif v >= self.low_v:
                self.level = OK

        if self.check_level is not None:
            # The highest latched level above the current one that this
            # reading is under (a check for Power-off read again at 10.1 V is
            # still a Reserve).
            level = self._under(v)
            if level is None:
                # Back over every line, at rest or not: a sag only lowers a
                # reading, so this one is the pack's own.
                self.check_level = self.check_at = None
                self.sags += 1
                return CLEAR
            if at_rest:
                # Fire what the last two distinct samples both support: one
                # glitch read at rest under Power-off is not two (review).
                target = self._target()
                if target is None:
                    self.check_level = level
                    return self.tick(now)
                self.check_level = self.check_at = None
                return self._fire(target, now, "at rest")
            self.check_level = level
            return self.tick(now)

        target = self._target()
        if target is None:
            return self.tick(now)
        if all(s[1] for s in self.samples[-READINGS:]):
            return self._fire(target, now, "at rest")
        self.check_level, self.check_at = target, now
        self.checks += 1
        return CHECK

    def tick(self, now: float) -> Optional[str]:
        """Timers only: a check that never saw a sample at rest, and Reserve
        held for `reserve_max_s`."""
        if self.level == POWEROFF:
            return None
        if self.check_level is not None and now - self.check_at >= self.check_max_s:
            self.check_level = self.check_at = None
            target = self._target()
            if target is None:          # the newest samples are no longer all under
                self.sags += 1
                return CLEAR
            return self._fire(target, now, "no reading at rest")
        if (self.level == RESERVE and self.reserve_at is not None
                and now - self.reserve_at >= self.reserve_max_s):
            return self._fire(POWEROFF, now, f"reserve held {self.reserve_max_s:g} s")
        return None

    def _fire(self, level: str, now: float, how: str) -> str:
        if level == RESERVE:
            self.level, self.reserve_at = RESERVE, now
        else:
            if self.reserve_at is None:
                self.reserve_at = now
            self.level, self.poweroff_at = POWEROFF, now
            self.poweroff_why = how
        return level

    # ---------- reporting ----------

    def status(self, now: float) -> dict:
        poweroff_in = None
        if self.level == RESERVE and self.reserve_at is not None:
            poweroff_in = round(max(0.0, self.reserve_max_s - (now - self.reserve_at)), 1)
        return {
            "level": self.level,
            "levels_v": {"low": self.low_v, "reserve": self.reserve_v,
                         "poweroff": self.poweroff_v},
            "checking": self.check_level,
            "reserve_s": (round(now - self.reserve_at, 1)
                          if self.reserve_at is not None else None),
            # The countdown the twin shows under Reserve (3.137.B).
            "poweroff_in_s": poweroff_in,
            "poweroff_why": self.poweroff_why,
            # Distinct samples held (0-2): a level needs two.
            "samples": len(self.samples),
            "checks": self.checks,
            "sags": self.sags,
        }
