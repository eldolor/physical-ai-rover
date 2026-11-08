# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""A simulated pack, as a motor board measures it -- 3.70 E4, 3.137.

A straight-line drain in time (not a discharge curve: enough to cross the
ladder's levels), plus the two things that make a real board's reading
harder to trust than the true voltage:

* **a refresh period** -- the Rover's board measures its battery every 10 s
  (`oled_ctrl.h` `inaDataUpdate()`) and repeats that value in every frame
  between (3.76 M2.7). 0 measures on every read, as the fake board always did;
* **a sag under motor start** -- for `sag_s` after the wheels start turning,
  while they turn, the reading is `sag_v` lower (the motors' inrush). Wheels
  at rest never sag, so a reading taken at rest is the pack's own.

3.156.D's charge stand-in: `charging(True)` and the pack rises at
`charge_v_per_s` up to `FULL_V`, instead of draining (`MockRobot` calls it, at
rest on its pad). A pack never charged reads exactly as before.

Used by `sim/fake_esp32.py` (always: 12.00 V and no drain unless told) and,
opt-in, by `sim/mock_robot.py` (only when `SIM_BATTERY_V` is set), so the
plain simulator can run the battery ladder in process.
"""

import os
from typing import Callable, Optional


class SimBattery:
    FULL_V = 12.6

    def __init__(self, v0: float = 12.0, drain_v_per_s: float = 0.0,
                 refresh_s: float = 0.0, sag_v: float = 0.0, sag_s: float = 1.0,
                 clock: Optional[Callable[[], float]] = None, charge_v_per_s: float = 0.0):
        import time
        self.clock = clock or time.monotonic
        self.v0 = float(v0)
        self.drain_v_per_s = float(drain_v_per_s)
        self.refresh_s = float(refresh_s)
        self.sag_v = float(sag_v)
        self.sag_s = float(sag_s)
        self.since = self.clock()
        self._driving_since: Optional[float] = None
        self._held: Optional[float] = None
        self._held_at: Optional[float] = None
        self.sagged_reads = 0           # readings measured inside a sag (for tests)
        self.charge_v_per_s = float(charge_v_per_s)
        self.is_charging = False
        self._anchor: Optional[tuple] = None    # (t, v) of the last change of charging

    @classmethod
    def from_env(cls, clock=None, v0: Optional[float] = None,
                 drain_v_per_s: Optional[float] = None) -> "SimBattery":
        """The pack a server run asks for: `SIM_BATTERY_V` (12.0),
        `SIM_BATTERY_DRAIN_V_PER_S` (0), `SIM_BATTERY_REFRESH_S` (0),
        `SIM_BATTERY_SAG_V` (0), `SIM_BATTERY_SAG_S` (1.0) and
        `SIM_BATTERY_CHARGE_V_PER_S` (0). Arguments given win over the
        environment."""
        def env(name, default):
            raw = os.environ.get(name)
            return float(raw) if raw else default
        return cls(v0=v0 if v0 is not None else env("SIM_BATTERY_V", 12.0),
                   drain_v_per_s=(drain_v_per_s if drain_v_per_s is not None
                                  else env("SIM_BATTERY_DRAIN_V_PER_S", 0.0)),
                   refresh_s=env("SIM_BATTERY_REFRESH_S", 0.0),
                   sag_v=env("SIM_BATTERY_SAG_V", 0.0),
                   sag_s=env("SIM_BATTERY_SAG_S", 1.0),
                   clock=clock,
                   charge_v_per_s=env("SIM_BATTERY_CHARGE_V_PER_S", 0.0))

    def true_v(self, now: Optional[float] = None) -> float:
        now = self.clock() if now is None else now
        if self._anchor is None:
            return max(0.0, self.v0 - self.drain_v_per_s * max(0.0, now - self.since))
        t, v = self._anchor
        if self.is_charging:
            return min(max(self.FULL_V, v), v + self.charge_v_per_s * max(0.0, now - t))
        return max(0.0, v - self.drain_v_per_s * max(0.0, now - t))

    @property
    def is_driving(self) -> bool:
        return self._driving_since is not None

    def charging(self, on: bool, now: Optional[float] = None) -> None:
        """3.156.D: on or off the charger (no rate, no charger)."""
        now = self.clock() if now is None else now
        on = bool(on and self.charge_v_per_s > 0)
        if on != self.is_charging:
            self._anchor = (now, self.true_v(now))
            self.is_charging = on

    def driving(self, on: bool, now: Optional[float] = None) -> None:
        """The wheels started or stopped turning. A start opens a sag."""
        now = self.clock() if now is None else now
        if on and self._driving_since is None:
            self._driving_since = now
        elif not on:
            self._driving_since = None

    def _measure(self, now: float) -> float:
        v = self.true_v(now)
        if (self.sag_v and self._driving_since is not None
                and now - self._driving_since < self.sag_s):
            self.sagged_reads += 1
            v -= self.sag_v
        return max(0.0, v)

    def read(self, now: Optional[float] = None) -> float:
        """What the board reports now: a fresh measurement, or the last one
        repeated until its refresh is due."""
        now = self.clock() if now is None else now
        if (self.refresh_s <= 0 or self._held is None
                or now - self._held_at >= self.refresh_s):
            self._held, self._held_at = self._measure(now), now
        return self._held
