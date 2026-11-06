# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
remote_navigator.py -- nav2's goals, over HTTP (PLAN-ros-alignment.md 3.31).

The `Navigator` that `brain/explore.py` declares, implemented over the robot
server's existing `/world/goal` routes. `RemoteWorld`'s sibling and the same
shape, but deliberately not part of it: `WorldInterface` is asked what is
true about the house and is never asked to drive, and a goal is a drive
command (`tests/test_world_contract.py`).

Like `RemoteWorld`, it reads JSON and contains no hint that ROS exists. A
server whose world cannot take goals answers 501; that is reported as
`unsupported`, never as an accepted goal, and `available()` lets the brain
refuse an `explore` mission at start rather than on its first tick.
"""

import logging
import threading
import uuid
from typing import Optional

import httpx

from control.remote_robot import DEFAULT_DRIVER
from world.bridge_faults import fault_from_headers, response_detail, setup_fault_message

DEFAULT_TIMEOUT_S = 10.0
# 3.145: a goal this navigator sends lives on the robot server only while it
# beats (`POST /autonomy/heartbeat`): its lease lapses `watchdog_timeout_s`
# (1.0 s) after the last beat. Four beats a lease, on a thread of their own,
# so a long tick (a VLM call) never starves them -- only a dead or frozen
# process does. Each beat's own timeout stays under the beat period.
HEARTBEAT_S = 0.25
HEARTBEAT_TIMEOUT_S = 0.2

logger = logging.getLogger(__name__)


class RemoteNavigator:
    """HTTP client of `/world/goal` (POST to send, GET to read, DELETE to
    cancel), in the house frame."""

    def __init__(self, base_url: str, secret: Optional[str] = None,
                 timeout: float = DEFAULT_TIMEOUT_S,
                 client: Optional[httpx.Client] = None, driver: str = DEFAULT_DRIVER,
                 heartbeat_s: Optional[float] = HEARTBEAT_S):
        self.base_url = base_url.rstrip("/")
        self.heartbeat_s = heartbeat_s
        self._beats_stop = threading.Event()
        self._beats: Optional[threading.Thread] = None
        self._beats_lock = threading.Lock()
        # 3.84: explore's goals name the brain, so the robot server can tell
        # them from a script's and refuse them after a person's Stop.
        # 3.145: and name this navigator, whose beats lease its goals -- a
        # restarted brain beating as `brain` never keeps a dead one's lease.
        self.headers = {"x-driver": driver, "x-lease-id": uuid.uuid4().hex,
                        **({"x-app-secret": secret} if secret else {})}
        # An injected client is how tests mount the app in-process.
        self._owns_client = client is None
        self._client = client or httpx.Client(base_url=self.base_url, timeout=timeout)
        # A beat's own short timeout, on a client this navigator owns (an
        # injected test client takes none).
        self._beat_kw = {"timeout": HEARTBEAT_TIMEOUT_S} if self._owns_client else {}

    def _call(self, method: str, json: Optional[dict] = None) -> httpx.Response:
        return self._client.request(method, "/world/goal", json=json, headers=self.headers)

    def available(self) -> bool:
        """Can this server take goals at all? (A world with nav2 behind it.)"""
        r = self._call("GET")
        if r.status_code in (401, 403):
            # 3.76: a wrong secret read as "this world has no nav2".
            raise PermissionError(f"the robot server refused the brain's secret "
                                  f"(HTTP {r.status_code}) -- check APP_SHARED_SECRET")
        fault = fault_from_headers(r.headers) if r.status_code == 503 else None
        if fault is not None and not fault["retryable"]:
            # 3.82: the robot server's bridge refused it for a fault no wait
            # mends (a 401: the two secrets differ), which read as "no nav2".
            raise PermissionError(setup_fault_message("the goal route", fault, response_detail(r)))
        return r.status_code == 200

    def _beat(self) -> bool:
        try:
            r = self._client.request("POST", "/autonomy/heartbeat", headers=self.headers,
                                     **self._beat_kw)
            return r.status_code == 200
        except httpx.HTTPError:
            return False

    def _beat_loop(self) -> None:
        missed = 0
        while not self._beats_stop.wait(self.heartbeat_s):
            if self._beat():
                missed = 0
                continue
            missed += 1
            if missed == 1:
                logger.warning("goal lease heartbeat not answered (retrying every %ss)",
                               self.heartbeat_s)

    def start_heartbeat(self) -> None:
        """3.145: beat once now (the robot server arms a goal's lease only
        for a driver it has heard within it), then every `heartbeat_s` until
        close(). Idempotent; a navigator with `heartbeat_s=None` never beats
        (its goals are not leased, as before 3.145)."""
        with self._beats_lock:
            if self.heartbeat_s is None or self._beats is not None or self._beats_stop.is_set():
                return
            for _ in range(3):                   # a goal sent unbeaten is not leased
                if self._beat():
                    break
            self._beats = threading.Thread(target=self._beat_loop, daemon=True,
                                           name="goal-lease-heartbeat")
            self._beats.start()

    def stop_heartbeat(self) -> None:
        """Stop beating and release a lease this driver holds: a goal the
        mission ended is not a lapse."""
        with self._beats_lock:
            self._beats_stop.set()
            beats, self._beats = self._beats, None
        if beats is None:
            return
        beats.join(timeout=1.0)
        try:
            self._client.request("DELETE", "/autonomy/heartbeat", headers=self.headers,
                                 **self._beat_kw)
        except httpx.HTTPError:
            pass        # the lease then lapses; its goal was the mission's to end

    def set_goal(self, x_m: float, y_m: float) -> dict:
        self.start_heartbeat()
        r = self._call("POST", {"x_m": x_m, "y_m": y_m})
        if r.status_code == 501:
            return {"accepted": False, "reason": "unsupported"}
        r.raise_for_status()
        reply = r.json()
        if self.heartbeat_s is not None and reply.get("accepted") and reply.get("leased") is False:
            # 3.145: the robot server heard no beat in time (or predates the
            # lease): this goal would outlive a dead brain. Said, not hidden.
            logger.warning("nav2 goal (%s, %s) accepted WITHOUT a lease: the robot server "
                           "heard no heartbeat from this brain", x_m, y_m)
        return reply

    def get_goal(self) -> dict:
        r = self._call("GET")
        if r.status_code == 501:
            return {"goal": None, "plan": []}
        r.raise_for_status()
        return r.json()

    def cancel_goal(self) -> dict:
        r = self._call("DELETE")
        if r.status_code == 501:
            return {"canceled": False, "reason": "unsupported"}
        r.raise_for_status()
        return r.json()

    def clear_keepout(self) -> dict:
        """3.76 K5.3: the mission ended; its keepout marks go with it. A
        server that predates the route (404/405) has none to clear."""
        r = self._client.request("DELETE", "/world/keepout", headers=self.headers)
        if r.status_code in (404, 405, 501):
            return {"cleared": 0, "reason": "unsupported"}
        r.raise_for_status()
        return r.json()

    def close(self) -> None:
        """One client per explore mission: closed at its end (3.76), or a
        long-lived brain gathers sockets. An injected client is the
        caller's to close."""
        self.stop_heartbeat()
        if self._owns_client:
            self._client.close()
