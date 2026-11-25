# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
robot/board_http.py

`PLAN-ros-alignment.md` 3.217 -- the motor board's commands over its web
route (`GET /js?json=<command>`), for a board whose serial receive is dead.
A stopgap behind `ROBOT_COMMAND_URL`; serial stays the default and the
shipped route. Feedback still arrives over serial: this is the
host-to-board direction only.

`robot/hardware_robot.py` hands every command to `submit()`, which never
waits on the network: one thread sends them, one at a time, in order.

* **Wheel commands (`T:1`) are one slot, newest wins.** A stop is never
  queued behind older speeds, and an older speed is never sent after it.
* **Set-up commands keep their order and go first.**
* **A request that does not answer in `COMMAND_TIMEOUT_S` is abandoned**,
  its connection closed, and the newest wheel command sent on a new one.
  The board may still act on the abandoned request late, so after any
  failure the newest wheel command is sent `REASSERT_SENDS` more times; a
  stale speed that still lands last runs until the board's heartbeat.
* **No delivery, no motion** (3.34's rule, in the other direction): with a
  command undelivered for `COMMAND_STALE_S`, `submit()` refuses a non-zero
  wheel command and queues a zero in its place. A zero is never refused.
"""

import collections
import http.client
import json
import threading
import time
import urllib.parse
from typing import Optional

# One request's patience. Measured on the Rover's board from the Jetson,
# 2026-11-25: p50 24 ms, p99 73-116 ms; 6 of 2,600 never answered in 1 s.
# 0.10 s leaves one retry inside the board's 200 ms heartbeat.
COMMAND_TIMEOUT_S = 0.10
# A command undelivered this long and the route is taken as down: the same
# five 50 ms periods as hardware_robot.FEEDBACK_STALE_S.
COMMAND_STALE_S = 0.25
# After a failure, how many more times the newest wheel command is sent.
REASSERT_SENDS = 3
REASSERT_PERIOD_S = 0.05
# After a failure, before the next try: a refused connection fails at once,
# and a bare loop on it would spin a core beside the wheel loop.
RETRY_PAUSE_S = 0.02
SETUP_QUEUE_MAX = 32
ZERO = {"T": 1, "L": 0.0, "R": 0.0}


def _is_wheel(obj: dict) -> bool:
    return obj.get("T") == 1


def _is_zero(obj: dict) -> bool:
    return not obj.get("L") and not obj.get("R")


class HttpCommandSender:
    def __init__(self, base_url: str, timeout_s: float = COMMAND_TIMEOUT_S):
        url = urllib.parse.urlsplit(base_url)
        if (url.scheme != "http" or not url.hostname or url.username or url.password
                or url.path not in ("", "/") or url.query or url.fragment):
            raise ValueError("the board's command URL must be http://<host>[:<port>] "
                             f"and nothing else, got {base_url!r}")
        self.host, self.port = url.hostname, url.port or 80
        self.timeout_s = float(timeout_s)
        self._cond = threading.Condition()
        # Each pending item is (when it first waited, command).
        self._setup = collections.deque()
        self._wheel = None
        self._inflight_at: Optional[float] = None
        self._last_wheel: Optional[dict] = None     # the newest one issued
        self._reassert = 0
        self.sent = 0
        self.failed = 0
        self.coalesced = 0          # wheel commands replaced before they were sent
        self.refused = 0            # non-zero wheel commands refused on a stale route
        self.dropped = 0            # set-up commands lost to a full queue or a bad status
        self.last_error: Optional[str] = None
        self._conn: Optional[http.client.HTTPConnection] = None
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---------- the caller's side: never waits on the network ----------

    def _age_locked(self, now: float) -> Optional[float]:
        waits = [t for t in (self._inflight_at,
                             self._setup[0][0] if self._setup else None,
                             self._wheel[0] if self._wheel else None) if t is not None]
        return now - min(waits) if waits else None

    def stale(self) -> bool:
        with self._cond:
            age = self._age_locked(time.monotonic())
        return age is not None and age > COMMAND_STALE_S

    def submit(self, obj: dict) -> bool:
        """Queue one command. False if it was a non-zero wheel command and
        the route is stale: a zero was queued in its place."""
        now = time.monotonic()
        with self._cond:
            ok = True
            if _is_wheel(obj):
                age = self._age_locked(now)
                if age is not None and age > COMMAND_STALE_S and not _is_zero(obj):
                    obj, ok = dict(ZERO), False
                    self.refused += 1
                since = now
                if self._wheel is not None:
                    since = self._wheel[0]
                    self.coalesced += 1
                self._wheel = (since, obj)
                self._last_wheel = obj
            else:
                if len(self._setup) >= SETUP_QUEUE_MAX:
                    self._setup.popleft()
                    self.dropped += 1
                self._setup.append((now, obj))
            self._cond.notify_all()
        return ok

    def flush(self, timeout_s: float) -> bool:
        """Wait until nothing is pending, or `timeout_s`."""
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._age_locked(time.monotonic()) is not None:
                left = deadline - time.monotonic()
                if left <= 0:
                    return False
                self._cond.wait(left)
        return True

    def status(self) -> dict:
        with self._cond:
            age = self._age_locked(time.monotonic())
            return {"url": f"http://{self.host}:{self.port}",
                    "undelivered_age_s": None if age is None else round(age, 3),
                    "stale_after_s": COMMAND_STALE_S, "timeout_s": self.timeout_s,
                    "sent": self.sent, "failed": self.failed,
                    "coalesced": self.coalesced, "refused": self.refused,
                    "dropped": self.dropped, "last_error": self.last_error}

    def close(self) -> None:
        with self._cond:
            self._running = False
            self._cond.notify_all()
        self._thread.join(timeout=1.0)
        self._drop_connection()

    # ---------- the sender thread ----------

    def _take(self):
        """The next command and its kind, or None when closing."""
        with self._cond:
            while self._running:
                if self._setup:
                    since, obj = self._setup.popleft()
                    self._inflight_at = since
                    return "setup", obj
                if self._wheel is not None:
                    (since, obj), self._wheel = self._wheel, None
                    self._inflight_at = since
                    return "wheel", obj
                if self._reassert and self._last_wheel is not None:
                    if not self._cond.wait(REASSERT_PERIOD_S):
                        # Nothing newer in a period: say the newest again.
                        self._reassert -= 1
                        self._inflight_at = time.monotonic()
                        return "again", self._last_wheel
                    continue
                self._cond.wait(0.5)
        return None

    def _drop_connection(self) -> None:
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except OSError:
                pass

    def _request(self, obj: dict) -> int:
        if self._conn is None:
            self._conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout_s)
        path = "/js?json=" + urllib.parse.quote(json.dumps(obj, separators=(",", ":")))
        self._conn.request("GET", path)
        reply = self._conn.getresponse()
        reply.read(65536)
        if not reply.isclosed():
            self._drop_connection()     # a body longer than we read: do not reuse
        return reply.status

    def _run(self) -> None:
        while True:
            taken = self._take()
            if taken is None:
                return
            kind, obj = taken
            heard = False               # the board answered, whatever it said
            try:
                status = self._request(obj)
                heard = True
                error = None if status == 200 else f"the board answered HTTP {status}"
            except (OSError, http.client.HTTPException) as e:
                self._drop_connection()
                error = f"{type(e).__name__}: {e}"[:200]
            with self._cond:
                since, self._inflight_at = self._inflight_at, None
                if error is None:
                    self.sent += 1
                else:
                    self.failed += 1
                    self.last_error = error
                    self._reassert = REASSERT_SENDS
                    if kind == "setup" and heard:
                        self.dropped += 1       # refused by the board: never block the wheels on it
                    elif kind == "setup":
                        self._setup.appendleft((since, obj))
                    elif self._wheel is None:
                        # Nothing newer was issued: this is still the newest.
                        self._wheel = (since, obj)
                    else:
                        self._wheel = (min(since, self._wheel[0]), self._wheel[1])
                self._cond.notify_all()
            if error is not None and self._running:
                time.sleep(RETRY_PAUSE_S)
