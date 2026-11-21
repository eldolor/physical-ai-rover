# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/sim-chaos-3202/fault_proxy.py

S7's fault-injection proxy (3.202 K6-K8): an HTTP/1.1 reverse proxy put
between the brain's `RemoteRobot` (its `ROBOT_URL`) and the robot server, so
the link can be broken without touching either process.

    python fault_proxy.py <listen_port> <upstream_url>

Its faults are set at run time on the proxy's own control route, never
forwarded:

    POST /__fault {"latency_ms": 200, "jitter_ms": 50, "drop": 0.05,
                   "dead": false, "dup_action": false, "seed": 1}
    GET  /__fault  -> the current faults and the counters

* `latency_ms` +/- `jitter_ms` (uniform) is slept before the request goes
  upstream AND before the answer goes back: "added each way".
* `drop` is the share of requests whose connection is reset (SO_LINGER 0,
  an RST) instead of forwarded -- the request never reaches the robot.
* `dead` resets every connection: the link is gone (K6).
* `dup_action`: every `POST /action` is sent upstream twice, the copy 50 ms
  after the original on its own connection; the brain sees the original's
  answer (K8). The copy's status is counted (`dup_status`).

Each request is one line in `<FAULT_PROXY_LOG>` when that is set.
"""

from __future__ import annotations

import http.client
import json
import os
import random
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

HOP = {"connection", "keep-alive", "proxy-connection", "transfer-encoding", "te", "trailer",
       "upgrade", "content-length", "host"}
DUP_DELAY_S = 0.05
UPSTREAM_TIMEOUT_S = 60.0


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.faults = {"latency_ms": 0.0, "jitter_ms": 0.0, "drop": 0.0, "dead": False,
                       "dup_action": False}
        self.rng = random.Random(0)
        self.counts = {"forwarded": 0, "dropped": 0, "dead": 0, "dup_sent": 0,
                       "upstream_error": 0}
        self.dup_status: dict = {}

    def set(self, body: dict) -> dict:
        with self.lock:
            for k in self.faults:
                if k in body:
                    self.faults[k] = type(self.faults[k])(body[k])
            if "seed" in body:
                self.rng = random.Random(body["seed"])
            return dict(self.faults)

    def count(self, key: str, n: int = 1) -> None:
        with self.lock:
            self.counts[key] = self.counts.get(key, 0) + n

    def snapshot(self) -> dict:
        with self.lock:
            return {"faults": dict(self.faults), "counts": dict(self.counts),
                    "dup_status": dict(self.dup_status)}

    def delay(self) -> float:
        with self.lock:
            lat, jit = self.faults["latency_ms"], self.faults["jitter_ms"]
            return max(0.0, lat + (self.rng.uniform(-jit, jit) if jit else 0.0)) / 1000.0

    def roll_drop(self) -> bool:
        with self.lock:
            return self.faults["drop"] > 0 and self.rng.random() < self.faults["drop"]


def make_handler(state: State, upstream: str, log):
    up = urlsplit(upstream)
    host, port = up.hostname, up.port or 80
    log_lock = threading.Lock()

    def note(rec: dict) -> None:
        if log is None:
            return
        with log_lock:
            log.write(json.dumps(rec) + "\n")

    def forward(method: str, path: str, headers: dict, body: bytes):
        conn = http.client.HTTPConnection(host, port, timeout=UPSTREAM_TIMEOUT_S)
        try:
            conn.request(method, path, body=body or None, headers=headers)
            r = conn.getresponse()
            return r.status, r.getheaders(), r.read()
        finally:
            conn.close()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _reset(self) -> None:
            """An RST, not a FIN: what a dead link or a dropped request looks like."""
            try:
                self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                           struct.pack("ii", 1, 0))
            except OSError:
                pass
            self.close_connection = True
            try:
                self.connection.close()
            except OSError:
                pass

        def _reply(self, code: int, headers, body: bytes) -> None:
            self.send_response(code)
            for k, v in headers:
                if k.lower() not in HOP:
                    self.send_header(k, v)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _control(self, body: bytes) -> None:
            if self.command == "POST":
                try:
                    got = state.set(json.loads(body or b"{}"))
                except (ValueError, TypeError) as e:
                    return self._reply(400, [], json.dumps({"error": str(e)}).encode())
                note({"t": round(time.monotonic(), 4), "control": got})
            self._reply(200, [("Content-Type", "application/json")],
                        json.dumps(state.snapshot()).encode())

        def _handle(self) -> None:
            n = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(n) if n else b""
            if self.path.startswith("/__fault"):
                return self._control(body)
            t0 = time.monotonic()
            faults = state.snapshot()["faults"]
            if faults["dead"]:
                state.count("dead")
                note({"t": round(t0, 4), "m": self.command, "p": self.path, "fate": "dead"})
                return self._reset()
            if state.roll_drop():
                state.count("dropped")
                note({"t": round(t0, 4), "m": self.command, "p": self.path, "fate": "dropped"})
                return self._reset()
            d = state.delay()
            if d:
                time.sleep(d)
            headers = {k: v for k, v in self.headers.items() if k.lower() not in HOP}
            headers["Host"] = f"{host}:{port}"
            dup = faults["dup_action"] and self.command == "POST" and self.path.split("?")[0] == "/action"
            if dup:
                threading.Thread(target=self._dup, args=(self.path, headers, body), daemon=True).start()
            try:
                code, rh, rb = forward(self.command, self.path, headers, body)
            except (OSError, http.client.HTTPException) as e:
                state.count("upstream_error")
                note({"t": round(t0, 4), "m": self.command, "p": self.path,
                      "fate": f"upstream_error {type(e).__name__}"})
                return self._reset()
            d2 = state.delay()
            if d2:
                time.sleep(d2)
            state.count("forwarded")
            note({"t": round(t0, 4), "m": self.command, "p": self.path, "fate": code,
                  "s": round(time.monotonic() - t0, 4), **({"dup": True} if dup else {})})
            self._reply(code, rh, rb)

        def _dup(self, path: str, headers: dict, body: bytes) -> None:
            # `path` is passed in: on a kept-alive connection this handler's
            # `self.path` is the NEXT request's by the time the copy goes.
            time.sleep(DUP_DELAY_S)
            try:
                code = forward("POST", path, headers, body)[0]
            except (OSError, http.client.HTTPException) as e:
                code = type(e).__name__
            state.count("dup_sent")
            with state.lock:
                state.dup_status[str(code)] = state.dup_status.get(str(code), 0) + 1
            note({"t": round(time.monotonic(), 4), "m": "POST", "p": path, "dup_copy": code})

        do_GET = do_POST = do_PUT = do_DELETE = do_PATCH = _handle

    return Handler


def serve(port: int, upstream: str, log_path: str = "") -> ThreadingHTTPServer:
    log = open(log_path, "a", buffering=1) if log_path else None
    srv = ThreadingHTTPServer(("127.0.0.1", port), make_handler(State(), upstream, log))
    srv.daemon_threads = True
    return srv


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2:
        raise SystemExit(__doc__)
    serve(int(argv[0]), argv[1], os.environ.get("FAULT_PROXY_LOG", "")).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
