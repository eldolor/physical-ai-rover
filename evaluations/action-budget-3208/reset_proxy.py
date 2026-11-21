# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/action-budget-3208/reset_proxy.py

3.208 criterion 6: an HTTP/1.1 reverse proxy between the brain's
`RemoteRobot` (its `ROBOT_URL`) and the robot server that resets chosen
`POST /action` connections with a TCP RST (SO_LINGER 0).

    python reset_proxy.py <listen_port> <upstream_url>

Moves (`FORWARD`, `REVERSE`, `LEFT`, `RIGHT`) are numbered from 1 as they
arrive. The plan is set on the proxy's own route, never forwarded:

    POST /__plan {"resets": {"5": "before", "12": "after"}, "from": null}
    GET  /__plan -> the plan and `wire`: every POST in order,
                    {n, path, action, how, forwarded}

`before`: the request is read and the connection reset; the robot never
sees it. `after`: the request is forwarded, the robot's answer read, and the
connection reset instead of answering. `from: k` resets EVERY request
(any method and path) from the k-th move on, `before`: the link is dead.
Nothing is ever re-sent by the proxy.
"""

import http.client
import json
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

MOVES = {"FORWARD", "REVERSE", "LEFT", "RIGHT"}
HOP = {"connection", "keep-alive", "transfer-encoding", "content-length", "host", "date", "server"}


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.resets = {}
        self.reset_from = None
        self.moves = 0
        self.wire = []
        self.dead = False
        self.dead_at = None      # wall time of the first reset of the dead link


STATE = State()
UPSTREAM = None


def _rst(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    sock.close()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _reply(self, code, body: dict):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> bytes:
        n = int(self.headers.get("content-length") or 0)
        return self.rfile.read(n) if n else b""

    def do_GET(self):
        if self.path == "/__plan":
            with STATE.lock:
                return self._reply(200, {"resets": STATE.resets, "from": STATE.reset_from,
                                         "moves": STATE.moves, "wire": STATE.wire,
                                         "dead_at": STATE.dead_at})
        if self._dead_reset():
            return
        self._forward(b"")

    def _dead_reset(self) -> bool:
        with STATE.lock:
            dead = STATE.dead
        if dead:
            self.close_connection = True
            _rst(self.connection)
        return dead

    def do_POST(self):
        body = self._body()
        if self.path == "/__plan":
            plan = json.loads(body or b"{}")
            with STATE.lock:
                STATE.resets = {int(k): v for k, v in (plan.get("resets") or {}).items()}
                STATE.reset_from = plan.get("from")
                STATE.moves = 0
                STATE.wire = []
                STATE.dead = False
                STATE.dead_at = None
            return self._reply(200, {"ok": True})
        how, rec = None, None
        if self.path.split("?")[0] in ("/action", "/stop"):
            action = None
            if self.path.startswith("/action"):
                try:
                    action = json.loads(body or b"{}").get("action")
                except ValueError:
                    action = None
            with STATE.lock:
                n = None
                if action in MOVES:
                    STATE.moves += 1
                    n = STATE.moves
                    how = STATE.resets.get(n)
                    if STATE.reset_from is not None and n >= STATE.reset_from:
                        how = "before"
                        if not STATE.dead:
                            STATE.dead, STATE.dead_at = True, time.time()
                if STATE.dead and how is None:
                    how = "before"
                rec = {"n": n, "path": self.path, "action": action or "STOP", "how": how,
                       "forwarded": how != "before", "t": time.time()}
                STATE.wire.append(rec)
        elif self._dead_reset():
            return
        if how == "before":
            self.close_connection = True
            _rst(self.connection)
            return
        self._forward(body, reset_after=(how == "after"))

    def _forward(self, body: bytes, reset_after: bool = False):
        u = urlsplit(UPSTREAM)
        conn = http.client.HTTPConnection(u.hostname, u.port, timeout=60)
        headers = {k: v for k, v in self.headers.items() if k.lower() not in HOP}
        try:
            conn.request(self.command, self.path, body=body or None, headers=headers)
            resp = conn.getresponse()
            data = resp.read()
        except OSError as e:
            return self._reply(502, {"detail": f"proxy upstream: {e}"})
        finally:
            conn.close()
        if reset_after:
            self.close_connection = True
            _rst(self.connection)
            return
        self.send_response(resp.status)
        for k, v in resp.getheaders():
            if k.lower() not in HOP:
                self.send_header(k, v)
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    global UPSTREAM
    port, UPSTREAM = int(sys.argv[1]), sys.argv[2].rstrip("/")
    ThreadingHTTPServer.daemon_threads = True
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
