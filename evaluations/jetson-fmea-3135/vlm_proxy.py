# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/vlm_proxy.py

A pass-through in front of the board's local VLM (VE, `picar-local-vlm` on
:8005), so the VE rows (F3, F17; amendment 8) can see from outside when a
question is open, and of which kind, without touching the brain or VE:
the brain's `LOCAL_VLM_URL` points here, and every request is forwarded
as it came (headers, body) and answered as VE answered.

* One JSON line per request in `<log>`: `t0`/`t1` (monotonic), the route,
  `kind` -- `frame` (a frame's question, sent with `box`), `arrival` (the
  arrival's identity question, no box), `direction`, `health` -- and VE's
  status (or the error a dead VE gave).
* `GET /fmea/open`: the requests open now, oldest first, each with its kind
  and `t0` -- what an injector waits on to fire mid-question.
* An upstream that cannot be reached (refused, reset, dropped: a killed VE)
  is passed on as a dropped connection, never as an answer, so the brain's
  client reads it as it reads VE gone (`LocalVLMAbsent`, `brain/local_vlm.py`);
  VE's own HTTP statuses (its 503 while it reloads) pass through as given.

    python evaluations/jetson-fmea-3135/vlm_proxy.py <port> <upstream> <log.jsonl>
"""

import json
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

FORWARD_HEADERS = ("content-type", "x-app-secret", "authorization")


def kind_of(path: str, body: bytes) -> str:
    if path.startswith("/health"):
        return "health"
    if path.startswith("/direction"):
        return "direction"
    if path.startswith("/identity"):
        try:
            return "frame" if json.loads(body or b"{}").get("box") else "arrival"
        except ValueError:
            return "identity"
    return "other"


def main():
    port, upstream, log = int(sys.argv[1]), sys.argv[2].rstrip("/"), sys.argv[3]
    out = open(log, "a", buffering=1)
    lock = threading.Lock()
    open_reqs: dict = {}
    seq = [0]

    class H(BaseHTTPRequestHandler):
        # One request a connection (HTTP/1.0, closed after each reply): a kept-alive
        # connection the proxy dropped could fail in a way VE itself never does
        # (seen on the board: one EBADF in 142 questions, 2026-11-03).
        protocol_version = "HTTP/1.0"

        def log_message(self, *a):
            pass

        def _reply(self, code: int, body: bytes, ctype: str = "application/json"):
            self.send_response(code)
            self.send_header("content-type", ctype)
            self.send_header("content-length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _forward(self, method: str):
            n = int(self.headers.get("content-length") or 0)
            body = self.rfile.read(n) if n else b""
            if self.path == "/fmea/open":
                with lock:
                    rows = sorted(open_reqs.values(), key=lambda r: r["t0"])
                return self._reply(200, json.dumps({"open": rows, "now": time.monotonic()}).encode())
            k = kind_of(self.path, body)
            with lock:
                seq[0] += 1
                i = seq[0]
                open_reqs[i] = {"i": i, "kind": k, "path": self.path, "t0": time.monotonic()}
            row = dict(open_reqs[i])
            code, data, ctype = None, b"", "application/json"
            try:
                req = urllib.request.Request(upstream + self.path, data=body if method == "POST" else None,
                                             method=method,
                                             headers={h: self.headers[h] for h in FORWARD_HEADERS
                                                      if self.headers.get(h)})
                try:
                    with urllib.request.urlopen(req, timeout=120) as r:
                        code, data = r.status, r.read()
                        ctype = r.headers.get("content-type", ctype)
                except urllib.error.HTTPError as e:
                    code, data = e.code, e.read()
                row["status"] = code
            except OSError as e:
                row["error"] = f"{type(e).__name__}: {e}"
            finally:
                with lock:
                    open_reqs.pop(i, None)
                row["t1"] = time.monotonic()
                with lock:
                    out.write(json.dumps(row) + "\n")
            if code is None:                   # VE unreachable: dropped, as VE's own death is
                self.close_connection = True
                return
            try:
                self._reply(code, data, ctype)
            except OSError:
                pass

        def do_GET(self):
            self._forward("GET")

        def do_POST(self):
            self._forward("POST")

    ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()


if __name__ == "__main__":
    main()
