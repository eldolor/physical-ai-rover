# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/hang_cloud.py

F12's cloud: a local listener that accepts every connection, reads the
request line, and never answers -- a cloud that hangs. $0: nothing leaves
the machine. One JSON line per request in `<log>`: `t` (monotonic), the
method and path, and how many paid calls (`POST /navigate`, `POST /room`)
were open at that instant, this one included -- "a second paid call
stacked on a hung one" is `open_paid` >= 2. A connection's end (the
caller's timeout) is logged as `closed` with its stamp.

    python evaluations/jetson-fmea-3135/hang_cloud.py <port> <calls.jsonl>
"""

import json
import socket
import sys
import threading
import time

PAID = {("POST", "/navigate"), ("POST", "/room")}


def main():
    port, path = int(sys.argv[1]), sys.argv[2]
    out = open(path, "a", buffering=1)
    lock = threading.Lock()
    open_paid = [0]

    def write(row):
        with lock:
            out.write(json.dumps(row) + "\n")

    def serve(conn):
        paid = False
        try:
            conn.settimeout(10.0)
            line = conn.makefile("rb").readline(4096).decode(errors="replace").split()
            method, target = (line + ["", ""])[:2]
            paid = (method, target) in PAID
            with lock:
                if paid:
                    open_paid[0] += 1
                row = {"t": round(time.monotonic(), 4), "method": method, "path": target,
                       "paid": paid, "open_paid": open_paid[0]}
            write(row)
            conn.settimeout(None)
            while conn.recv(4096):          # never answer; wait for the caller to give up
                pass
        except OSError:
            pass
        finally:
            if paid:
                with lock:
                    open_paid[0] -= 1
            write({"t": round(time.monotonic(), 4), "closed": True, "paid": paid})
            conn.close()

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(64)
    write({"t": round(time.monotonic(), 4), "listening": port})
    while True:
        conn, _ = s.accept()
        threading.Thread(target=serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
