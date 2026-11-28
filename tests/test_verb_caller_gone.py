# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_verb_caller_gone.py

`PLAN-ros-alignment.md` 3.147 -- a bounded verb outlived its dead brain
(3.135.B F6: ~24 cm of true motion after a `kill -9` 0.15 s into a FORWARD,
bar 15 cm). The fix: the robot server ends an AUTONOMOUS driver's verb whose
HTTP caller hung up -- a killed process's sockets are closed by its kernel.

Judged on GROUND TRUTH (the body program's `/truth`), on the hardware backend
over the fake board (3.36's split programs), through a REAL uvicorn socket:
the brain's death is a closed socket, which only a real connection has. One
test per criterion of the section file; criterion numbers in the names.
"""

import json
import math
import socket
import threading
import time

import httpx
import pytest
import uvicorn

from tests.conftest import free_port

CLOSE_AFTER_S = 0.15      # F6's kill, into the verb
TRAVEL_BAR_M = 0.08       # criterion 1/2 (the row's own bar is 0.15)
TURN_BAR_DEG = 25.0       # criterion 2, LEFT
STOP_BAR_S = 0.35         # criterion 1: wheels zero after the close


@pytest.fixture
def stack(monkeypatch, sim_programs):
    """The robot server (hardware backend, fake board, split body) on a real
    socket in this process. Yields (robot_url, body_url, app)."""
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    programs = sim_programs(SIM_MAP="scaled_house").apply(monkeypatch)
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("WORLD_MODE", "none")
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    import robot.server as server
    app = server.create_app()
    port = free_port()
    srv = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port,
                                        log_level="warning", lifespan="on"))
    t = threading.Thread(target=srv.run, daemon=True)
    t.start()
    url = f"http://127.0.0.1:{port}"
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        try:
            if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                break
        except httpx.HTTPError:
            pass
        time.sleep(0.1)
    else:
        pytest.fail("robot server never answered /health")
    time.sleep(1.0)         # the first wheel feedback
    try:
        yield url, programs.body_url, app
    finally:
        srv.should_exit = True
        t.join(timeout=10)


def truth(body_url):
    t = httpx.get(f"{body_url}/truth", timeout=2.0).json()
    return t


def xy(t):
    return t["x_m"], t["y_m"]


def raw_action(url, body: dict, driver: str) -> socket.socket:
    """POST /action on a raw socket and return it unread: the caller's side,
    to close (a death) or read (a living caller)."""
    host, port = url.removeprefix("http://").split(":")
    payload = json.dumps(body).encode()
    s = socket.create_connection((host, int(port)))
    s.sendall((f"POST /action HTTP/1.1\r\nHost: {host}\r\nContent-Type: application/json\r\n"
               f"x-driver: {driver}\r\nContent-Length: {len(payload)}\r\n\r\n").encode() + payload)
    return s


def read_reply(s: socket.socket) -> dict:
    s.settimeout(10)
    data = b""
    while b"\r\n\r\n" not in data:
        data += s.recv(65536)
    head, _, rest = data.partition(b"\r\n\r\n")
    n = int([h for h in head.split(b"\r\n") if h.lower().startswith(b"content-length")][0]
            .split(b":")[1])
    while len(rest) < n:
        rest += s.recv(65536)
    s.close()
    return json.loads(rest[:n])


def run_until_rest(body_url, timeout=4.0):
    """Sample truth until the body has not moved for 0.4 s. Returns
    (samples [(t, truth)], rest truth)."""
    samples = []
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        samples.append((time.monotonic(), truth(body_url)))
        time.sleep(0.03)
        if len(samples) > 14:
            a, b = samples[-14][1], samples[-1][1]
            if (math.dist(xy(a), xy(b)) < 0.002
                    and abs(((a["heading_deg"] - b["heading_deg"] + 180) % 360) - 180) < 0.3):
                break
    return samples, samples[-1][1]


def died_mid_verb(url, body_url, action, driver="brain"):
    """The caller sends the brain's verb and dies CLOSE_AFTER_S into it.
    Returns (travel_m after the close, turn_deg after the close, s to rest)."""
    body = {"action": action, "speed": 50, "duration": 0.5}
    if action in ("LEFT", "RIGHT"):
        body["angle"] = 90
    s = raw_action(url, body, driver)
    time.sleep(CLOSE_AFTER_S)
    s.close()
    t_close = time.monotonic()
    at_close = truth(body_url)
    samples, rest = run_until_rest(body_url)
    # The last moment the body was still moving, after the close.
    moving_until = t_close
    for (ta, a), (tb, b) in zip(samples, samples[1:]):
        if math.dist(xy(a), xy(b)) > 0.001 or abs(a["heading_deg"] - b["heading_deg"]) > 0.2:
            moving_until = tb
    turn = abs(((rest["heading_deg"] - at_close["heading_deg"] + 180) % 360) - 180)
    return math.dist(xy(at_close), xy(rest)), turn, moving_until - t_close


def test_criterion_1_a_forward_ends_when_its_brain_hangs_up(stack):
    url, body_url, _ = stack
    rows = []
    for _ in range(5):
        rows.append(died_mid_verb(url, body_url, "FORWARD"))
        died_mid_verb(url, body_url, "REVERSE")     # back to the clear run
        time.sleep(1.2)        # the watchdog's lapse: the next close is a new claim
    travels = [round(r[0], 3) for r in rows]
    stops = [round(r[2], 2) for r in rows]
    print("3.147 c1 travel_m", travels, "to_rest_s", stops)
    assert all(t <= TRAVEL_BAR_M for t in travels), travels
    assert all(s <= STOP_BAR_S for s in stops), stops


def test_criterion_2_reverse_and_left_end_too(stack):
    url, body_url, _ = stack
    for action in ("REVERSE", "LEFT"):
        for _ in range(3):
            travel, turn, _ = died_mid_verb(url, body_url, action)
            print("3.147 c2", action, round(travel, 3), round(turn, 1))
            if action == "REVERSE":
                assert travel <= TRAVEL_BAR_M, (action, travel)
            else:
                assert turn <= TURN_BAR_DEG, (action, turn)
            time.sleep(1.2)


def whole_forward(url, body_url, driver="brain"):
    t0 = truth(body_url)
    s = raw_action(url, {"action": "FORWARD", "speed": 50, "duration": 0.5}, driver)
    reply = read_reply(s)
    _, rest = run_until_rest(body_url)
    return reply, math.dist(xy(t0), xy(rest))


def test_criterion_3_a_living_brain_is_never_cut(stack):
    url, body_url, _ = stack
    travels = []
    for i in range(10):
        reply, travel = whole_forward(url, body_url)
        assert reply["executed"], reply
        travels.append(travel)
        # back, so the run stays clear
        read_reply(raw_action(url, {"action": "REVERSE", "speed": 50, "duration": 0.5}, "brain"))
    ref = travels[0]
    print("3.147 c3 travel_m", [round(t, 3) for t in travels])
    assert ref > 0.2, travels
    assert all(abs(t - ref) <= 0.03 for t in travels), travels


def test_criterion_4_a_persons_verb_runs_to_its_end(stack):
    url, body_url, _ = stack
    _, whole = whole_forward(url, body_url, driver="twin-dpad")
    read_reply(raw_action(url, {"action": "REVERSE", "speed": 50, "duration": 0.5}, "twin-dpad"))
    t0 = truth(body_url)
    s = raw_action(url, {"action": "FORWARD", "speed": 50, "duration": 0.5}, "twin-dpad")
    time.sleep(CLOSE_AFTER_S)
    s.close()
    _, rest = run_until_rest(body_url)
    print("3.147 c4 whole", round(whole, 3), "closed", round(math.dist(xy(t0), xy(rest)), 3))
    assert abs(math.dist(xy(t0), xy(rest)) - whole) <= 0.03, (whole, math.dist(xy(t0), xy(rest)))


def test_criterion_5_ranks_and_watchdog_unchanged():
    """3.147 changes neither who drives nor the watchdog's timeout."""
    from pathlib import Path

    import yaml

    from robot.interface import DRIVER_PRIORITY
    cfg = yaml.safe_load((Path(__file__).resolve().parent.parent
                          / "config" / "robot.yaml").read_text())
    assert cfg["safety"]["watchdog_timeout_s"] == 1.0
    assert DRIVER_PRIORITY == {"twin-dpad": 30, "teleop-operator": 30, "brain": 20,
                               "teleop": 20, "ros": 20, "twin-local-brain": 10}


def test_criterion_6_health_says_why(stack):
    url, body_url, _ = stack
    died_mid_verb(url, body_url, "FORWARD")
    h = httpx.get(f"{url}/health", timeout=2).json()
    reason = h.get("last_stop_reason") or {}
    assert reason.get("reason") == "driver_gone", reason
    assert reason.get("driver") == "brain", reason
