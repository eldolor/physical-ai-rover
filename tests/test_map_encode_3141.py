# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.F1: `/world/map`'s body encoded in slices (`robot.server.map_json`)
is byte for byte what `JSONResponse(m).body` -- the route's encode before --
gives, on maps of every shape the route sees and on awkward values; and no
single `json.dumps` call encodes more than one slice of a big list (the
encode never holds the GIL in one C call for a whole map: the board's late
wheel ticks, 3.141.F0)."""

import json
import math
import random

import pytest
from fastapi.responses import JSONResponse

import robot.server as server


def _map(n, seed=0):
    rng = random.Random(seed)
    return {"usable": True, "map_id": "slam-s1", "map_version": n, "resolution_m": 0.05,
            "width": n, "height": n, "origin_x_m": rng.uniform(-9, 9), "origin_y_m": -0.1 * n,
            "cells": [rng.choice((-1, 0, 100)) for _ in range(n * n)]}


@pytest.mark.parametrize("n", [0, 1, 7, 90, 91, 400])
def test_a_map_is_the_same_bytes(n):
    m = _map(n, n)
    assert server.map_json(m) == JSONResponse(m).body


def test_awkward_values_are_the_same_bytes():
    rng = random.Random(3141)
    for _ in range(300):
        m = {"usable": rng.random() < 0.5, "map_id": rng.choice(["slam-é", "a\"b", "☃", ""]),
             "map_version": rng.randrange(10 ** 9), "f": rng.uniform(-1e9, 1e9), "none": None,
             "cells": [rng.choice((-1, 0, 100, 1.5, -0.0, 1e-300, "x]", None, [1, 2], {"k": 1}))
                       for _ in range(rng.randrange(0, 3 * server.MAP_ENCODE_SLICE))],
             "small": [1, 2, 3], "nested": {"a": [1, {"b": "]"}]}}
        assert server.map_json(m) == JSONResponse(m).body


def test_what_json_refuses_is_still_refused():
    m = _map(10)
    m["cells"][5] = math.nan
    with pytest.raises(ValueError):
        JSONResponse(m)
    with pytest.raises(ValueError):
        server.map_json(m)
    bad = {1: "an int key"}
    assert server.map_json(bad) == JSONResponse(bad).body        # the whole-dict path


def test_no_one_call_encodes_a_whole_big_list(monkeypatch):
    lists, real = [], json.dumps

    def spy(obj, *a, **k):
        if isinstance(obj, (list, dict)):
            lists.append(len(obj))
        return real(obj, *a, **k)
    monkeypatch.setattr(server.json, "dumps", spy)
    m = _map(400)
    server.map_json(m)
    assert lists and max(lists) <= server.MAP_ENCODE_SLICE      # never the whole 160,000


def test_the_route_sends_the_same_reply(monkeypatch):
    """Full, untagged and 304 replies: status, content type and body as a
    JSONResponse of the same map would send."""
    from fastapi.testclient import TestClient
    m = _map(60)

    class World:
        def get_map(self):
            return dict(m)
    monkeypatch.setattr(server, "get_world", lambda *a, **k: World())
    tc = TestClient(server.create_app())
    r = tc.get("/world/map")
    assert r.status_code == 200 and r.content == JSONResponse(m).body
    assert r.headers["content-type"] == "application/json" and r.headers["etag"]
    assert tc.get("/world/map", headers={"if-none-match": r.headers["etag"]}).status_code == 304
    m["map_id"] = "bad id"                                   # cannot be tagged: sent in full
    r2 = tc.get("/world/map")
    assert r2.status_code == 200 and r2.content == JSONResponse(m).body
    assert r2.headers["content-type"] == "application/json" and "etag" not in r2.headers
