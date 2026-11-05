# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.F1: the bridge's `/slam/map` body parsed in slices
(`world.ros_world._map_body`) is exactly `json.loads` of it -- every map
shape the bridge sends and every body that is not one (which falls back to
one parse) -- and no single `json.loads` call parses a whole big grid.
`_resample`'s cells stay the frozen loop's (tests/test_ros_world_resample.py)
with `asarray` in slices."""

import json
import random

import httpx
import numpy as np
import pytest

import world.ros_world as rw


def _resp(body: bytes, status=200):
    return httpx.Response(status, content=body, request=httpx.Request("GET", "http://b/slam/map"))


def _map_body(n, rng):
    return {"session": "s1", "version": rng.randrange(10 ** 6), "resolution_m": 0.05,
            "width": n, "height": n, "origin_x_m": rng.uniform(-9, 9), "origin_y_m": -1.5,
            "origin_yaw_rad": 0.0, "data": [rng.choice((-1, 0, 100, 37)) for _ in range(n * n)]}


@pytest.mark.parametrize("n", [0, 1, 50, 91, 400])
def test_a_bridge_map_parses_the_same(n):
    m = _map_body(n, random.Random(n))
    for body in (json.dumps(m), json.dumps(m, indent=1), json.dumps(m, separators=(",", ":"))):
        assert rw._map_body(_resp(body.encode())) == json.loads(body)


def test_bodies_that_are_not_a_flat_grid_fall_back_to_one_parse():
    rng = random.Random(1)
    big = list(range(3 * rw.MAP_PARSE_SLICE_BYTES))
    cases = [
        {"data": [1, [2, 3]] + big},                       # nested
        {"data": [{"a": 1}] + big},                        # objects
        {"data": ["x,y", "]"] + big},                      # strings with a comma, a bracket
        {"error": "\"data\":[1,2]", "data": big},          # the key twice in the text
        {"data": [1.5, -0.0, 1e-300, 2e300] + big},        # floats
        {"data": big, "after": [9, 8]},                    # a key after it
        [1, 2, 3],                                         # not an object
    ]
    for c in cases:
        body = json.dumps(c).encode()
        got = rw._map_body(_resp(body))
        assert got == (c if isinstance(c, dict) else None)
    assert rw._map_body(_resp(b"<html>proxy</html>")) is None
    assert rw._map_body(_resp(b'{"data":[1,2,')) is None   # cut


def test_no_one_call_parses_a_whole_big_grid(monkeypatch):
    seen, real = [], json.loads

    def spy(s, *a, **k):
        seen.append(len(s))
        return real(s, *a, **k)
    monkeypatch.setattr(rw.json, "loads", spy)
    m = _map_body(400, random.Random(2))
    body = json.dumps(m).encode()
    assert rw._map_body(_resp(body)) == m
    assert max(seen) <= rw.MAP_PARSE_SLICE_BYTES + 64 and len(body) > 10 * rw.MAP_PARSE_SLICE_BYTES


def test_asarray_in_slices_is_the_same_array():
    rng = random.Random(3)
    for data in ([rng.choice((-1, 0, 100)) for _ in range(160000)], [], [5],
                 [rng.choice((-1, 0, 1.5)) for _ in range(9000)]):
        a, b = rw._as_array(data), np.asarray(data)
        assert a.dtype == b.dtype and np.array_equal(a, b)
