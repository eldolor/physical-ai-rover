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


def _bridge_map(n, rng):
    return {"session": "s1", "version": rng.randrange(10 ** 6), "resolution_m": 0.05,
            "width": n, "height": n, "origin_x_m": rng.uniform(-9, 9), "origin_y_m": -1.5,
            "origin_yaw_rad": 0.0, "data": [rng.choice((-1, 0, 100, 37)) for _ in range(n * n)]}


@pytest.mark.parametrize("n", [0, 1, 50, 91, 400])
def test_a_bridge_map_parses_the_same(n):
    m = _bridge_map(n, random.Random(n))
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
    nested = b'{"m":{"data":[1,2]}}'                         # the one key not at the top level
    assert rw._map_body(_resp(nested)) == json.loads(nested)


@pytest.mark.parametrize("tail", [b",", b", ", b",\n ", b" , ", b",,"])
def test_a_trailing_or_double_comma_is_refused_wherever_the_cut_lands(tail):
    S = rw.MAP_PARSE_SLICE_BYTES
    for pad in (0, 1, S - 3, S - 1, S, S + 1, 2 * S - 2):
        inner = b"1," * 50 + b"1" + b" " * pad + tail
        body = b'{"data":[' + inner + b"]}"
        with pytest.raises(ValueError):
            json.loads(body)
        assert rw._map_body(_resp(body)) is None, (tail, pad)
    ok = b'{"data":[' + b"1," * (S // 2) + b"2" + b" " * S + b"]}"      # whitespace, no comma: fine
    assert rw._map_body(_resp(ok)) == json.loads(ok)


def test_a_malformed_grid_is_parsed_once(monkeypatch):
    calls, real = [], rw._body
    monkeypatch.setattr(rw, "_body", lambda r: calls.append(1) or real(r))
    r = _resp(b'{"session":"s","data":[1,2,]}')
    with pytest.raises(httpx.HTTPError):
        rw._json_object(r, rw._map_body(r))
    assert calls == []


def test_no_one_call_parses_a_whole_big_grid(monkeypatch):
    seen, real = [], json.loads

    def spy(s, *a, **k):
        seen.append(len(s))
        return real(s, *a, **k)
    monkeypatch.setattr(rw.json, "loads", spy)
    m = _bridge_map(400, random.Random(2))
    body = json.dumps(m).encode()
    assert rw._map_body(_resp(body)) == m
    assert max(seen) <= rw.MAP_PARSE_SLICE_BYTES + 64 and len(body) > 10 * rw.MAP_PARSE_SLICE_BYTES


def test_asarray_in_slices_is_the_same_array():
    rng = random.Random(3)
    for data in ([rng.choice((-1, 0, 100)) for _ in range(160000)], [], [5],
                 [rng.choice((-1, 0, 1.5)) for _ in range(9000)],
                 [0] * 9000 + [1.5], [0] * 9000 + [2 ** 63]):
        a, b = rw._as_array(data), np.asarray(data)
        assert a.dtype == b.dtype and np.array_equal(a, b)


def test_random_bodies_parse_as_json_loads_does(monkeypatch):
    """Cuts every few bytes, over valid and broken grids: the same dict, or
    None where `json.loads` refuses."""
    monkeypatch.setattr(rw, "MAP_PARSE_SLICE_BYTES", 5)
    rng = random.Random(3141)
    atoms = [b"1", b"-1", b"100", b"0", b"1.5", b"-0.0", b"1e3", b" ", b"\n", b",", b",", b",", b"]",
             b"x", b"nul", b"null", b"true", b"NaN"]
    same = refused = 0
    for _ in range(4000):
        vals = [rng.choice((b"1", b"-1", b"100", b"0", b"2.5")) for _ in range(rng.randrange(0, 40))]
        inner = b",".join(v + rng.choice((b"", b" ", b"\n ")) for v in vals)
        if rng.random() < 0.5:                       # break it somewhere
            i = rng.randrange(len(inner) + 1)
            inner = inner[:i] + rng.choice(atoms) + inner[i:]
        body = (b'{"session":"s","version":1,' + rng.choice((b"", b" ")) + b'"data":'
                + rng.choice((b"", b" ")) + b"[" + inner + b"]" + rng.choice((b"", b',"after":2')) + b"}")
        try:
            want = json.loads(body)
        except ValueError:
            want = None
        got = rw._map_body(_resp(body))
        assert got == want, body
        same += want is not None
        refused += want is None
    assert same > 1000 and refused > 500
