# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_tick_within.py

The brain's mission loop awaits each tick through `_tick_within`, not
`asyncio.wait_for`: Python 3.10's wait_for (the Jetson's interpreter)
returns the work's result when the work finished in the same loop turn as a
cancel, so a brain Stop that landed as a tick ended was lost and the
mission ran on (found on the board 2026-11-01, as a failure of
test_intervention_log's stopped-mission line under load). The laptop's
newer Python never swallows the cancel, so the first test is red only on
3.10 with wait_for in its place.
"""

import ast
import asyncio
import inspect

import pytest

from control import brain_server
from control.brain_server import _tick_within


def _run(coro):
    return asyncio.run(coro)


def test_a_cancel_as_the_work_finishes_is_not_lost():
    async def scenario():
        work = asyncio.get_running_loop().create_future()
        loop_task = asyncio.ensure_future(_tick_within(work, 5.0))
        await asyncio.sleep(0)              # the loop is now awaiting the tick
        work.set_result("ticked")           # the tick ends ...
        loop_task.cancel()                  # ... in the same turn as the Stop
        with pytest.raises(asyncio.CancelledError):
            await loop_task
    _run(scenario())


def test_a_cancel_mid_tick_cancels_the_wait():
    async def scenario():
        work = asyncio.get_running_loop().create_future()
        loop_task = asyncio.ensure_future(_tick_within(work, 5.0))
        await asyncio.sleep(0)
        loop_task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await loop_task
        assert work.cancelled()
    _run(scenario())


def test_a_tick_past_its_deadline_times_out():
    async def scenario():
        work = asyncio.get_running_loop().create_future()
        with pytest.raises(asyncio.TimeoutError):
            await _tick_within(work, 0.01)
        assert work.cancelled()
    _run(scenario())


def test_the_ticks_result_and_errors_come_back():
    async def ok():
        return 7

    async def boom():
        raise RuntimeError("tick failed")

    assert _run(_tick_within(ok(), 1.0)) == 7
    with pytest.raises(RuntimeError, match="tick failed"):
        _run(_tick_within(boom(), 1.0))


def test_the_brain_server_never_awaits_through_wait_for():
    calls = [n for n in ast.walk(ast.parse(inspect.getsource(brain_server)))
             if isinstance(n, ast.Attribute) and n.attr == "wait_for"
             and isinstance(n.value, ast.Name) and n.value.id == "asyncio"]
    assert not calls, "asyncio.wait_for swallows a cancel on Python 3.10: use _tick_within"


def test_a_tick_that_fails_as_the_stop_lands_is_logged(caplog):
    async def scenario():
        work = asyncio.get_running_loop().create_future()
        loop_task = asyncio.ensure_future(_tick_within(work, 5.0))
        await asyncio.sleep(0)
        work.set_exception(RuntimeError("robot unreachable"))
        loop_task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await loop_task
    with caplog.at_level("ERROR", logger="brain_server"):
        _run(scenario())
    assert "robot unreachable" in caplog.text
