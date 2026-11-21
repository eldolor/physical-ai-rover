"""3.205.A: a find on a saved map whose inventory has no memory of the target
searches the floor the camera has not seen (3.70.M3's re-search by sight,
`ExploreAgent.resight`, widened from "handed a recall" to "on a carried map").

`docs/plans/ros-alignment/3.205-carried-map-no-memory.md`, "3.205.A:
criteria" (N7): in process through `MissionRunner`'s own seed path -- the
runner carries landmarks saved on the pose's saved map (`inventory_prior`)
and tells explore (`carried_map()`), none of them the target.
"""

import math

from control.mission_runner import EXHAUSTED, FOUND, MissionRunner
from sim.grid_world import CELL_WALL
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from world.interface import CELL_FREE, CELL_OCCUPIED

CELL = 0.30
FRAME = "map-3205"


class _SavedMap(MockWorld):
    def get_pose(self):
        p = super().get_pose()
        return dict(p, frame_id=FRAME) if p.get("usable") else p


def _known(grid):
    """The whole house mapped before the mission: a saved map's cells."""
    return [CELL_OCCUPIED if (grid._cell(x, y) == CELL_WALL or (x, y) in grid.objects) else CELL_FREE
            for y in range(grid.height) for x in range(grid.width)]


def _prior(labels):
    """A saved report on FRAME holding `labels`, none of them the target."""
    return {"frame_id": FRAME, "candidates": [],
            "reported": [{"label": lab, "x_m": 1.0 + i, "y_m": 1.0, "belief": 0.95}
                         for i, lab in enumerate(labels)]}


def _find(target, start, carried=("sofa",), job=None, recall=None, house="complex_house",
          known=True, resight=True):
    grid = build_world(house)
    grid.x, grid.y = start
    world = _SavedMap(grid)
    if known:
        world._cells = _known(grid)
    robot = MockRobot(grid, render=False)
    kw = {"job": job} if job else {"target_object": target}
    runner = MissionRunner(robot, policy="explore", navigator=FakeNav(robot), world=world,
                           max_steps=400, clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=False, **kw)
    prior = _prior(carried) if carried else None
    if recall is not None:
        prior["reported"].append({"label": target, "x_m": recall[0], "y_m": recall[1], "belief": 0.95})
    runner.inventory_prior = lambda frame_id: prior if frame_id == FRAME else None
    runner.start()
    events, note = [], runner.agent._event
    runner.agent._event = lambda name, pose=None, **f: (events.append((name, f)), note(name, pose, **f))[1]
    if not resight:
        runner.agent.carried_map = lambda: None     # dev before 3.205: the runner's word unheard
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner, grid, [n for n, _ in events], events


def _target_m(grid, target):
    tx, ty = next(c for c, n in grid.objects.items() if n == target)
    return math.dist((grid.x, grid.y), (tx + 0.5, ty + 0.5)) * CELL


def test_red_on_dev_a_carried_map_with_no_memory_ends_exhausted():
    """N7 (a), red: the runner's word unheard (dev), the laptop bag unremembered,
    the whole house one unseen patch looked at once: the find gives up."""
    runner, grid, names, _ = _find("laptop bag", (35.5, 38.5), resight=False)
    assert runner.inventory.carried == 1 and runner.agent.recall == []
    assert runner.status()["outcome"] == EXHAUSTED
    assert "resight" not in names


def test_a_carried_map_with_no_memory_re_searches_by_sight_and_finds():
    """N7 (a): the same find on the branch searches the unseen floor and finds
    the target by sight."""
    runner, grid, names, events = _find("laptop bag", (35.5, 38.5))
    s = runner.status()
    assert runner.agent._carried_map and not runner.agent._recalled
    assert s["outcome"] == FOUND, s["log_tail"][-5:]
    assert names.count("resight") == 1 and "recall_set" not in names
    assert dict(events)["resight"]["recalled"] is False
    assert _target_m(grid, "laptop bag") <= 0.9


def test_nothing_carried_never_re_searches():
    """N7 (b): a saved map with nothing saved on it (no landmark carried) is
    not a carried map to explore: unchanged."""
    runner, grid, names, _ = _find("purple elephant", (6.5, 4.5), carried=(), house="scaled_house")
    assert not runner.agent._carried_map
    assert runner.status()["outcome"] == EXHAUSTED
    assert "resight" not in names


def test_a_survey_on_a_carried_map_never_re_searches():
    """N7 (c)."""
    runner, grid, names, _ = _find(None, (6.5, 4.5), job="survey", house="scaled_house")
    assert runner.agent._carried_map
    assert "resight" not in names


def test_a_find_with_a_recall_re_searches_once_as_before():
    """N7 (d): 3.70.M3's case through the runner -- recalled and carried."""
    runner, grid, names, events = _find("purple elephant", (6.5, 4.5), house="scaled_house",
                                        recall=(12.15, 3.15))
    assert runner.agent._recalled and runner.agent._carried_map
    assert runner.status()["outcome"] == EXHAUSTED
    assert names.count("resight") == 1 and names.count("exhausted") == 1
    assert names.index("recall_set") < names.index("resight")
    assert dict(events)["resight"]["recalled"] is True


def test_an_agent_without_the_hook_is_not_told():
    """The runner's call is optional (`getattr`), as `recall_from` is."""
    runner, *_ = _find("purple elephant", (6.5, 4.5), house="scaled_house", known=False)
    runner.agent = object()
    runner._mark_carried_map()          # no hook: nothing happens, nothing raises
