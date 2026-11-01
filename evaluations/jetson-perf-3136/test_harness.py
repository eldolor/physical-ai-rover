"""3.136.A's harness against a fake board (no ssh, no locks):

    .venv/bin/python -m pytest evaluations/jetson-perf-3136/test_harness.py -q
"""

import copy
import json
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import boardrun as br  # noqa: E402
import run as runner  # noqa: E402
import scorers  # noqa: E402
import validity  # noqa: E402

GOOD = {"kind": "full", "window": [100.0, 400.0], "bench": [110.0, 390.0],
        "wheel": {"late": 0, "moving": 6000}, "oom_kills": 0, "nav": {"ok": 18, "n": 18},
        "power_mode": "NV Power Mode: 15W", "desktop_load": [[1, 0.5, 0.5, 0.5]],
        "temps": 300, "clocks": 300}


def test_a_good_run_is_valid_and_scored():
    assert validity.judge(GOOD) == {"valid": True, "reasons": []}
    assert validity.score(GOOD, 183) == 183


@pytest.mark.parametrize("change, reason", [
    ({"kind": "quiet"}, "not a full-stack run"),
    ({"window": None}, "no full-stack window"),
    ({"bench": [90.0, 390.0]}, "before the window"),
    ({"bench": [110.0, 401.0]}, "after the window"),
    ({"wheel": {"late": 1, "moving": 6000}}, "1 late wheel ticks"),
    ({"wheel": {"late": 0, "moving": 0}}, "no moving wheel ticks"),
    ({"oom_kills": 1}, "1 OOM kills"),
    ({"oom_kills": None}, "OOM kills not counted"),
    ({"nav": {"ok": 17, "n": 18}}, "17/18"),
    ({"nav": {"ok": 0, "n": 0}}, "no nav2 goals"),
    ({"power_mode": "NV Power Mode: MAXN"}, "not 15W"),
    ({"desktop_load": []}, "desktop load"),
    ({"temps": 0}, "temperature"),
    ({"clocks": 0}, "clocks"),
])
def test_each_broken_condition_flags_the_run_and_it_is_not_scored(change, reason):
    run = {**GOOD, **change}
    j = validity.judge(run)
    assert not j["valid"]
    assert any(reason in r for r in j["reasons"]), j["reasons"]
    assert validity.score(run, 183) is None


def test_telemetry_counts_inside_the_window():
    rows = [{"t": t, "mem_mb": 900 - t, "oom_kill": 3 + (t >= 5), "temps": {"cpu": 45000, "tj": 51000},
             "cpu_khz": [[729600, 1497600]], "gpu_hz": [306000000, 612000000]} for t in range(10)]
    c = validity.telemetry_counts(rows, 2, 7)
    assert c["samples"] == 6 and c["temps"] == 6 and c["clocks"] == 6
    assert c["max_temp_c"] == 51.0 and c["oom_kills"] == 1 and c["mem_available_min_mb"] == 893
    assert c["cpu_below_max_samples"] == 6 and c["gpu_below_max_samples"] == 6
    rows[3]["temps"]["PMIC-Die"] = 100000
    rows[4]["gpu_hz"] = [None, None]
    c = validity.telemetry_counts(rows, 2, 7)
    assert c["max_temp_c"] == 51.0 and c["clocks"] == 5


def test_perception_scorer_reads_frame_lines_inside_the_window_only():
    t0 = time.mktime(time.strptime("2026-11-01 10:00:00", "%Y-%m-%d %H:%M:%S"))
    log = "loaded 1 pipeline\n09:59:59 900 absent\n10:00:00 120 detected\n10:00:01 130 absent\n" \
          "10:00:05 999 absent\ndone: 4 frames\n"
    assert scorers.perception_ms(log) == [900, 120, 130, 999]
    assert scorers.perception_ms(log, t0, t0 + 2) == [120, 130]


def test_percentile_is_nearest_rank():
    assert scorers.percentile(list(range(1, 11)), 90) == 9
    assert scorers.percentile([5], 90) == 5
    assert scorers.percentile([], 90) is None


def test_nav_counts_and_headroom_window_across_midnight():
    assert scorers.nav_counts("a: succeeded\ngoals 17/18 succeeded\n") == {"ok": 17, "n": 18}
    assert scorers.nav_counts("nothing") is None
    t0, t1 = scorers.headroom_window("== window start 23:59:50; x\n== window end 00:00:10 bench", "2026-11-01")
    assert t1 - t0 == 20


def test_wheel_delta():
    d = scorers.wheel_delta({"wheel_loop": {"moving_ticks": 10, "late_ticks": 1, "max_dt_s": 0.06}},
                            {"wheel_loop": {"moving_ticks": 210, "late_ticks": 2, "max_dt_s": 0.152}})
    assert d == {"moving": 200, "late": 1, "max_dt_s": 0.152, "max_dt_s_before": 0.06}


# ------------------------------------------------------------ a fake board

FOUND = {"units": {u: {"active": "active", "enabled": "enabled"} for u in br.UNITS},
         "hold_flag": None, "vlm_env": {"sha256": "ab", "mode": "600"}, "containers": "",
         "listening": "8000 8001 8005 8080 ", "power_mode": "NV Power Mode: 15W",
         "mem_available_mb": 2500, "oom_kill": 0, "uptime_s": 1.0, "epoch": 1.0}


class FakeBoard:
    def __init__(self, state=None):
        self.state = copy.deepcopy(state or FOUND)
        self.cmds = []

    def __call__(self, cmd, check=True, timeout=0, input=None):
        self.cmds.append(cmd)
        if "systemctl is-active" in cmd and "units=" in cmd:       # the snapshot script
            return json.dumps(self.state)
        if "sudo -n systemctl stop picar-stack picar-ngrok" in cmd:
            for u in ("picar-stack", "picar-ngrok"):
                self.state["units"][u]["active"] = "inactive"
            return ""
        if "sudo -n systemctl" in cmd and "start" in cmd:
            u = cmd.split()[-1]
            self.state["units"][u]["active"] = "active"
            return ""
        if "sudo -n rm -f" in cmd:
            self.state["hold_flag"] = None
            return ""
        if "/proc/" in cmd and "stat" in cmd:
            return ""                                             # the flag's owner is gone
        if "echo $?" in cmd:
            return "0\n"
        return ""


def test_differences_name_what_moved_and_ignore_what_moves_by_itself():
    after = copy.deepcopy(FOUND)
    after["mem_available_mb"] = 100
    after["uptime_s"] = 99
    assert br.differences(FOUND, after) == []
    after["units"]["picar-ngrok"]["active"] = "inactive"
    after["hold_flag"] = "123 456"
    d = br.differences(FOUND, after)
    assert len(d) == 2 and "picar-ngrok" in d[0] and "hold_flag" in d[1]


def test_without_sudo_nothing_is_stopped_and_a_difference_is_reported_not_hidden():
    b = FakeBoard()
    with pytest.raises(br.BoardError):
        br.stop_twin(b, allow_sudo=False)
    b.state["units"]["picar-ngrok"]["active"] = "failed"
    acts = br.restore(FOUND, b, allow_sudo=False)
    assert acts and acts[0].startswith("NOT RESTORED")
    assert not any("sudo" in c for c in b.cmds)


def test_with_sudo_restore_puts_back_units_and_a_stale_hold_flag():
    b = FakeBoard()
    br.stop_twin(b, allow_sudo=True)
    b.state["units"]["picar-local-vlm"]["active"] = "inactive"
    b.state["hold_flag"] = "4242 99"
    acts = br.restore(FOUND, b, allow_sudo=True)
    assert any("stale hold flag" in a for a in acts)
    assert sum(a.startswith("started") for a in acts) == 3
    after, diffs = br.settle_and_check(FOUND, b, wait_s=0)
    assert diffs == [] and after["hold_flag"] is None
    # The flag goes before any start: while it exists the VLM's starts are skipped.
    rm = next(i for i, c in enumerate(b.cmds) if "rm -f" in c)
    first_start = next(i for i, c in enumerate(b.cmds) if "systemctl start" in c)
    assert rm < first_start


def test_board_run_refuses_without_the_board_lock(monkeypatch, tmp_path):
    monkeypatch.delenv("PICAR_BOARD_TOKEN", raising=False)
    with pytest.raises(br.BoardError):
        runner.board_run("x", "true", tmp_path, run=FakeBoard())


class FakeSampler:
    def __init__(self, out, run):
        self.desktop_load = [[1, 0.1, 0.1, 0.1]]

    def start(self):
        pass

    def stop(self):
        pass


def test_board_run_restores_even_when_the_command_breaks_the_board(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()

    def push(sha, run):
        return "f" * 40

    def bad_cmd_board(cmd, check=True, timeout=0, input=None):
        if "echo $?" in cmd:          # the bench: stops the VLM, then dies
            b.state["units"]["picar-local-vlm"]["active"] = "inactive"
            raise RuntimeError("ssh dropped")
        return b(cmd, check, timeout, input)

    with pytest.raises(RuntimeError):
        runner.board_run("x", "true", tmp_path, allow_sudo=True, stop_twin=True, run=bad_cmd_board,
                         push=push, fetch=lambda *a: None, sampler_cls=FakeSampler)
    rec = json.loads((tmp_path / "record.json").read_text())
    assert rec["left_as_found"] and rec["as_found_differences"] == []
    assert sum(a.startswith("started") for a in rec["restore_actions"]) == 3


def test_board_run_passes_the_token_to_the_board_command(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()
    rec = runner.board_run("x", "echo hi", tmp_path, run=b, push=lambda s, r: "f" * 40,
                           fetch=lambda *a: None, sampler_cls=FakeSampler)
    cmd = next(c for c in b.cmds if "echo hi" in c)
    assert "PICAR_BOARD_TOKEN=t0k" in cmd and rec["cmd_exit"] == 0 and rec["left_as_found"]
    assert not any("sudo" in c for c in b.cmds)


def test_plant_verdict_tolerance():
    import plant
    assert plant.verdict(30, 131)["met"] and plant.verdict(30, 139.9)["met"]
    assert not plant.verdict(30, 141)["met"] and not plant.verdict(30, 119)["met"]
