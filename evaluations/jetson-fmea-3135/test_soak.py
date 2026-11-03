# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.135.C's soak loop and scorer against made-up records (no stack, no
ssh, no locks):

    .venv/bin/python -m pytest evaluations/jetson-fmea-3135/test_soak.py -q
"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import soak  # noqa: E402
import soak_score as ss  # noqa: E402

_spec = importlib.util.spec_from_file_location("fmea_run", HERE / "run.py")
fmea_run = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fmea_run)

H = 3600.0


# ------------------------------------------------------------ the workload

def test_every_mission_is_a_new_target_and_the_houses_take_turns():
    plan = soak.schedule(200)
    assert [h for h, _ in plan[:10]] == list(soak.HOUSES) * 2
    assert all(a[1] != b[1] for a, b in zip(plan, plan[1:]))
    for h, t in plan:
        assert t in soak.TARGETS[h]
    # each house goes through its whole list
    for h in soak.HOUSES:
        assert {t for hh, t in plan if hh == h} == set(soak.TARGETS[h])


def test_a_shared_target_next_to_itself_is_skipped():
    plan = soak.schedule(4, houses=("a", "b"), targets={"a": ("x", "y"), "b": ("x", "z")})
    assert plan == [("a", "x"), ("b", "z"), ("a", "y"), ("b", "x")]


def test_each_house_list_is_the_preference_rule_and_in_the_house():
    from sim.maps import build_world
    for h in soak.HOUSES:
        objs = set(build_world(h).objects.values())
        assert soak.TARGETS[h] == tuple(t for t in soak.PREFERENCE if t in objs)[:8], h
        assert len(soak.TARGETS[h]) == 8


def test_the_stack_gets_ros_the_local_tier_and_explore(tmp_path):
    env = soak.stack_env("s3cret")
    assert env == {"ROBOT_DRIVE": "ros", "WORLD_MODE": "ros", "LOCAL_VLM_URL": soak.VE_URL,
                   "TIER_SEARCH": "explore", "LOCAL_VLM_SECRET": "s3cret"}
    assert "LOCAL_VLM_SECRET" not in soak.stack_env("")
    f = tmp_path / "vlm.env"
    f.write_text("LOCAL_VLM_ENGINES=/x\nAPP_SHARED_SECRET='abc'\n")
    assert soak.read_vlm_secret(f) == "abc"
    assert soak.read_vlm_secret(tmp_path / "none") == ""


def test_every_mission_encodes_its_target_in_the_soaks_own_cache(tmp_path, monkeypatch):
    """3.142 criterion 1: run 1's perception loop read the board's own text
    cache, so a target cached by an earlier bench or mission encoded nothing
    (5 of 19 missions) and the new-target path did not run every time."""
    monkeypatch.setattr(soak, "read_vlm_secret", lambda *a, **k: "")
    s = soak.Soak(tmp_path / "out", 60)
    cache = s.text_cache
    assert cache.parent == s.out                       # the run's own, never ~/.cache
    env = s.perception_env()
    assert env["PICAR_TEXT_CACHE"] == str(cache)
    assert env["PERCEPTION_SWITCH_FILE"] == str(s.switch_file)
    s.switch_file.write_text("")
    # An encoding left from the last mission (or a bench) is gone before the switch.
    cache.mkdir(parents=True)
    (cache / "old.json").write_text(json.dumps({"model": "mobileclip:blt", "texts": ["plunger"]}))
    s.next_target("plunger")
    assert list(cache.iterdir()) == []
    assert s.switch_file.read_text() == "plunger\n"
    # What the mission's children wrote is its record.
    (cache / "a.json").write_text(json.dumps({"model": "open_clip:RN50:openai",
                                              "texts": ["plunger", "a wall"]}))
    (cache / "b.json").write_text(json.dumps({"model": "mobileclip:blt", "texts": ["plunger"]}))
    (cache / "c.tmp").write_text("{")                    # a write cut short is not one
    (cache / "d.json").write_text("not json")
    assert s.encoded() == ["mobileclip:blt", "open_clip:RN50:openai"]
    s.next_target("box")
    assert s.encoded() == []
    assert s.switch_file.read_text() == "plunger\nbox\n"


def test_the_soak_holds_the_board_past_four_hours_on_a_shared_desktop_lock():
    argv = fmea_run.soak_lock_argv(["x"], "3.135.C soak", 14400 + 7200, False)
    assert argv[2:4] == ["shared", "--no-wait"]
    i = argv.index("--max")
    assert float(argv[i + 1]) > 4 * 3600
    assert argv[-2:] == ["--", "x"]


# ------------------------------------------------------------- the scorer

def proc(pid, cmd, rss_mb, comm="python3"):
    return {"pid": str(pid), "comm": comm, "cmd": cmd, "cgroup": "/user.slice", "Rss": rss_mb * 1024}


def account(t, mem_mb, procs, nv=None):
    return {"t": t, "phase": "m", "meminfo": {"MemAvailable": int(mem_mb * 1024)},
            "nvmap": nv or {}, "procs": procs}


def test_roles():
    assert ss.role(proc(1, "python -m uvicorn robot.server:app --port 10350", 1)) == "robot server"
    assert ss.role(proc(2, "python evaluations/jetson-fmea-3135/truth_body.py 10352", 1)) == "simulator body"
    assert ss.role(proc(3, "python -m sim.fake_lidar --link x", 1)) == "fake lidar"
    assert ss.role(proc(4, "python evaluations/tiered-380/E/standin_cloud.py 8099 x", 1)) == "cloud stand-in"
    assert ss.role(proc(5, "python tools/jetson/perception_loop.py 9", 1)) == "perception (YOLOE + CLIP)"
    assert ss.role(proc(6, "python soak_standin.py --name leak --x", 1)) == "stand-in leak"
    assert ss.role(proc(7, "/usr/lib/systemd/systemd-journald", 1, "systemd-journal")) == "other: systemd-journal"
    assert ss.role(proc(8, "python evaluations/jetson-fmea-3135/soak.py out", 1)) == "harness"


def test_compact_sums_a_roles_processes_and_its_gpu():
    c = ss.compact(account(5.0, 900, [proc(1, "uvicorn sim.sensor_server:app", 100),
                                      proc(2, "uvicorn sim.sensor_server:app", 50)],
                           nv={"1": 2048}))
    assert c["mem_mb"] == 900 and c["roles"]["simulator sensors"] == [150.0, 2.0, 2, 150.0]


def test_slope():
    assert ss.slope_per_h([(0, 100.0), (H, 90.0), (2 * H, 80.0)]) == pytest.approx(-10.0)
    assert ss.slope_per_h([(0, 1.0)]) is None


def write_board(tmp, missions, samples, run=(0.0, 4 * H), oom=(0, 0)):
    b = tmp / "board"
    b.mkdir(parents=True, exist_ok=True)
    (b / "run_start").write_text(f"{run[0]}\n")
    (b / "run_end").write_text(f"{run[1]}\n")
    (b / "oom_start").write_text(f"{oom[0]}\n")
    (b / "oom_end").write_text(f"{oom[1]}\n")
    (b / "missions.jsonl").write_text("".join(json.dumps(m) + "\n" for m in missions))
    (b / "account.jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples))
    return b


def soak_records(n_missions=24, leak_mb_per_h=0.0, mem_drop_mb_per_h=0.0, mem0=900.0,
                 outcome=lambda k: "found"):
    """A 4 h soak: missions back to back, a sample every 60 s; one long-lived
    role (VE) and one per-mission role (robot server) restarted per mission."""
    span = 4 * H / n_missions
    missions, samples = [], []
    for k in range(n_missions):
        t0, t1 = k * span + 30, (k + 1) * span - 10
        missions.append({"k": k, "house": soak.HOUSES[k % 5], "target": "t", "setup_t": k * span,
                         "ready_t": t0 - 5, "t0": t0, "t1": t1, "outcome": outcome(k),
                         "wheel_loop": {"late_ticks": 0, "moving_ticks": 100, "max_dt_s": 0.08},
                         "contacts": 0, "truth_hole_s": 0.05, "truth_hole_all_s": 0.05,
                         "gap_unknown": 0})
    t = 0.0
    while t <= 4 * H:
        h = t / H
        samples.append(account(t, mem0 - mem_drop_mb_per_h * h - (40 if int(t) % 600 == 0 else 0),
                               [proc(1, "python -m service.local_vlm.server", 4000 + leak_mb_per_h * h),
                                proc(2, "uvicorn robot.server:app", 300)]))
        t += 60.0
    return missions, samples


def sysfs_rows(mem=900, oom=0):
    return [{"t": float(t), "mem_mb": mem, "oom_kill": oom} for t in range(0, int(4 * H) + 1, 30)]


def test_a_clean_soak_meets_every_bar(tmp_path):
    missions, samples = soak_records()
    b = write_board(tmp_path, missions, samples)
    s = ss.score(b, sysfs=sysfs_rows())
    assert s["a"]["met"] and s["b"]["met"] and s["c"]["met"] and s["d"]["met"] and s["met"], s
    assert s["b"]["growth"]["local VLM service"]["growth_mb"] == 0
    assert s["a"]["min_mb"] == 860


def test_a_planted_leak_in_one_process_fails_b(tmp_path):
    missions, samples = soak_records(leak_mb_per_h=30.0)    # +90 MB hour 1 -> 4
    s = ss.score(write_board(tmp_path, missions, samples), sysfs=sysfs_rows())
    assert not s["b"]["met"] and s["b"]["over"] == ["local VLM service"]
    assert s["a"]["met"] and s["c"]["met"] and s["d"]["met"] and not s["met"]


def test_a_falling_floor_fails_b_and_one_low_sample_fails_a(tmp_path):
    missions, samples = soak_records(mem_drop_mb_per_h=15.0)
    s = ss.score(write_board(tmp_path, missions, samples), sysfs=sysfs_rows())
    assert not s["b"]["met"] and s["b"]["slope_mb_per_h"] == pytest.approx(-15.0, abs=1.0)
    rows = sysfs_rows()
    rows[100]["mem_mb"] = 499
    s = ss.score(write_board(tmp_path / "x", *soak_records()), sysfs=rows)
    assert not s["a"]["met"] and s["a"]["min_mb"] == 499


def test_an_oom_kill_or_a_late_tick_or_an_unread_loop_fails_c(tmp_path):
    missions, samples = soak_records()
    s = ss.score(write_board(tmp_path / "1", missions, samples, oom=(3, 4)), sysfs=sysfs_rows())
    assert not s["c"]["met"] and s["c"]["oom_kills"] == 1
    missions[5]["wheel_loop"]["late_ticks"] = 2
    s = ss.score(write_board(tmp_path / "2", missions, samples), sysfs=sysfs_rows())
    assert not s["c"]["met"] and s["c"]["late_by_mission"] == {5: 2}
    missions[5]["wheel_loop"]["late_ticks"] = 0
    del missions[7]["wheel_loop"]
    s = ss.score(write_board(tmp_path / "3", missions, samples), sysfs=sysfs_rows())
    assert not s["c"]["met"] and s["c"]["wheel_unread_missions"] == [7]


def test_d_reads_found_share_contacts_and_whole_truth(tmp_path):
    missions, samples = soak_records(n_missions=20, outcome=lambda k: "found" if k > 1 else "exhausted")
    s = ss.score(write_board(tmp_path / "1", missions, samples), sysfs=sysfs_rows())
    assert s["d"]["share"] == 0.9 and s["d"]["met"]
    missions[0]["outcome"] = "found"
    missions[3] = {"k": 3, "house": "complex_house", "target": "t", "setup_t": 1.0,
                   "error": "StackError: the ROS bridge not reached in 180 s"}
    missions[2]["outcome"] = "exhausted"
    s = ss.score(write_board(tmp_path / "2", missions, samples), sysfs=sysfs_rows())
    assert s["d"]["found"] == 17 and not s["d"]["met"]      # the stack that never came up counts
    missions, samples = soak_records()
    missions[4]["contacts"] = 1
    assert not ss.score(write_board(tmp_path / "3", missions, samples), sysfs=sysfs_rows())["d"]["met"]
    missions[4]["contacts"] = 0
    missions[6]["truth_hole_s"] = 0.5
    s = ss.score(write_board(tmp_path / "4", missions, samples), sysfs=sysfs_rows())
    assert not s["d"]["met"] and s["d"]["truth_unjudged_missions"] == [6]


def test_the_compact_form_scores_the_same(tmp_path):
    missions, samples = soak_records(leak_mb_per_h=30.0)
    b = write_board(tmp_path, missions, samples)
    s1 = ss.score(b, sysfs=sysfs_rows())
    ss.write_compact(b)
    (b / "account.jsonl").unlink()
    s2 = ss.score(b, sysfs=sysfs_rows())
    assert s1["b"] == s2["b"] and s1["a"] == s2["a"]


def test_harness_roles_are_reported_not_judged():
    mins = [{"k": k, "t0": t, "t1": t + 100, "n": 3, "mem_min_mb": 900.0,
             "rss_min_mb": {"harness": 10.0 + (300 if t > 3 * H else 0)}} for k, t in
            enumerate([10, 1000, 2000, 3 * H + 10, 3 * H + 1000])]
    b = ss.bar_b(mins, 0.0)
    assert b["growth"]["harness"]["growth_mb"] == 300 and b["over"] == []


def test_a_process_outside_the_stack_is_judged_on_its_anonymous_memory():
    def mins(rss1, anon1, rss4, anon4):
        return [{"k": k, "t0": t, "t1": t + 100, "n": 3, "mem_min_mb": 900.0,
                 "rss_min_mb": {"other: systemd-journal": r},
                 "judged_min_mb": {"other: systemd-journal": a}}
                for k, (t, r, a) in enumerate([(10, rss1, anon1), (3 * H + 10, rss4, anon4)])]
    # its mapped journal grows 200 MB (file-backed): not a leak
    assert ss.bar_b(mins(100, 20, 300, 25), 0.0)["over"] == []
    # its heap grows 60 MB: a leak
    assert ss.bar_b(mins(100, 20, 160, 80), 0.0)["over"] == ["other: systemd-journal"]
    p = proc(1, "/usr/lib/systemd/systemd-journald", 300, "systemd-journal")
    p["Pss_Anon"] = 25 * 1024
    v = ss.compact(account(0, 900, [p]))["roles"]["other: systemd-journal"]
    assert ss.judged_mb("other: systemd-journal", v) == 25 and ss.judged_mb("robot server", v) == 300


def test_dmesg_alone_cannot_vouch_for_no_oom_kill(tmp_path):
    missions, samples = soak_records()
    s = ss.score(write_board(tmp_path / "1", missions, samples, oom=(2, 1)), sysfs=sysfs_rows())
    assert s["c"]["oom_kills_dmesg"] is None and s["c"]["met"]        # vmstat read 0
    s = ss.score(write_board(tmp_path / "2", missions, samples), sysfs=[])
    assert s["c"]["oom_kills_vmstat"] is None and not s["c"]["met"]


def test_a_run_that_ended_early_is_not_met(tmp_path):
    missions, samples = soak_records()
    b = write_board(tmp_path, missions, samples)
    (b / "run_end").unlink()
    s = ss.score(b, sysfs=sysfs_rows())
    assert not s["complete"] and not s["met"]


def test_soak_length_is_bounded_by_the_ssh_timeout():
    with pytest.raises(SystemExit):
        fmea_run.main(["soak", "--seconds", str(6 * 3600)])


def test_run1_on_the_board_scores_as_recorded(tmp_path):
    """3.135.C run 1 (2026-11-02/03, rev dd0a7c9f), re-scored from its
    committed form (memory.jsonl.gz, sysfs.jsonl.gz; no account or truth
    logs): criterion 1 FAILED on (b), (c) and (d)."""
    import shutil
    src = HERE / "board" / "run1"
    out = tmp_path / "run1"
    shutil.copytree(src, out, ignore=shutil.ignore_patterns(
        "account.jsonl", "account.jsonl.gz", "truth.jsonl.gz", "summary.json"))
    assert not (out / "board" / "telemetry" / "sysfs.jsonl").exists()
    assert ss.main([str(out)]) == 0
    s = json.loads((out / "summary.json").read_text())
    assert s["complete"] and s["run_s"] == 14676.7 and not s["met"]
    assert s["a"]["met"] and s["a"]["min_mb"] == 700 and s["a"]["account_min_mb"] == 791.1
    assert s["telemetry"]["mem_available_min_mb"] == 700 and s["telemetry"]["oom_kills"] == 0
    assert not s["b"]["met"] and s["b"]["slope_mb_per_h"] == -21.08
    assert s["b"]["slope_missions"] == 13 and s["b"]["over"] == []
    assert not s["c"]["met"] and s["c"]["oom_kills"] == 0 and s["c"]["late_ticks"] == 2
    assert s["c"]["late_by_mission"] == {"1": 1, "18": 1} and s["c"]["max_dt_s"] == 0.104
    assert s["c"]["moving_ticks"] == 201841
    assert not s["d"]["met"] and (s["d"]["found"], s["d"]["missions"]) == (10, 19)
    assert s["d"]["contacts"] == 0 and s["d"]["truth_unjudged_missions"] == []
    assert s["d"]["outcomes"] == {"found": 10, "out_of_budget": 8, "exhausted": 1}
    assert s["d"]["by_house"] == {"home_first_floor": "4/4", "complex_house": "1/4",
                                  "procthor:val:0": "3/4", "procthor:val:2": "0/4",
                                  "procthor:val:3": "2/3"}
