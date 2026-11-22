# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.212: a live run's logs kept beside its rows (`tests/demo_explore.keep_logs`).
The bars are docs/plans/ros-alignment/3.212-keep-live-logs.md's A1-A5."""

import ast
import gzip
import os
import subprocess
import sys
from pathlib import Path

import pytest

from tests import demo_explore as de

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def _fresh(monkeypatch):
    monkeypatch.setattr(de, "RUN_DIRS", [])
    monkeypatch.setattr(de, "_KEPT", set())
    monkeypatch.setattr(de, "_ATEXIT", [])
    monkeypatch.setattr(de, "RUN_DIR", None)


def _run(tmp_path, monkeypatch, name="logs"):
    monkeypatch.setattr(de, "LOGDIR", str(tmp_path / name))
    run = Path(de._new_run_dir())
    (run / "ros-s1.log").write_bytes(b"[planner_server] failed\n" * 1000)
    (run / "explore_robot.log").write_text("keepout: marked (1.00, 2.00)\n")
    (run / "brain-s2.log").write_bytes(os.urandom(4096))
    (run / "empty.log").write_bytes(b"")
    return run


def _plant(run):
    """A2's decoys: a map and an inventory beside the logs."""
    for sub in ("maps-complex_house", "inventory-complex_house"):
        (run / sub).mkdir()
        (run / sub / "map.log").write_text("a floor plan")
        (run / sub / "inventory.json").write_text("{}")
    (run / "map.pgm").write_bytes(b"P5 floor plan")
    (run / "status.json").write_text("{}")
    (run / "slam.yaml").write_text("x: 1")
    os.symlink(run / "maps-complex_house" / "map.log", run / "evil.log")


def _gunzip_tree(dest):
    return {p.relative_to(dest).as_posix(): gzip.decompress(p.read_bytes())
            for p in Path(dest).rglob("*") if p.is_file()}


def test_a1_every_log_kept_byte_identical(tmp_path, monkeypatch):
    runs = [_run(tmp_path, monkeypatch), _run(tmp_path, monkeypatch)]   # two runs: run-..., run-...-2
    dest = tmp_path / "rows-logs"
    out = de.keep_logs(dest)
    assert out["lost"] == []
    got = _gunzip_tree(dest)
    want = {f"{r.name}/{p.name}.gz": p.read_bytes() for r in runs for p in r.glob("*.log")}
    assert got == want and len(want) == 8


def test_a1_kept_on_an_exception_inside_the_harness_try(tmp_path, monkeypatch):
    run = _run(tmp_path, monkeypatch)
    dest = tmp_path / "rows-logs"
    with pytest.raises(RuntimeError, match="stack did not come up"):
        try:
            raise RuntimeError("stack did not come up")
        finally:
            de.keep_logs(dest)
    assert set(_gunzip_tree(dest)) == {f"{run.name}/{p.name}.gz" for p in run.glob("*.log")}


def test_a1_live_harness_keeps_logs_in_its_outer_finally():
    """`live.run` cannot run without Docker here; its outermost try's
    `finally` must call `de.keep_logs(...)` after the container's last log
    is saved and the container is gone."""
    tree = ast.parse((ROOT / "evaluations/find-last-seen-370/live.py").read_text())
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run")
    outer = [n for n in run.body if isinstance(n, ast.Try)]
    assert len(outer) == 1 and outer[0].finalbody
    calls = [ast.unparse(c.func) for st in outer[0].finalbody for c in ast.walk(st)
             if isinstance(c, ast.Call)]
    assert "de.keep_logs" in calls
    assert calls.index("de.keep_logs") > calls.index("save_logs")
    assert calls.index("de.keep_logs") > calls.index("de._kill_ports")


def test_a2_nothing_but_top_level_regular_logs(tmp_path, monkeypatch):
    run = _run(tmp_path, monkeypatch)
    _plant(run)
    dest = tmp_path / "rows-logs"
    de.keep_logs(dest)
    got = set(_gunzip_tree(dest))
    assert got == {f"{run.name}/{n}.gz" for n in
                   ("ros-s1.log", "explore_robot.log", "brain-s2.log", "empty.log")}
    assert not any("map" in g or "inventory" in g or "evil" in g for g in got)


def test_a3_an_unreadable_file_is_reported_not_raised(tmp_path, monkeypatch, capsys):
    run = _run(tmp_path, monkeypatch)
    bad = run / "ros-s1.log"
    real_open = os.open

    def deny(path, *a, **k):          # as root a mode-000 file still reads
        if str(path) == str(bad):
            raise PermissionError(13, "Permission denied", str(path))
        return real_open(path, *a, **k)
    monkeypatch.setattr(de.os, "open", deny)
    out = de.keep_logs(tmp_path / "rows-logs")
    assert [src for src, _ in out["lost"]] == [str(bad)]
    assert str(bad) in capsys.readouterr().err
    assert not (tmp_path / "rows-logs" / run.name / "ros-s1.log.gz").exists()
    assert len(out["kept"]) == 3


def test_a3_a_failing_copy_is_reported_not_raised(tmp_path, monkeypatch, capsys):
    run = _run(tmp_path, monkeypatch)

    def boom(*a, **k):
        raise OSError("disk full")
    monkeypatch.setattr(de.shutil, "copyfileobj", boom)
    out = de.keep_logs(tmp_path / "rows-logs")
    assert out["kept"] == [] and len(out["lost"]) == 4
    assert "disk full" in capsys.readouterr().err
    assert list((tmp_path / "rows-logs" / run.name).iterdir()) == []   # no half copies


def test_a3_an_unwritable_destination_is_reported_not_raised(tmp_path, monkeypatch, capsys):
    _run(tmp_path, monkeypatch)
    blocker = tmp_path / "rows-logs"
    blocker.write_text("a file where the directory would go")
    out = de.keep_logs(blocker)
    assert out["kept"] == [] and len(out["lost"]) == 4
    assert "not kept" in capsys.readouterr().err


def test_a3_never_replaces_the_runs_own_exception(tmp_path, monkeypatch):
    monkeypatch.setattr(de, "RUN_DIRS", None)      # even a broken book: no raise
    with pytest.raises(KeyError, match="the run's own"):
        try:
            raise KeyError("the run's own")
        finally:
            de.keep_logs(tmp_path / "rows-logs")


def test_a4_never_overwrites(tmp_path, monkeypatch):
    run = _run(tmp_path, monkeypatch)
    dest = tmp_path / "rows-logs"
    (dest / run.name).mkdir(parents=True)
    (dest / run.name / "ros-s1.log.gz").write_bytes(b"an earlier run's")
    de.keep_logs(dest)
    assert (dest / run.name / "ros-s1.log.gz").read_bytes() == b"an earlier run's"
    assert len(list((dest / f"{run.name}-2").iterdir())) == 4


def test_a4_a_second_keep_to_the_same_dest_copies_nothing_again(tmp_path, monkeypatch):
    run = _run(tmp_path, monkeypatch)
    dest = tmp_path / "rows-logs"
    de.keep_logs(dest)
    assert de.keep_logs(dest) == {"kept": [], "lost": []}
    assert [p.name for p in dest.iterdir()] == [run.name]


def test_a5_unset_registers_nothing(tmp_path, monkeypatch):
    monkeypatch.delenv("PICAR_KEEP_LOGS", raising=False)
    _run(tmp_path, monkeypatch)
    assert de._ATEXIT == []


def _child(tmp_path, keep):
    code = ("import os\n"
            "from tests import demo_explore as de\n"
            "r = de._new_run_dir()\n"
            "open(os.path.join(r, 'ros-end.log'), 'w').write('nav2 aborted')\n"
            "os.makedirs(os.path.join(r, 'maps-x'))\n"
            "raise SystemExit(3)\n")
    env = dict(os.environ, EXPLORE_LOGDIR=str(tmp_path / "logdir"))
    env.pop("PICAR_KEEP_LOGS", None)
    if keep:
        env["PICAR_KEEP_LOGS"] = str(keep)
    return subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=120)


def test_a5_set_keeps_at_exit(tmp_path):
    keep = tmp_path / "kept"
    p = _child(tmp_path, keep)
    assert p.returncode == 3, p.stderr
    got = _gunzip_tree(keep)
    assert len(got) == 1 and list(got.values()) == [b"nav2 aborted"]
    assert next(iter(got)).endswith("/ros-end.log.gz")


def test_a5_unset_writes_nothing_at_exit(tmp_path):
    p = _child(tmp_path, None)
    assert p.returncode == 3, p.stderr
    assert sorted(x.name for x in tmp_path.iterdir()) == ["logdir"]
