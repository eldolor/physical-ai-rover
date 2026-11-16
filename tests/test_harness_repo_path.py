# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.186 H1-H2: a harness loads the robot code of the checkout it names, not
the launch directory's.

Each case starts a fresh interpreter in a directory holding a decoy
`brain/` package, puts this checkout on the path the way the sweeps do
(`REPO` first), imports an exposed harness and prints where `brain` came
from. Before 3.186 `evaluations/explore-365/harness.py` put `"."` first, so
the decoy won."""
import os
import subprocess
import sys

import pytest

REPO = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))

# (directory to put on the path, module to import), as the sweeps do it
CASES = {
    "harness": ("evaluations/explore-365", "harness"),
    "oracle": ("evaluations/goal-tap-388", "oracle"),
    "train": ("evaluations/goal-learn-396", "train"),
}

SNIPPET = """
import os, sys
repo, sub, mod = sys.argv[1:4]
sys.path.insert(0, os.path.join(repo, sub))
sys.path.insert(0, repo)
__import__(mod)
import brain, harness
print("BRAIN=" + os.path.realpath(brain.__file__))
print("RECORDED=" + str(getattr(harness, "BRAIN", None)))
"""


def _decoy(tmp_path):
    (tmp_path / "brain").mkdir()
    (tmp_path / "brain" / "__init__.py").write_text("DECOY = True\n")
    return tmp_path


def _run(cwd, sub, mod):
    return subprocess.run([sys.executable, "-c", SNIPPET, REPO, sub, mod], cwd=cwd,
                          capture_output=True, text=True, timeout=300)


@pytest.mark.parametrize("name", sorted(CASES))
def test_brain_comes_from_repo_not_launch_dir(tmp_path, name):
    sub, mod = CASES[name]
    p = _run(_decoy(tmp_path), sub, mod)
    assert p.returncode == 0, p.stderr[-2000:]
    out = dict(line.split("=", 1) for line in p.stdout.splitlines() if "=" in line)
    want = os.path.join(REPO, "brain")
    assert os.path.dirname(out["BRAIN"]) == want
    # H2: the harness records what it loaded, in the module and on stderr
    assert out["RECORDED"] == want
    assert f"brain: {want}" in p.stderr


def test_a_brain_from_outside_the_checkout_is_refused(tmp_path):
    """H2: a brain already imported from elsewhere is refused, not run."""
    decoy = _decoy(tmp_path)
    code = ("import sys; sys.path.insert(0, sys.argv[2]); import brain; "
            "sys.path.insert(0, sys.argv[1]); import harness")
    p = subprocess.run([sys.executable, "-c", code,
                        os.path.join(REPO, "evaluations", "explore-365"), str(decoy)],
                       cwd=REPO, capture_output=True, text=True, timeout=300)
    assert p.returncode != 0
    assert "brain is" in p.stderr and "not under" in p.stderr


def test_a_decoy_earlier_on_the_path_loses(tmp_path):
    """H1: a launch directory (or any entry) put first on the path before the
    harness is imported does not supply its robot code."""
    decoy = _decoy(tmp_path)
    code = ("import os, sys; sys.path.insert(0, sys.argv[1]); sys.path.insert(0, sys.argv[2]); "
            "import harness, brain; print(os.path.realpath(brain.__file__))")
    p = subprocess.run([sys.executable, "-c", code,
                        os.path.join(REPO, "evaluations", "explore-365"), str(decoy)],
                       cwd=decoy, capture_output=True, text=True, timeout=300)
    assert p.returncode == 0, p.stderr[-2000:]
    assert os.path.dirname(p.stdout.strip()) == os.path.join(REPO, "brain")


@pytest.mark.parametrize("name,commit", [("d4-dev-a0447516", "a0447516"),
                                         ("c7-dev-73a77251", "73a77251")])
def test_the_two_undetermined_dev_arms_are_confirmed(name, commit):
    """3.186.B: 3.160.D's D4 and 3.116.B's criterion 7, their `dev` arms re-run
    from inside a clean worktree at the recorded commit: 200 of 200 rows equal
    the committed ones (outcome, goals, metres)."""
    import json
    d = json.load(open(os.path.join(REPO, "evaluations", "harness-repo-3186", name + ".json")))
    assert d["repo_commit"] == commit
    assert d["brain"].startswith(f"wt-{commit}/brain")
    assert (d["pairs"], d["matched"], d["differ"]) == (200, 200, [])
    assert len(d["rows"]) == 200
