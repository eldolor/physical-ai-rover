# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/request-3176/score_parser.py -- 3.176.B's P1-P4: the rule parser
(brain/request.py) on the pinned request set, both splits, written to
parser_score.json. The test split's first score is the result (3.176.A); a
parser changed after it is scored again on dev only.

    python evaluations/request-3176/score_parser.py OUT.json
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from tests.test_request import _rows, score                   # noqa: E402

if __name__ == "__main__":
    rows = _rows()
    rev = subprocess.run(["git", "log", "-1", "--format=%h", "--", "brain/request.py"],
                         cwd=REPO, capture_output=True, text=True).stdout.strip()
    # The test split's first score is kept as recorded (scored once); only
    # dev is written again.
    out = json.load(open(sys.argv[1])) if os.path.exists(sys.argv[1]) else {
        "test": {"score": score([r for r in rows if r["split"] == "test"]), "parser_commit": rev}}
    out["dev"] = {"score": score([r for r in rows if r["split"] == "dev"]), "parser": rev}
    json.dump(out, open(sys.argv[1], "w"), indent=1)
    print(json.dumps(out))
