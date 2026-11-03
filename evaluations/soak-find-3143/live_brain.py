# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/soak-find-3143/live_brain.py

3.143 step B: the brain server with ladder.py's local tier installed in its
process (`truth`: TruthLocal, A2's; `no`: NoLocal, A3n's), then served by
uvicorn exactly as `uvicorn control.brain_server:app <args>`. The brain's
own config still says which tier it uses: `LOCAL_VLM_URL=stub://<arm>` makes
the tier "configured", and only the client it builds is replaced.

    python evaluations/soak-find-3143/live_brain.py <truth|no> --port N [uvicorn args]
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
sys.path.insert(0, str(HERE))

import uvicorn  # noqa: E402

from control import brain_server  # noqa: E402
from ladder import NoLocal, TruthLocal  # noqa: E402

LOCALS = {"truth": lambda target, *a, **k: TruthLocal(target),
          "no": lambda *a, **k: NoLocal()}


def main(argv):
    arm, args = argv[0], argv[1:]
    brain_server._local_identity_fn = LOCALS[arm]
    sys.argv = ["uvicorn", "control.brain_server:app", *args]
    uvicorn.main()


if __name__ == "__main__":
    main(sys.argv[1:])
