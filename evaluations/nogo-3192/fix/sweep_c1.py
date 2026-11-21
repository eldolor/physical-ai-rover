# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3192/fix/sweep_c1.py -- 3.192.B amendment 3's runs:
`evaluations/nogo-3192/sweep2.py` (3.161's harness plus the chooser and F10's
goal ends) imported unchanged, plus C7's count: each row gets `reseen`, the
look-again rounds explore logged (`reseen` events), and `reseen_places`, the
places they held. On `dev` (no such event) both read 0.

    SWEEP_WORKERS=4 python evaluations/nogo-3192/fix/sweep_c1.py find|survey OUT.json \\
        [--chooser rooms]
"""

import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import sweep2                                            # noqa: E402
import brain.explore as ex                               # noqa: E402

_RESEEN: list = []
_prev_event = ex.ExploreAgent._event


def _event(self, name, pose=None, **fields):
    if name == "reseen":
        _RESEEN.append(len(fields.get("places") or ()))
    return _prev_event(self, name, pose, **fields)


ex.ExploreAgent._event = _event
_run2 = sweep2.run


def run(*a, **kw):
    _RESEEN.clear()
    row = _run2(*a, **kw)
    row["reseen"] = len(_RESEEN)
    row["reseen_places"] = sum(_RESEEN)
    return row


def _apply():
    sweep2._apply()
    sweep2.sw.run = run


_apply()   # in a pool worker, the same patches

if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--chooser" in argv:
        i = argv.index("--chooser")
        os.environ["SWEEP2_CHOOSER"] = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    _apply()
    sweep2.sw.main(argv)
