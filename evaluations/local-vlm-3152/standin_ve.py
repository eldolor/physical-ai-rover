# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/standin_ve.py

3.152 criterion 9's stand-in VE: the local VLM service's own app
(`service/local_vlm/server.py` `create_app`) over its test suite's fake
driver (`tests/test_local_vlm.py` `make_driver`), served on a port, so the
brain reaches it exactly as it reaches the board's VE. Killing this process
is VE stopping (the port refuses); starting it again is VE coming back,
`/health` answering `ready: false` while the fake driver loads.

    python evaluations/local-vlm-3152/standin_ve.py <port> <workdir>
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import uvicorn  # noqa: E402

from service.local_vlm.server import create_app  # noqa: E402
from tests.test_local_vlm import make_driver  # noqa: E402


def main():
    port, workdir = int(sys.argv[1]), Path(sys.argv[2])
    workdir.mkdir(parents=True, exist_ok=True)
    os.environ.pop("APP_SHARED_SECRET", None)
    app = create_app(lambda: make_driver(workdir), on_death=lambda: os._exit(3),
                     ready_timeout_s=30)
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")


if __name__ == "__main__":
    main()
