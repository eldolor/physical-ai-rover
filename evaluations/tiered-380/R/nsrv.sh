#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# 3.80.R: the bench's "server": native_server.py under $NSRV_PY (default the
# repo's .venv), llama-server's argv passed through (only -m and --port read).
HERE="$(cd "$(dirname "$0")" && pwd)"
echo 1000 > /proc/self/oom_score_adj 2>/dev/null
exec "${NSRV_PY:-$HERE/../../../.venv/bin/python}" "$HERE/native_server.py" "$@"
