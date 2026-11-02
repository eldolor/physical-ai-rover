#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# SessionStart hook: put the plan's step board into the session's context,
# so it claims a step before working (docs/guides/PARALLEL-SESSIONS.md,
# rules 8 and 12). Since 3.139 claims live on origin (`claim/<step>`), so a
# cloud session sees the same claims as every other machine.
cd "$CLAUDE_PROJECT_DIR" 2>/dev/null || exit 0
[ -f tools/plan_claims.py ] && python3 tools/plan_claims.py hook-start 2>/dev/null
exit 0
