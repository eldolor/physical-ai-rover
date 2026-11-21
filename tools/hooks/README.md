# tools/hooks: the git hooks every checkout uses

Two POSIX shell hooks that enforce the parallel-session rules in git
itself, so they bind a person and a session alike. Install once per clone
(a Claude Code cloud session does it at start-up through
`.claude/hooks/session-start.sh`):

```bash
git config core.hooksPath tools/hooks
```

`core.hooksPath` is shared by every worktree of the clone.

| Hook | Fires on | What it enforces | Rule |
|---|---|---|---|
| `pre-commit` | every `git commit` | Runs `tools/plan_claims.py hook-commit`: **refuses** a commit on a `plan/3.N-*` branch unless this Claude Code session holds a claim in section 3.N (a merge passes), and **warns** when a staged file lies inside another live session's claim, which catches edits made through Bash that the edit hook cannot see. Does nothing in a worktree whose branch predates `tools/plan_claims.py` | `docs/guides/PARALLEL-SESSIONS.md` rules 8 and 12 |
| `pre-push` | every `git push` | Only a push to `refs/heads/dev` is gated; `refs/heads/claim/*` passes untouched. Runs `tools/plan_section.py gate <base> <sha>`: a docs-only push runs the spec lint, anything else the fast tier unless `plan_section.py ready` recorded a pass for that commit; the gate first refuses a change to a plan section's file or an open step's Touches without the pusher's claim. Refuses to delete `dev` | rules 6, 8 and 12 |

**Overrides are the user's, never a session's:** `PICAR_CLAIM_BYPASS=1`
(skips the claim check) and `PICAR_SKIP_GATE=1` (skips the merge gate,
printed to stderr when used). If a hook refuses, stop and report; do not
work around it.

`pre-push` uses `<repo root>/.venv/bin/python` when it exists, else
`python3`.

Read `docs/guides/PARALLEL-SESSIONS.md` rules 6, 8 and 12 for the rules
themselves; this file does not restate them. The Claude Code hooks (Bash
and edit guards) are a different set, in `.claude/hooks/`.
