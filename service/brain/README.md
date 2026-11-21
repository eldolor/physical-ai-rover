# service/brain: the brain's ECS image (DELETED STACK, kept for B5)

**Not deployed.** This is the container image of the `brain` ECS stack,
deleted 2026-09-05 (`docs/plans/PLAN-aws-cost-redesign.md`; the ECR images
went with it, so a rollback is a rebuild). `CLAUDE.md` section 4 keeps it
for B5, the deployment phase (`docs/plans/PLAN-brain-relocation.md`). The
Dockerfile calls itself an interim cloud deployment: the brain's home is
the robot's own computer, and today it runs locally, reached through
`service/tunnel/`. The brain cannot be deployed to Lambda (it loads YOLOE
and CLIP).

| File | Purpose |
|---|---|
| `Dockerfile` | `python:3.12-slim`; **build context is the repo root**; copies `brain/`, `control/`, `robot/` and `config/` (and deliberately not `sim/`: the brain must never import the simulator or a backend); serves `control.brain_server:app` on :8001 |
| `requirements.txt` | The image's Python dependencies |

```bash
docker build -f service/brain/Dockerfile -t vision-picar-brain .   # from the repo root; the Dockerfile's own comment
```

Rule 5 of `CLAUDE.md` section 2 (the brain imports no backend) is why the
image has no `sim/`; `tests/test_brain_server.py` checks it. See
`docs/engineering/operations/ENGINEERING.md` ("Deleted 2026-09-05") and
`docs/guides/AGENT-HARNESS.md` for the brain itself.
