# service/twin: the robot-server ECS image (DELETED STACK, kept for B5)

**Not deployed.** This is the container image of the `twin` ECS stack,
deleted 2026-09-05 (`docs/plans/PLAN-aws-cost-redesign.md`; the ECR images
went with it, so a rollback is a rebuild). `CLAUDE.md` section 4 keeps it
for B5, the deployment phase (`docs/plans/PLAN-brain-relocation.md`), and
`docs/engineering/operations/ENGINEERING.md` names `service/twin/` as the
one kept for it. **The deployed twin is not this image**: it is static
files on S3 behind CloudFront (`service/static/`), and the robot server
runs locally (`service/tunnel/`).

| File | Purpose |
|---|---|
| `Dockerfile` | `python:3.12-slim`; **build context is the repo root**; copies `robot/`, `sim/`, `config/`, `web-twin/` and `web-shared/` (the real packages, not dependency-light copies); serves `robot.server:app` on :8000 |
| `requirements.txt` | The image's Python dependencies |

```bash
docker build -f service/twin/Dockerfile -t vision-picar-twin .   # from the repo root; the Dockerfile's own comment
```

It is the same server as the local one, so every movement still goes
through `robot/safety.py` (`docs/engineering/safety/ENGINEERING.md`).
