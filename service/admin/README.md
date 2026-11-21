# service/admin: the admin console's ECS image (DELETED STACK, kept for B5)

**Not deployed.** This is the container image of the `admin` ECS stack,
deleted 2026-09-05 (`docs/plans/PLAN-aws-cost-redesign.md`: nine stacks
deleted, only `recordings-s3` and `serverless` remain; the ECR images went
with them, so a rollback is a rebuild from this Dockerfile). `CLAUDE.md`
section 4 keeps it for B5, the deployment phase
(`docs/plans/PLAN-brain-relocation.md`). The same `control/admin_server.py`
now runs as a Lambda (`service/lambda/walks_handler.py`); nothing builds or
ships this image today.

| File | Purpose |
|---|---|
| `Dockerfile` | `python:3.12-slim`; **build context is the repo root**; copies `control/`, `config/` and `web-shared/`; serves `control.admin_server:app` on :8002. It never imports `brain_server`, `mission_runner`, `remote_robot` or `drills`: no `robot/`, no `brain/` |
| `requirements.txt` | `fastapi`, `uvicorn[standard]`, `pyyaml`, `boto3` (the scorecard's judge tier calls Bedrock directly) |

```bash
docker build -f service/admin/Dockerfile -t vision-picar-admin .   # from the repo root; the Dockerfile's own comment
```

The live equivalent is described
in `service/lambda/README.md`; the history is in
`docs/engineering/operations/ENGINEERING.md` ("Deleted 2026-09-05").
