# service/lambda: the two Lambda functions and the script that builds them

The serverless deployment (`cloudformation/serverless.yaml`) is two arm64
Python 3.12 Lambda functions behind API Gateway. This directory holds their
entry points and requirements, and `build.sh`, which builds both zips and
uploads them to the deploy bucket.

| File | Purpose |
|---|---|
| `build.sh` | Builds `vision` and `walks` zips and uploads each to `s3://<bucket>/lambda/<name>-<UTC stamp>.zip`; prints the `aws cloudformation deploy` command to run. Wheels are fetched `--platform manylinux2014_aarch64 --only-binary=:all:` so a Mac build cannot ship the wrong architecture. Work directory: `.build/lambda/` |
| `vision_handler.py` | Entry point (`vision_handler.handler`, a Mangum wrapper) for `service/vision_analyze/app.py`. Imports `app` flat, so `build.sh` puts `app.py`, `vision_core.py` and `rooms_core.py` at the zip root |
| `walks_handler.py` | Entry point (`walks_handler.handler`) for `control/admin_server.py`: the recorded-walk read, score and replay routes plus the write routes. Refuses to start with `RECORDING_BACKEND=s3` and no `RECORDING_BUCKET` |
| `requirements-vision.txt` | What Lambda adds to `service/vision_analyze/requirements.txt` (`mangum`); `boto3` comes from the runtime image |
| `requirements-walks.txt` | The curated third-party list for `control/`; `tests/test_lambda_packaging.py` checks it against the real imports |

## Run

```bash
export VISION_SHARED_SECRET="$VISION_SECRET" WALKS_SHARED_SECRET="$WALKS_SECRET"   # from the secrets files
bash service/lambda/build.sh <deploy-bucket> [region]    # region defaults to us-east-2
```

`build.sh` exits 2 before building if either secret variable is empty: the
printed deploy command expands them, and an empty one would deploy that
function with no auth. The bucket is the deploy stack's `DeployBucketName`
output. **It uploads to S3, and the deploy it prints is the user's go** (cloud
sessions can deploy the twin only, `CLAUDE.md` section 3b).

## What ships

`build.sh` copies **named files**: the vision zip gets the three
`service/vision_analyze/` modules and `vision_handler.py`; the walks zip
gets `control/*.py`, the Lab console's page and scripts, `web-shared/ui.css`,
`config/robot.yaml` and `walks_handler.py`. Nothing here is copied by
wildcard, so a README in this directory is not shipped.

Do not call these functions through a Function URL: on this account
Lambda resource-based policies do not grant invocation, which is why an API
Gateway integration with an explicit credentials role is used
(`vision_handler.py`'s docstring; `PLAN-aws-cost-redesign.md` section 6).

## Where the rest is written

`docs/engineering/operations/ENGINEERING.md` (build and publish scripts, the
deploy command); `docs/engineering/cloud-vision/ENGINEERING.md`.
