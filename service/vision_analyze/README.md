# service/vision_analyze: the cloud vision service

The FastAPI app behind the deployed `/navigate`, `/analyze`, `/describe`,
`/room`, `/room-prior` and `/guidance` routes. It calls **Amazon Bedrock's
Converse API** (IAM credentials, no API key). In production it runs as a
Lambda through `service/lambda/vision_handler.py`; the Dockerfile is for
local and container runs.

| File | Purpose |
|---|---|
| `app.py` | The FastAPI app: routes `GET /health`, `POST /analyze`, `/describe`, `/navigate`, `/room`, `/room-prior`, `/guidance`, `GET /navigate/models` |
| `vision_core.py` | Prompts, the Bedrock calls and the model and region tables |
| `rooms_core.py` | Room recognition |
| `Dockerfile` | `python:3.12-slim`; build context is this directory; copies `app.py`, `vision_core.py`, `rooms_core.py` flat into `/app`; serves on :8080 |
| `requirements.txt` | The service's own list; `service/lambda/build.sh` installs it too |
| `tests/` | The service's own suite |

**`vision_core.py` and `rooms_core.py` are deliberate copies** (of the
logic in `brain/vision.py` and `brain/rooms.py`, which calls Anthropic
directly where this calls Bedrock), so this directory stays a
self-contained build context. Mirror a prompt or room-logic change by hand
(`CLAUDE.md` section 6).

## Run

```bash
pytest service/vision_analyze/tests/ -q        # its own suite, not `tests/`
cd service/vision_analyze && uvicorn app:app --port 8080   # flat imports: run from here
docker build -t vision-picar-vision service/vision_analyze
```

## Environment (read from the code)

| Variable | Effect |
|---|---|
| `APP_SHARED_SECRET` | If set, every call needs a matching `x-app-secret` header (constant-time compare, else 401). Unset: no check, for local runs only. Under Lambda the template passes `VISION_SHARED_SECRET` |
| `ALLOWED_ORIGINS` | Comma-separated CORS origins; defaults to two localhost origins |
| `ENV_LABEL` | A label `/health` echoes for environment banners |
| `BEDROCK_MODEL_ID`, `BEDROCK_ANALYZE_MODEL_ID`, `BEDROCK_NAVIGATE_MODEL_ID`, `BEDROCK_GUIDANCE_MODEL_ID`, `BEDROCK_ROOM_MODEL_ID`, `BEDROCK_NAVIGATE_MODEL_CHOICES`, `BEDROCK_MODEL_REGIONS`, `NAVIGATE_PROMPT_VARIANT` | Model ids, the `/navigate` A/B set, per-model region pins and the prompt variant; defaults are in `vision_core.py`. **Never guess a Bedrock model id**, and keep the code defaults in step with `cloudformation/serverless.yaml` |

Some Fable models answer only from `us-east-1` on this account, so
`vision_core.MODEL_REGIONS` pins them there; re-confirm a pinned model from
the deployed region.

## Where the rest is written

`docs/engineering/cloud-vision/ENGINEERING.md` (routes, models, regions,
CORS); `docs/cloud-vision/` for the architecture.
