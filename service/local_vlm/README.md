# service/local_vlm: the local VLM service

A small vision-language model resident on the Jetson's GPU, behind an HTTP
service on the board's loopback address (plan section 3.80.B). The brain
asks it the identity question ("is this the target?") before it asks the
cloud. It is its own process so that, when the board runs out of memory,
the kernel kills it and nothing else; the brain then reads it as absent and
asks the cloud.

**The two specs are the reference; this page only points at them.**

* What it is, why it is a separate service, which questions go to it and
  which to the cloud, and what happens when it is down:
  `docs/perception/ARCHITECTURE.md`, "The local VLM: a separate service,
  asked before the cloud".
* The routes, the environment variables, the constants, the procedures and
  the tests: `docs/engineering/perception/ENGINEERING.md` (search it for
  "The local VLM service").

## What is here

| File | What it is |
|---|---|
| `server.py` | The FastAPI app: `GET /health` (open), `POST /identity` and `POST /direction` (behind `APP_SHARED_SECRET` when it is set). It exits with status 3 when its driver dies, so systemd decides on a restart. |
| `driver.py` | `EdgeDriver`: one resident `edgellm_identity` process (TensorRT Edge-LLM), asked one request at a time. |

The brain's client is `brain/local_vlm.py`. The driver's source and build
are in `tools/jetson/` (`edgellm_identity.cpp`,
`build_edgellm_identity.sh`), with the unit
(`systemd/picar-local-vlm.service`), the bench (`bench_local_vlm.sh`) and
the script benches use to hold the service off (`vlm_hold.sh`).

## Running it

It runs on the Jetson only: the driver binary and the engines exist only
there. On the board systemd runs it as `picar-local-vlm`, enabled at boot;
installing and updating it are `tools/jetson/README.md` section 5.

```bash
systemctl is-active picar-local-vlm
curl -s localhost:8005/health          # "ready": true about 40 s after a start
```

By hand on the board, from the repo root (stop the unit first):

```bash
LOCAL_VLM_ENGINES=~/bench-378/ws91/q25VE/engines \
LOCAL_VLM_EDGELLM=~/bench-378/ell91 \
    python -m service.local_vlm.server
```

`LOCAL_VLM_ENGINES` and `LOCAL_VLM_EDGELLM` are required;
`LOCAL_VLM_DRIVER_LOG` (the driver's stderr, default discarded) and
`LOCAL_VLM_PORT` (default 8005) are optional. The brain uses the service
only when `LOCAL_VLM_URL` (or `brain.tier_local_vlm_url`) names it; the
default is empty, which is no local tier.

Off the board, `tests/test_local_vlm.py` runs this server against a fake
driver (`tests/fake_edgellm_driver.py`), and
`python -m tests.demo_local_tier` runs one simulator mission with it
answering. No test loads the model.
