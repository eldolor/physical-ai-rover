# service/tunnel: the local robot and brain, reachable from the deployed twin

The brain cannot be deployed (it loads YOLOE and CLIP) and the robot runs
locally, so the twin served from CloudFront reaches them through **one ngrok
domain**, split by path behind a small proxy: `/brain/*` goes to the brain
(`ROUTE_PREFIX=/brain`), `/vision/*` to the deployed vision service,
everything else to the robot server. These scripts run that stack.

**Always set the shared secret.** `run.sh` reads `LOCAL_SECRET` from
`~/.vision-picar-local-secrets` (mode 600) and refuses to start if it is
empty or shorter than 16 characters: the servers are on the public internet
through the tunnel, and `require_secret()` is inert without
`APP_SHARED_SECRET` (`CLAUDE.md` section 6).

| File | Purpose |
|---|---|
| `run.sh` | Sources the secrets files, exports the variables below, sets `WORLD_MODE`, then starts the robot server (:8000, memory-locked through `robot.locked_serve`), the brain (:8001, `ROUTE_PREFIX=/brain`) and the proxy (:8080), and stops all of them if any one ends. Under `SIM_MOTOR_BOARD=fake` it first starts the simulated body (:8002) and sensor programs (:8003, :8004), and with `SIM_LIDAR=fake` the fake D500. Needs the repo's `.venv`. On the Jetson it is the `picar-stack` unit |
| `restart.sh` | Stops whatever holds :8000, :8001 and :8080 (graceful, then `kill -9` after 5 s), starts `run.sh` detached, and **reports OK only when every server answers `/health` with this checkout's git revision**. Refuses while the `picar-stack` unit is active (use `systemctl` there). Leaves ngrok alone and warns if it is not running |
| `proxy.py` | The FastAPI catch-all on :8080 that ngrok points at: `/brain/*` to the brain, `/vision/*` to the deployed vision service (answers `OPTIONS` itself), the rest to the robot |

## Run

```bash
bash service/tunnel/run.sh                      # the grid-world simulator (Sim tab)
ROBOT_MODE=teleop bash service/tunnel/run.sh    # a phone on the rig
bash service/tunnel/restart.sh                  # restart, and prove the new code answers
ngrok start picar                               # in another terminal; point it at :8080
```

Then put the ngrok URL in the twin's Settings: robot `https://<domain>`,
brain `https://<domain>/brain`, secret `LOCAL_SECRET`. Restarting kills a
mission in flight; the phone reports a bare "Load failed".

## Variables (read from the scripts)

| Variable | Default | Effect |
|---|---|---|
| `LOCAL_SECRET` (from `~/.vision-picar-local-secrets`) | none; required, 16+ characters | Exported as `APP_SHARED_SECRET` (gates both servers) and `ROBOT_SHARED_SECRET` (the brain calling the robot) |
| `VISION_SECRET` (from the same file) | empty | Exported as `VISION_SHARED_SECRET`. Empty: the cloud answers 401 and the brain runs offline |
| `WALKS_SECRET` (from `~/.vision-picar-serverless-secrets`, optional) | empty | Exported as `METRICS_SECRET` |
| `VISION_URL` | a hard-coded CloudFront URL | The deployed vision service. `METRICS_URL` defaults to it |
| `INVENTORY_BUCKET` | looked up from the recordings stack's CloudFormation export; if the lookup fails it is left unset | Where each mission's object inventory goes. Set but empty means local only |
| `ROBOT_MODE` | `sim` | `sim`, `teleop` or `hardware` |
| `WORLD_MODE` | `sim` when `ROBOT_MODE=sim`, else `none` | **Must follow the body** (N1): `world/factory.py` refuses `world: sim` for a robot with no grid, so a teleop run without it dies on a `ValueError` |
| `SIM_MOTOR_BOARD`, `SIM_LIDAR` | unset | `fake` starts the simulated body, sensors and lidar as their own programs (3.36, 3.42) |
| `SIM_BODY_URL`, `SIM_SENSORS_URL`, `SIM_BODY_SHM`, `ROBOT_LIDAR` | `http://127.0.0.1:8002`; `http://127.0.0.1:8003,http://127.0.0.1:8004`; `picar_sim_body`; `/tmp/picar-lidar-<uid>` | Where the fake programs listen or link (only with `SIM_MOTOR_BOARD=fake`) |
| `PROXY_ROBOT_URL`, `PROXY_BRAIN_URL`, `PROXY_VISION_URL` | `http://127.0.0.1:8000`, `http://127.0.0.1:8001`, a hard-coded vision URL | `proxy.py`'s upstreams |
| `TUNNEL_LOG` | `~/.vision-picar-tunnel.log` | `restart.sh`'s log |

Warm the perception models once before a rig walk (the first tiered start
downloads weights); `run.sh` prints the one-liner.

## Where the rest is written

`docs/engineering/operations/ENGINEERING.md` (the tunnel section and the
variable table); `tools/jetson/README.md` for the `picar-stack` unit.
