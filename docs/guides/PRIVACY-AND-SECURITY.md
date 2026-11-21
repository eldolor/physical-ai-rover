# Privacy and security: what the robot sends, stores and exposes

This robot carries a camera through a home and draws a map of it. This
page says what leaves the house, what is stored and for how long, and who
can reach the robot. **It is a description, not a policy.** Every line
names the code, template or spec it was read from (`dev` `7c0bedf`,
2026-11-21), and that source governs. Where nothing in the repo answers a
question, the line says "not stated". Gaps are listed at the end as open
questions; this page sets no rule about them.

The decision behind most of it is in `docs/policy/ARCHITECTURE.md`,
"Privacy is a reason for the trigger discipline" (the user, 2026-10-09;
plan 3.52): "every frame that does not go to the cloud is a frame of the
house that never leaves it."

## 1. What leaves the house

| What | Where it goes | When | Read from |
|---|---|---|---|
| A camera frame, the target string and the rooms searched | The cloud vision service's `/navigate` route, which calls a model on Amazon Bedrock | Under the `tiered` policy, only on a trigger: mission start, a candidate sighting, a cold search, staleness, a disagreement between tiers, and one identity check at arrival. Under the `vision` policy, every step | `brain/navigate.py`; the trigger names in `brain/tiered.py`; arrival in `brain/arrival.py`; `service/vision_analyze/app.py` |
| A frame, to name the room it shows | The vision service's `/room` route | Only when room labels are set to come from the cloud. The shipped setting is `room_label_source: frame` (nothing is asked) and `find_room_labels: "off"` | `config/robot.yaml`; `room_name()` in `brain/navigate.py` |
| The target's name, no image | The vision service's `/room-prior` route | Once at a find's start, when that feature is on. It is off by default | `service/vision_analyze/app.py` (the route checks the secret and decodes no image); `CLAUDE.md` section 3b, 3.190 |
| A phone's camera frame | `/navigate` or `/guidance`, straight from the twin's Guide tab | Every guidance step | `docs/policy/ARCHITECTURE.md`, the table under the privacy decision |
| Recorded walk frames and the answers given on them | The walk store: local disk by default, or the private recordings bucket on S3 when the backend is `s3` (the deployed walks function runs with `RECORDING_BACKEND: s3`) | Only while a walk is being recorded | `control/walk_store.py`; `control/recording_routes.py`; `control/brain_config.py` (`recording_backend` defaults to `local`); `cloudformation/serverless.yaml` |
| Recorded frames, again | Bedrock, from the walks function: a replay re-asks `/navigate` for every frame of a walk, and the scorecard's judge and label help call a Bedrock model | When a person starts a replay, an evaluation or label help | `cloudformation/serverless.yaml` (`bedrock-judge`, `VISION_URL`); `control/walk_eval.py`; `control/label_assist.py`; `docs/recordings/ARCHITECTURE.md`, "Replay asks the deployed service" |
| The object inventory: labels and map positions of what one mission saw, no pixels | The recordings bucket under the inventory prefix, when `INVENTORY_BUCKET` is set. `service/tunnel/run.sh` sets it from the recordings stack's export; set but empty means local only | At mission end | `control/inventory_store.py` (`_for_s3()` strips the fields that tie a report to a saved map; the upload asks for `AES256`); `config/robot.yaml`; `service/tunnel/run.sh` |
| A metrics row: outcome, counts, timings, config; no pixels | The walks service's `/metrics/runs` route, when a metrics URL is set. `run.sh` sets it to the deployed site | At mission end | `control/metrics_client.py`; `control/metrics_routes.py`; `service/tunnel/run.sh` |
| Everything the phone asks of the robot and the brain, and their answers (camera frames, the map, the depth strip) | Through one ngrok domain to the person's phone | While the tunnel runs | `service/tunnel/run.sh`; `service/tunnel/proxy.py`; `docs/operations/ARCHITECTURE.md`, "One tunnel domain, split by path" |

**What does not leave.**

* **The map.** The ROS bridge saves the pose graph at every mission end to
  the robot's own disk (`PICAR_MAP_DIR`; a Docker volume in the simulator),
  three versions of each of three maps
  (`service/slam/src/picar_bridge/picar_bridge/mapstore.py`, `KEEP`,
  `KEEP_MAPS`). No code uploads a map: the only modules that open an S3
  client are the walk store, the inventory store, the scorecard, label
  help and the admin console. A private S3 copy was decided
  (`docs/plans/PLAN-ros-alignment.md` section 6, question 6) and is not
  built: step 3.50.D6 is open and waits on the user
  (`docs/plans/ros-alignment/3.50-map-persistence.md`). `CLAUDE.md` section
  6: "Maps are a home's floor plan: never commit or upload one unasked."
* **What-I-saw** (3.162): the frame an arrival's identity check judged and
  a thumbnail of each landmark stay on the robot, and reach only the
  person's phone through the brain's authenticated route
  (`brain/what_i_saw.py`).
* **No-go zones, room names, the schedule, intervention reports:** each
  module says local only (`control/nogo_store.py`, `brain/room_names.py`,
  `control/scheduler.py`, `control/brain_config.py` at `intervention_dir`).
* **Frames the local tiers handle.** YOLOE and CLIP run in the brain
  process, and the local VLM's `local_track` question is "never handed to
  the cloud" (`brain/tiered.py`). One exception to know: a local VLM that
  is down still reads as configured, and its questions then go to the paid
  cloud (`CLAUDE.md` section 6, 3.80).
* **`brain/vision.py`'s direct Anthropic call** is reached only from a
  manual test (`docs/policy/ARCHITECTURE.md`; `CLAUDE.md` section 1).

## 2. What is stored, where, and for how long

| What | Where | How long | Read from |
|---|---|---|---|
| Recorded walks (frames, answers, labels, replays), inventories, metrics rows | The recordings bucket: private (all four public-access blocks on), encrypted at rest (`AES256`), versioned, kept when its stack is deleted | Current objects have no expiry: after 30 days they move to infrequent-access storage and stay. A deleted or overwritten version is kept 90 days. Objects under `exports/` expire after 1 day. Incomplete uploads are aborted after 7 days | `cloudformation/recordings-s3.yaml` |
| A download link to a walk's file | A presigned S3 URL, "its own credential" | 900 seconds by default | `control/walk_store.py` (`expires_s`); `control/admin_server.py` |
| Recorded walks on the local backend; inventories; room names; no-go zones; intervention reports | Under `recordings/` on the machine that runs the brain. Git ignores the directory | Not stated: nothing deletes them | `config/robot.yaml`; `.gitignore` |
| What-I-saw | `recordings/what-i-saw` on the robot | At most 50 missions and 50 MB, the oldest deleted first; one mission at most 1 MB | `brain/what_i_saw.py` (`KEEP`, `STORE_MAX_BYTES`, `MISSION_MAX_BYTES`) |
| Saved maps | The robot's map directory | Three versions of each of three maps | `mapstore.py`, as above |
| Frames sent to Bedrock | Not stated in this repo. The vision service itself writes no frame anywhere: it has no storage client except Bedrock's | Not stated | `service/vision_analyze/app.py`; `service/vision_analyze/vision_core.py` |
| The cloud functions' logs | CloudWatch, through the basic Lambda execution role. A reply the service cannot parse is logged as text | Not stated: the template declares no log group and no retention | `cloudformation/serverless.yaml`; `service/vision_analyze/vision_core.py` |
| Lambda build archives | The deploy bucket's `lambda/` prefix | Expired after the template's `ExpireAfterDays` | `cloudformation/deploy-bucket.yaml` |
| The secrets and URLs the twin was given | The phone browser's `localStorage` | Until the person clears them | `web-twin/app.js` (`prefillTesterSecrets`) |

## 3. Who can reach the robot and the brain

**The shared secret.** The robot server, the brain, the simulator's body
and sensor servers, the local VLM, the ROS bridge and the cloud functions
each check an `x-app-secret` header against the `APP_SHARED_SECRET`
environment variable (`require_secret()` in `robot/server.py`,
`control/brain_server.py`, `control/admin_server.py`,
`sim/body_server.py`, `sim/sensor_server.py`,
`service/local_vlm/server.py`; `_check_secret()` in
`service/vision_analyze/app.py`; the handler in
`service/slam/src/picar_bridge/picar_bridge/bridge.py`).

* **It is inert when the variable is unset.** That is how local
  development and the test suite run (`CLAUDE.md` section 6).
* **There are three different secrets on purpose.** The local one gates
  driving the robot; the vision one gates asking the model; the walks one
  gates reviewing and deleting recordings, "a different privilege than
  being able to start a mission" (`control/admin_server.py`;
  `cloudformation/serverless.yaml`, `VisionSharedSecret` and
  `WalksSharedSecret`, both `NoEcho`).
* **The template's defaults are empty**, and it says so: "EMPTY MEANS NO
  AUTH ... Always pass one" (`cloudformation/serverless.yaml`).

**Routes left without the check, on purpose.**

| Service | Open routes | Read from |
|---|---|---|
| Robot server | `/health`; the twin's own page and assets (`/`, `/app.js`, `/ui.css`, `/twin.css`, `/strings.js`, `/js/{name}`, `/manifest.json`, the icons) | `robot/server.py` |
| Brain | `/health` | `control/brain_server.py` |
| Admin console and walks function | `/health`, `/recording/health` | `control/admin_server.py` |
| Vision service | `/health`, `GET /navigate/models` | `service/vision_analyze/app.py` |
| Local VLM, simulator body and sensor servers | `/health` | their own modules, as above |

Every other route of those services carries the check (counted from the
route decorators on 2026-11-21: 33 of 44 on the robot server, 18 of 19 on
the brain). The recording write routes take it from whichever server
mounts them (`control/recording_routes.py`).

**The tunnel.** `service/tunnel/run.sh` starts the robot server, the
brain and a path-splitting proxy on `127.0.0.1` only, and one ngrok domain
points at the proxy; `/brain/*` goes to the brain. The script refuses to
start when `LOCAL_SECRET` is missing, empty or shorter than 16 characters
("refusing to put an open robot on the internet"). `web-twin/app.js`
sends the `ngrok-skip-browser-warning` header to ngrok hosts only. ngrok's
free plan gives one domain to one running agent, so one machine holds the
tunnel at a time (`tools/jetson/README.md`).

**CloudFront.** The twin and the console are static files in a private
bucket that only the distribution reads (an origin access control); the
page itself is public. The API paths (`/navigate`, `/room`, `/guidance`,
`/recording/*`, `/metrics/*` and the rest) are HTTPS only and go to the
two functions, which apply their secrets (`cloudformation/serverless.yaml`).

**The ROS bridge** listens on every interface inside its container's own
network namespace, and on `127.0.0.1` under host networking, "so a run on
a laptop is not on the LAN" (`bridge.py`; `tests/demo_explore.py`).

**Who drives** is a separate matter from who may connect: the robot
server ranks every caller, and a person outranks the brain (`CLAUDE.md`
section 6, "The robot server arbitrates who drives").

## 4. What plan 3.52 decided and built

Source: `docs/plans/ros-alignment/3.52-privacy.md` and
`docs/handoffs/HANDOFF-2026-10-09-3.52-privacy.md`.

* **The user chose (2026-10-09)** to "write privacy into the policy spec
  now" and to measure, before adopting it, whether blurring people before
  a frame goes to the cloud fits the perception budget and leaves the
  cloud's answers unchanged.
* **Built:** the decision and the table of what leaves the house in
  `docs/policy/ARCHITECTURE.md`; `tools/person_eval.py` and
  `tests/test_person_eval.py` (43 adjudicated person frames and a scorer
  for any person detector). No product code, config or default changed.
* **Measured, against criteria written first:** recall FAILED (best 16.3%,
  bar 90%); false blur FAILED (21.1%, bar 5%); the budget was MET only when
  the pass runs on frames about to go to the cloud (172 ms p90 against
  305 ms); cloud answers FAILED on `action` (85%, bar 90%) and met
  `target_visible` at the bar (95%). Two false "person" boxes sat on the
  search target and hid it from the cloud.
* **Verdict:** "Blurring people is not adopted, and no blur code is
  built."
* **What would reopen it:** a floor-height rig walk with people standing,
  walking, seated and lying at 1-5 m in two light levels, and a detector
  judged on it before any blur is wired.

## 5. Secrets

* **They live outside the repo.** `~/.vision-picar-local-secrets` (mode
  600) holds `LOCAL_SECRET` and, optionally, `VISION_SECRET`;
  `~/.vision-picar-serverless-secrets` holds `WALKS_SECRET`
  (`service/tunnel/run.sh`; `docs/engineering/operations/ENGINEERING.md`).
  On the Jetson the same two files are copied to the board's home
  directory, with the ngrok configuration (`tools/jetson/README.md`).
* **The cloud functions take theirs as stack parameters** marked `NoEcho`
  (`cloudformation/serverless.yaml`). They authenticate to Bedrock through
  their IAM role, not a key (`CLAUDE.md` section 1).
* **`ANTHROPIC_API_KEY`** is needed only by `tests/manual_describe_image.py`
  (`CLAUDE.md` section 1).
* **Git ignores** `.env`, `*.pem` and `recordings/` (`.gitignore`).
* **The twin** can be handed its secrets and URLs in a setup link's query
  string; it saves them to `localStorage` and strips the query from the
  address bar at once (`web-twin/app.js`).

## Open questions

Found while tracing the lines above. Each is a question for the user;
none is answered or changed here.

1. **No expiry on current recordings.** The bucket's rules move walk
   frames to cheaper storage and expire old versions, but a recorded walk,
   an inventory or a metrics row stays until a person deletes it. Is that
   intended for frames of a home?
2. **No retention stated for the cloud functions' logs,** and the vision
   service logs a model reply it cannot parse as text (a description of a
   room, not pixels).
3. **Nothing in the repo states how Bedrock handles the frames it is
   sent.**
4. **The stack's two secrets default to empty, which the template itself
   calls "NO AUTH".** Nothing in the template refuses an empty one, as
   `run.sh` does for the local secret.
5. **The secret is compared in constant time only in the vision service**
   (`hmac.compare_digest`, 3.76 H2-9). The robot server, the brain, the
   admin console, the local VLM and the simulator servers compare with
   `!=`.
6. **Any web origin may call the brain** (`allow_origins=["*"]` in
   `control/brain_server.py`; the robot server's default is the same), so
   the secret is the only control on a reachable server.
7. **A setup link carries secrets in its query string.** The page strips
   it from the address bar, but the first request for the page has already
   carried it. The template turns on no CloudFront logging; whether
   anything else on the path records it is not stated.
8. **`allow_drills` ships `true`.** Its comment in `config/robot.yaml`
   says to turn it off "if the brain is ever reachable beyond the LAN";
   through the tunnel it is reachable, behind the secret.
9. **Local stores under `recordings/` have no limit** except what-I-saw's.
10. **The map's S3 copy is decided and not built** (3.50.D6, waiting on
    the user), so today a saved map exists only on the robot's disk.
