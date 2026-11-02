#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# evaluations/jetson-perf-3136/steady.sh -- 3.136.B's full-stack window, ON
# the board, from the bench checkout (~/bench-3136), under the board lock
# (run.py's `steady` runs it through board_run). Built from
# tools/jetson/headroom_mission.sh (same stack, same teardown); amendment 2
# in the section says what it runs and why.
#
#   bash evaluations/jetson-perf-3136/steady.sh <out-dir>
#
# The full stack (the section's definition): the robot server in sim mode
# (the fake board and the fake D500, SIM_MOTOR_BOARD=fake SIM_LIDAR=fake,
# ROBOT_DRIVE=ros), the brain on tiered find missions with explore
# searching (TIER_SEARCH=explore), the local tier (VE, service/local_vlm, the
# unit's engines, first for the OOM killer) and room labels from the frame
# (FIND_ROOM_LABELS=frame, explore_goal_policy rooms), the ROS container
# (vision-picar-ros:latest, SLAM + nav2), YOLOE + CLIP (perc_run.py, i.e.
# tools/jetson/perception_loop.py at 4 Hz), the cloud a $0 stand-in.
#
# One window, at least WINDOW_S (1800 s, P4's 30 min), stamped in $OUT:
#   P1  perception, plain prompts, P1_S (600 s)
#   P8  bench_local_vlm.py frames on 3.78's 299 (4 walks), the P1 loop still on
#   --  the loop restarted with --hazards (not scored: the text encode)
#   P2  perception with 3.74.C's hazard prompts, P2_S (600 s)
#   --  held until WINDOW_S if shorter
# P4, P11 and P12 are read over the whole window. Telemetry is the harness's
# Sampler (run.py); this script records the stamps and the loads' logs.
set -uo pipefail
source tools/jetson/board_lock_check.sh
source tools/jetson/vlm_hold.sh || { echo "STOP: no vlm_hold.sh"; exit 2; }
OUT="${1:?out dir}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
HERE=evaluations/jetson-perf-3136
IMAGE="${IMAGE:-vision-picar-ros:latest}"
NAME="${NAME:-picar-ros-3136}"
HOUSE="${HOUSE:-procthor:val:12}"
TARGETS="${TARGETS:-garbage can|house plant|laundry hamper|basketball|dog bed|box|plunger|cart|garbage bag}"
WINDOW_S="${WINDOW_S:-1800}"; P1_S="${P1_S:-600}"; P2_S="${P2_S:-600}"; WARM_S="${WARM_S:-20}"
P8_WALKS="${P8_WALKS:-blue-bottle-20260907-142454 blue-bottle-20260907-185007 blue-shoes-20260907-152528 red-backpack-20260907-144856}"
ENGINES="${ENGINES:-$HOME/bench-378/ws91/q25VE/engines}"
EDGELLM="${EDGELLM:-$HOME/bench-378/ell91}"
source .venv/bin/activate
set -a; source ~/.vision-picar-local-secrets; set +a
export APP_SHARED_SECRET="$LOCAL_SECRET"
H=(-H "x-app-secret: $LOCAL_SECRET")
oom() { sudo dmesg | grep -ciE "killed process" || true; }
stamp() { date +%s.%N > "$OUT/$1"; echo "== $1 $(date +%T)" | tee -a "$OUT/run.log"; }
health() { curl -s "${H[@]}" localhost:8000/health > "$OUT/$1"; }
waitfor() {  # seconds what cmd...
  local s=$1 what=$2; shift 2
  local end=$(( $(date +%s) + s ))
  until "$@" 2>/dev/null; do
    [ "$(date +%s)" -ge "$end" ] && { echo "STOP: $what not reached in ${s}s" | tee -a "$OUT/run.log"; exit 3; }
    sleep 1
  done
}
PERC=; VLMP=; CLD=; MIS=; MAPP=; WHL=
teardown() {
  rm -f "$OUT/missions.run" "$OUT/mappoll.run"
  [ -n "$MIS" ] && { for _ in $(seq 60); do kill -0 $MIS 2>/dev/null || break; sleep 1; done; }
  docker exec "$NAME" pkill -f ros_pace.py >/dev/null 2>&1; sleep 1
  docker cp "$NAME:/tmp/pace.jsonl" "$OUT/pace.jsonl" >/dev/null 2>&1
  docker logs "$NAME" > "$OUT/ros.log" 2>&1
  kill $PERC $MIS $MAPP $CLD $WHL 2>/dev/null
  docker rm -f "$NAME" >/dev/null 2>&1
  pids="$(pgrep -f 'bash service/tunnel/run.sh'; for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN; done)"
  [ -n "$pids" ] && kill $pids 2>/dev/null; sleep 3
  pids="$(for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN; done)"
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  if [ -n "$VLMP" ]; then
    kill $VLMP 2>/dev/null
    for _ in $(seq 40); do kill -0 "$VLMP" 2>/dev/null || break; sleep 0.5; done
    kill -9 "$VLMP" 2>/dev/null
  fi
  echo "$(oom)" > "$OUT/oom_end"
  sudo dmesg | grep -iE "killed process|oom-kill|invoked oom-killer" > "$OUT/oom.log"
  sudo systemctl isolate graphical.target
  vlm_restore | tee -a "$OUT/run.log"
  echo "== torn down $(date +%T); desktop: $(systemctl is-active gdm)" | tee -a "$OUT/run.log"
}
trap teardown EXIT
vlm_hold "$OUT/vlm_hold.log" || { vlm_restore; trap - EXIT; exit 2; }

sudo systemctl isolate multi-user.target; sleep 5
sudo systemctl stop picar-stack picar-ngrok picar-local-vlm 2>/dev/null; sleep 3
{ echo "== run $(date +%T) rev $(git rev-parse --short HEAD) image $IMAGE house $HOUSE"
  echo "== $(sudo nvpmodel -q | head -1); boot $(uptime -s); uptime $(cut -d' ' -f1 /proc/uptime) s"
  echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok) $(systemctl is-active picar-local-vlm)"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
sudo nvpmodel -q 2>&1 | grep -m1 'NV Power Mode' > "$OUT/nvpmodel.txt"
echo "$(oom)" > "$OUT/oom_start"
docker rm -f "$NAME" >/dev/null 2>&1

# The cloud: a $0 stand-in (3.80.E's), never Bedrock.
{ ! command -v ss >/dev/null || ss -ltnH "sport = :8099" | grep -q .; } && { echo "STOP: :8099 in use" | tee -a "$OUT/run.log"; exit 2; }
python evaluations/tiered-380/E/standin_cloud.py 8099 "$OUT/cloud_calls.jsonl" > "$OUT/standin.log" 2>&1 &
CLD=$!
waitfor 30 "stand-in cloud" curl -sf localhost:8099/health -o /dev/null
# VE, as its unit runs it (same engines, secret, first for the OOM killer).
{ ! command -v ss >/dev/null || ss -ltnH "sport = :8005" | grep -q .; } && { echo "STOP: :8005 in use" | tee -a "$OUT/run.log"; exit 2; }
LOCAL_VLM_ENGINES="$ENGINES" LOCAL_VLM_EDGELLM="$EDGELLM" \
  python -m service.local_vlm.server > "$OUT/local_vlm.log" 2>&1 &
VLMP=$!
echo 1000 | sudo tee /proc/$VLMP/oom_score_adj >/dev/null
waitfor 300 "local VLM ready" sh -c "kill -0 $VLMP && curl -s localhost:8005/health | grep -q '\"ready\": *true'"
# Its driver child too (the unit's OOMScoreAdjust covers the whole service).
for c in $(pgrep -P $VLMP); do echo 1000 | sudo tee /proc/$c/oom_score_adj >/dev/null; done
echo "== VE ready $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"

# The robot server and the brain, with the brain's timing shim (P4, P12).
PYTHONPATH="$PWD/$HERE/shim" PERF_SHIM_LOG="$OUT/shim.jsonl" \
  SIM_MAP="$HOUSE" ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake SIM_LIDAR=fake \
  VISION_URL=http://127.0.0.1:8099 LOCAL_VLM_URL=http://127.0.0.1:8005 TIER_SEARCH=explore \
  FIND_ROOM_LABELS=frame EXPLORE_GOAL_POLICY=rooms INVENTORY_BUCKET= \
  bash service/tunnel/restart.sh >> "$OUT/run.log" 2>&1 || { echo "STOP: restart.sh failed" | tee -a "$OUT/run.log"; exit 2; }
waitfor 90 "wheels usable" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8000/wheels | grep -q '\"usable\": *true'"
# picar-stack's own OOMScoreAdjust (-900), as in production.
for p in 8000 8001 8002 8003 8004 8080; do
  for q in $(lsof -nP -tiTCP:$p -sTCP:LISTEN); do echo -900 | sudo tee /proc/$q/oom_score_adj >/dev/null; done
done
{ ! command -v ss >/dev/null || ss -ltnH "sport = :8090" | grep -q .; } && { echo "STOP: :8090 in use" | tee -a "$OUT/run.log"; exit 2; }
docker run -d --name "$NAME" --network host \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  "$IMAGE" ros2 launch picar_bringup picar.launch.py >/dev/null
waitfor 180 "bridge up" curl -sf localhost:8090/health -o /dev/null
waitfor 120 "start_truth" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8090/slam/pose | grep -q '\"start_truth\": {'"
docker cp "$HERE/ros_pace.py" "$NAME:/tmp/ros_pace.py"
docker exec -d "$NAME" /entrypoint.sh python3 /tmp/ros_pace.py /tmp/pace.jsonl
echo "== stack up $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"

# P1's loop warm (first frame, then WARM_S) before the missions.
PERC_LOG="$OUT/perc-p1.jsonl" PYTHONPATH=. python "$HERE/perc_run.py" 7200 > "$OUT/perception-p1.log" 2> "$OUT/perception-p1.err" &
PERC=$!
waitfor 300 "perception loaded" grep -q "loaded 1 pipeline" "$OUT/perception-p1.log"
waitfor 300 "perception's first frame" grep -qE '^[0-9:]{8} [0-9]+ ' "$OUT/perception-p1.log"
sleep "$WARM_S"

touch "$OUT/missions.run" "$OUT/mappoll.run"
IFS='|' read -ra TGT <<< "$TARGETS"
python "$HERE/board_probe.py" missions "$OUT/missions.run" "$OUT/missions.jsonl" "${TGT[@]}" > "$OUT/missions.log" 2>&1 &
MIS=$!
python "$HERE/board_probe.py" mappoll "$OUT/mappoll.run" "$OUT/mappoll.jsonl" 1.0 > "$OUT/mappoll.log" 2>&1 &
MAPP=$!
sleep 30   # the first mission under way
# When a late tick happened (the window's verdict reads only the two ends).
( while true; do echo "$(date +%s.%N) $(curl -s "${H[@]}" localhost:8000/health | python3 -c 'import sys,json; w=json.load(sys.stdin)["wheel_loop"]; print(w["moving_ticks"], w["late_ticks"], w["max_dt_s"])' 2>/dev/null)"; sleep 5; done ) > "$OUT/wheel.log" &
WHL=$!

health health_w0.json; stamp window_start; stamp p1_start
sleep "$P1_S"
stamp p1_end

stamp p8_start
WARGS=(); for w in $P8_WALKS; do WARGS+=(--walk "$w"); done
python -m tools.jetson.bench_local_vlm frames --recordings recordings "${WARGS[@]}" \
  --out "$OUT/vlm.json" > "$OUT/vlm.log" 2>&1
echo $? > "$OUT/p8_exit"
stamp p8_end

kill $PERC 2>/dev/null; wait $PERC 2>/dev/null
PERC_LOG="$OUT/perc-p2.jsonl" PYTHONPATH=. python "$HERE/perc_run.py" --hazards 7200 > "$OUT/perception-p2.log" 2> "$OUT/perception-p2.err" &
PERC=$!
waitfor 300 "hazard perception loaded" grep -q "loaded 1 pipeline" "$OUT/perception-p2.log"
waitfor 300 "hazard perception's first frame" grep -qE '^[0-9:]{8} [0-9]+ ' "$OUT/perception-p2.log"
sleep "$WARM_S"
stamp p2_start
sleep "$P2_S"
stamp p2_end

W0=$(cat "$OUT/window_start")
left=$(python3 -c "import time; print(max(0, int($WINDOW_S - (time.time() - $W0)) + 1))")
[ "$left" -gt 0 ] && sleep "$left"
health health_w1.json; stamp window_end
kill $PERC 2>/dev/null; PERC=
echo "== MemAvailable at the end $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB; VE $(kill -0 $VLMP 2>/dev/null && echo alive || echo dead)" | tee -a "$OUT/run.log"
echo "== done $(date +%T)" | tee -a "$OUT/run.log"
