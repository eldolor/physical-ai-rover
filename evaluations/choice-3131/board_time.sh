#!/usr/bin/env bash
# 3.131.B: 3.125.B's board script (evaluations/goal-learn-3125/board_time.sh)
# with this section's timing (choice_time.py: F-large as explore now makes it,
# beside the path without the hand-off, alternating at every pose). Runs ON
# the board from ~/bench-3131 (a git checkout of the branch: restart.sh checks each
# server's reported revision against it), the service venv:
#   1. quiet: the board as found (picar-stack serving the twin, sim mode)
#   2. full:  headroom_identity.sh's window (RUNTIME=trt, VE's engines,
#      vision-picar-ros:latest); the bench starts when the window starts and
#      must end inside it
#   3. the twin's units restarted (the harness stops them and leaves them down)
# PICAR_BOARD_TOKEN must be set (the board lock held by the caller).
set -uo pipefail
cd ~/bench-3131
# headroom_identity.sh activates the checkout's .venv; an archive has none.
[ -e .venv ] || ln -s ~/vision-picar-service/.venv .venv
# It runs perception from here too: the service checkout's untracked weights
# and the recorded frames it perceives.
for f in yoloe-11s-seg.pt mobileclip_blt.ts recordings; do
  [ -e "$f" ] || ln -s ~/vision-picar-service/"$f" "$f"
done
OUT=out; mkdir -p "$OUT"
PY=~/vision-picar-service/.venv/bin/python
log() { echo "$(date +%T) $*" | tee -a "$OUT/driver.log"; }
log "== quiet: $(systemctl is-active picar-stack) picar-stack, $(sudo nvpmodel -q | head -1), $(uptime)"
$PY -m pytest -q -p no:cacheprovider tests/test_room_features_ctx.py 2>&1 | tail -1 | tee -a "$OUT/driver.log"
$PY evaluations/choice-3131/choice_time.py quiet "$OUT/quiet.json" > "$OUT/quiet.log" 2>&1
log "quiet exit $?"

log "== full stack"
RUNTIME=trt ENGINES="$HOME/bench-378/ws91/q25VE/engines" IMAGE=vision-picar-ros:latest \
  NAME=picar-ros-3131 bash tools/jetson/headroom_identity.sh "$OUT/full" > "$OUT/harness.log" 2>&1 &
H=$!
until grep -q "== window start" "$OUT/full/run.log" 2>/dev/null; do
  kill -0 $H 2>/dev/null || { log "harness ended before its window"; break; }
  sleep 2
done
if grep -q "== window start" "$OUT/full/run.log" 2>/dev/null; then
  log "window started; choice bench"
  $PY evaluations/choice-3131/choice_time.py full "$OUT/full.json" > "$OUT/full.log" 2>&1
  log "full exit $?; window ended yet: $(grep -c '== window end' "$OUT/full/run.log")"
fi
wait $H; log "harness exit $?"
sudo systemctl start picar-stack picar-ngrok
sleep 15; log "twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); local VLM $(systemctl is-active picar-local-vlm)"
log "DONE"
