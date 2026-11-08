#!/usr/bin/env bash
# 3.158.B: batches, one process each, so the exclusive desktop lock is released between them.
out="$1"; shift
while true; do
  /home/anshu/vision-picar/.venv/bin/python evaluations/autonomy-3158/B/rehearsal.py "$out" "$@"
  rc=$?
  [ "$rc" = 3 ] || { echo "rehearsal exit=$rc"; exit "$rc"; }
  sleep 90    # a waiter (another session's live run) takes the lock here
done
