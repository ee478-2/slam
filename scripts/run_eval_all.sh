#!/usr/bin/env bash
# Batch-run eval on all 4 TUM bags. Sequential — only one rtabmap process at a time.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RATE="${1:-0.5}"

BAGS=(
  "$REPO_ROOT/src/data/rgbd_dataset_freiburg2_pioneer_360.bag"
  "$REPO_ROOT/src/data/rgbd_dataset_freiburg2_pioneer_slam.bag"
  "$REPO_ROOT/src/data/rgbd_dataset_freiburg2_pioneer_slam2.bag"
  "$REPO_ROOT/src/data/rgbd_dataset_freiburg2_pioneer_slam3.bag"
)

for BAG in "${BAGS[@]}"; do
  if [[ ! -f "$BAG" ]]; then
    echo "[SKIP] missing bag: $BAG" >&2
    continue
  fi
  "$REPO_ROOT/scripts/run_eval_one.sh" "$BAG" "$RATE"
done

echo
echo "=== Summary ==="
for d in "$REPO_ROOT"/eval_results/*/; do
  name="$(basename "$d")"
  if [[ -f "$d/ape.txt" ]]; then
    echo "--- $name ---"
    grep -E "^\s*(rmse|mean|median|max|min|sse|std)" "$d/ape.txt" || true
  fi
done
