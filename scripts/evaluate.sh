#!/usr/bin/env bash
# Evaluate an AMtown02 trajectory against both ground truths with evo.
# Usage: scripts/evaluate.sh [run_name]    (default: AMtown02)
set -euo pipefail
cd "$(dirname "$0")/.."

NAME=${1:-AMtown02}
TRAJ_NS=f_${NAME}.txt
TRAJ_S=traj_sec.txt

if [ ! -f "$TRAJ_NS" ]; then
    echo "ERROR: $TRAJ_NS not found. Run scripts/run_amtown.sh first." >&2
    exit 1
fi

# mono_euroc writes nanosecond timestamps; evo + our GT use seconds
awk '{printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2,$3,$4,$5,$6,$7,$8}' \
    "$TRAJ_NS" > "$TRAJ_S"

echo "=== ATE vs SfM ground truth (primary) ==="
evo_ape tum data/ground_truth_sfm.txt "$TRAJ_S" \
    --align --correct_scale --t_max_diff 0.1 -a

echo
echo "=== ATE vs RTK GPS ground truth ==="
evo_ape tum data/AMtown02_groundtruth.txt "$TRAJ_S" \
    --align --correct_scale --t_max_diff 0.1 -a

echo
echo "Expected: ATE RMSE 2.2-2.9 m vs SfM GT (see docs/REPRODUCIBILITY.md)"
