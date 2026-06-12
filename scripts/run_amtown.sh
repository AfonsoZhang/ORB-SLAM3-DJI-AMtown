#!/usr/bin/env bash
# Run monocular VO on AMtown02 end-to-end (image extraction + mono_euroc).
# Usage: scripts/run_amtown.sh [run_name]    (default: AMtown02)
set -euo pipefail
cd "$(dirname "$0")/.."

NAME=${1:-AMtown02}
BAG=data/AMtown02.bag
SEQ=data/AMtown02_offline

if [ ! -d "$SEQ/mav0/cam0/data" ]; then
    if [ ! -f "$BAG" ]; then
        echo "ERROR: $BAG not found. See scripts/download_dataset.sh" >&2
        exit 1
    fi
    echo "==> Extracting 0.5x images from rosbag (one-time, ~7500 frames)..."
    python3 tools/extract_images.py --bag "$BAG" --output "$SEQ"
fi

echo "==> Running mono_euroc on $SEQ (~13 min, headless)..."
./Examples/Monocular/mono_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/AMtown_Mono_MARSLVIG.yaml \
    "$SEQ" \
    "$SEQ/times.txt" \
    "$NAME"

echo "==> Done. Trajectory: f_${NAME}.txt (frames), kf_${NAME}.txt (keyframes)"
echo "    Evaluate with: scripts/evaluate.sh $NAME"
