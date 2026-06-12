#!/usr/bin/env bash
# Guide for obtaining the AMtown02 dataset and verifying its integrity.
# MARS-LVIG is distributed by HKU MARS Lab and requires manual download.
set -euo pipefail
cd "$(dirname "$0")/.."

BAG=data/AMtown02.bag
SHA256_EXPECTED="dc5347a8ee9efeb63fda6766af904e70256747b72b4c8405a746c2e83537115e"

if [ ! -f "$BAG" ]; then
    cat <<'EOF'
AMtown02.bag not found under data/.

The MARS-LVIG dataset is distributed by the HKU MARS Lab:

  1. Visit https://mars.hku.hk/dataset.html
  2. Locate the AMtown02 sequence (rosbag, ~17 GB)
  3. Download and place it at: data/AMtown02.bag
  4. Re-run this script to verify the checksum

EOF
    exit 1
fi

echo "Verifying SHA256 of $BAG (this takes a few minutes)..."
SHA256_ACTUAL=$(sha256sum "$BAG" | awk '{print $1}')
if [ "$SHA256_ACTUAL" = "$SHA256_EXPECTED" ]; then
    echo "OK: checksum matches."
else
    echo "WARNING: checksum mismatch!"
    echo "  expected: $SHA256_EXPECTED"
    echo "  actual:   $SHA256_ACTUAL"
    echo "The file may be corrupted or a different release of the sequence."
    exit 1
fi
