#!/usr/bin/env python3
"""Convert sampleinfos_interpolated.json (SfM/photogrammetry GT) to TUM format.

The MARS-LVIG dataset provides sampleinfos_interpolated.json containing per-frame
camera poses from offline SfM reconstruction. This is a higher-quality ground truth
than RTK GPS for visual odometry evaluation because both the GT and the estimated
trajectory live in a vision-based coordinate system, making Sim(3) alignment more
accurate.

Usage:
    python3 data/sampleinfos_to_tum.py data/sampleinfos_interpolated.json data/ground_truth_sfm.txt
"""

import json
import math
import sys


def rotation_matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2][1] - R[1][2]) / S
        qy = (R[0][2] - R[2][0]) / S
        qz = (R[1][0] - R[0][1]) / S
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        qw = (R[2][1] - R[1][2]) / S
        qx = 0.25 * S
        qy = (R[0][1] + R[1][0]) / S
        qz = (R[0][2] + R[2][0]) / S
    elif R[1][1] > R[2][2]:
        S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        qw = (R[0][2] - R[2][0]) / S
        qx = (R[0][1] + R[1][0]) / S
        qy = 0.25 * S
        qz = (R[1][2] + R[2][1]) / S
    else:
        S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        qw = (R[1][0] - R[0][1]) / S
        qx = (R[0][2] + R[2][0]) / S
        qy = (R[1][2] + R[2][1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 sampleinfos_to_tum.py <input.json> <output.txt>")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2]

    with open(input_path) as f:
        data = json.load(f)

    count = 0
    with open(output_path, 'w') as out:
        for d in data:
            timestamp = float(d["OriginalImageName"].replace(".jpg", ""))
            T = d["T4x4"]
            R = [row[:3] for row in T[:3]]
            tx, ty, tz = T[0][3], T[1][3], T[2][3]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
            out.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" %
                      (timestamp, tx, ty, tz, qx, qy, qz, qw))
            count += 1

    print("Converted %d poses from %s to %s" % (count, input_path, output_path))


if __name__ == "__main__":
    main()
