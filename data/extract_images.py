#!/usr/bin/env python3
"""Extract images from AMtown02.bag for offline mono_euroc processing.

Extracts compressed images from the rosbag, resizes to 0.5x (1224x1024),
and saves in the directory structure expected by mono_euroc (EuRoC format).

Usage:
    python3 data/extract_images.py [--bag data/AMtown02.bag] [--output data/AMtown02_offline]

Output structure:
    data/AMtown02_offline/
    ├── mav0/cam0/data/          # Images named <timestamp_ns>.png
    └── times.txt                # Timestamp list (nanoseconds)
"""

import argparse
import os
import time

import cv2
import numpy as np
import rosbag


def main():
    parser = argparse.ArgumentParser(description="Extract images from rosbag")
    parser.add_argument("--bag", default="data/AMtown02.bag", help="Input rosbag path")
    parser.add_argument("--output", default="data/AMtown02_offline", help="Output directory")
    parser.add_argument("--topic", default="/left_camera/image/compressed")
    parser.add_argument("--scale", type=float, default=0.5, help="Resize scale (0.5 = half resolution)")
    args = parser.parse_args()

    out_dir = os.path.join(args.output, "mav0", "cam0", "data")
    times_file = os.path.join(args.output, "times.txt")
    os.makedirs(out_dir, exist_ok=True)

    bag = rosbag.Bag(args.bag, 'r')
    count = 0
    timestamps = []
    t0 = time.time()

    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        ts_ns = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if args.scale != 1.0:
            h, w = img.shape[:2]
            new_w, new_h = int(w * args.scale), int(h * args.scale)
            img = cv2.resize(img, (new_w, new_h))

        cv2.imwrite(os.path.join(out_dir, "%d.png" % ts_ns), img)
        timestamps.append(str(ts_ns))
        count += 1

        if count % 500 == 0:
            elapsed = time.time() - t0
            rate = count / elapsed
            print("  %d images (%.0fs, %.1f img/s)" % (count, elapsed, rate), flush=True)

    bag.close()

    with open(times_file, 'w') as f:
        for ts in timestamps:
            f.write(ts + '\n')

    elapsed = time.time() - t0
    print("Done: %d images in %.0fs" % (count, elapsed))
    print("Output: %s" % out_dir)
    print("Times:  %s (%d entries)" % (times_file, len(timestamps)))


if __name__ == "__main__":
    main()
