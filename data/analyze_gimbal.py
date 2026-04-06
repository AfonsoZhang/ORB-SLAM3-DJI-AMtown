#!/usr/bin/env python3
"""
Analyze gimbal angle data from AMtown02 rosbag to understand
why fixed T_b_c1 extrinsics cause Mono-Inertial SLAM failure.

Key finding: gimbal yaw changes >100 degrees between survey legs,
making the camera-body transform time-varying.
"""
import rosbag
import numpy as np
import sys

bag_path = sys.argv[1] if len(sys.argv) > 1 else 'data/AMtown02.bag'

rolls, pitches, yaws, times = [], [], [], []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/dji_osdk_ros/gimbal_angle']):
        ts = t.to_sec()
        rolls.append(msg.vector.x)
        pitches.append(msg.vector.y)
        yaws.append(msg.vector.z)
        times.append(ts)

rolls = np.array(rolls)
pitches = np.array(pitches)
yaws = np.array(yaws)
times = np.array(times)

print(f"Gimbal data: {len(times)} samples over {times[-1]-times[0]:.1f}s")
print(f"\nRoll  range: [{rolls.min():.1f}, {rolls.max():.1f}] deg, std={rolls.std():.2f}")
print(f"Pitch range: [{pitches.min():.1f}, {pitches.max():.1f}] deg, std={pitches.std():.2f}")
print(f"Yaw   range: [{yaws.min():.1f}, {yaws.max():.1f}] deg, std={yaws.std():.2f}")
print(f"\nYaw total variation: {yaws.max() - yaws.min():.1f} deg")

dyaw = np.abs(np.diff(yaws))
big_changes = np.where(dyaw > 5.0)[0]
print(f"Yaw jumps > 5 deg: {len(big_changes)} times")

if yaws.max() - yaws.min() > 30:
    print("\n*** CONCLUSION: Gimbal yaw varies significantly (>30 deg).")
    print("*** Fixed T_b_c1 assumption in ORB-SLAM3 is VIOLATED.")
    print("*** Mono-Inertial SLAM with body IMU requires dynamic extrinsics.")
