#!/usr/bin/env python3
"""
Extract ground truth from DJI rosbag in TUM format.
Uses /dji_osdk_ros/gps_position (NavSatFix) converted to local ENU,
combined with /dji_osdk_ros/attitude (QuaternionStamped).
Output: timestamp tx ty tz qx qy qz qw
"""
import math
import bisect
import sys
import rosbag

bag_file = sys.argv[1] if len(sys.argv) > 1 else "AMtown02.bag"
output_file = sys.argv[2] if len(sys.argv) > 2 else "AMtown02_groundtruth.txt"

pos_topic = "/dji_osdk_ros/gps_position"
att_topic = "/dji_osdk_ros/attitude"

WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Convert geodetic (deg) to local ENU (meters) relative to origin."""
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    lat0_rad = math.radians(lat0)

    sin_lat0 = math.sin(lat0_rad)
    R_N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
    R_M = R_N * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sin_lat0 * sin_lat0)

    east = d_lon * (R_N + alt0) * math.cos(lat0_rad)
    north = d_lat * (R_M + alt0)
    up = alt - alt0
    return east, north, up


print(f"Reading bag: {bag_file}")
bag = rosbag.Bag(bag_file)

gps_data = []
attitudes = []

for topic, msg, t in bag.read_messages(topics=[pos_topic, att_topic]):
    stamp = msg.header.stamp.to_sec()
    if topic == pos_topic:
        gps_data.append((stamp, msg.latitude, msg.longitude, msg.altitude))
    elif topic == att_topic:
        attitudes.append((stamp, msg.quaternion.x, msg.quaternion.y,
                          msg.quaternion.z, msg.quaternion.w))

bag.close()
print(f"Loaded {len(gps_data)} GPS msgs, {len(attitudes)} attitude msgs")

lat0, lon0, alt0 = gps_data[0][1], gps_data[0][2], gps_data[0][3]
print(f"Origin: lat={lat0:.8f}, lon={lon0:.8f}, alt={alt0:.2f}")

att_times = [a[0] for a in attitudes]

matched = 0
with open(output_file, 'w') as f:
    for g in gps_data:
        t_gps = g[0]
        e, n, u = geodetic_to_enu(g[1], g[2], g[3], lat0, lon0, alt0)

        idx = bisect.bisect_left(att_times, t_gps)
        best_idx, best_dt = None, float('inf')
        for c in [idx - 1, idx]:
            if 0 <= c < len(attitudes):
                dt = abs(attitudes[c][0] - t_gps)
                if dt < best_dt:
                    best_dt = dt
                    best_idx = c

        if best_idx is not None and best_dt < 0.05:
            att = attitudes[best_idx]
            f.write(f"{t_gps:.9f} {e} {n} {u} "
                    f"{att[1]} {att[2]} {att[3]} {att[4]}\n")
            matched += 1

print(f"Written {matched} ground truth poses to {output_file}")
