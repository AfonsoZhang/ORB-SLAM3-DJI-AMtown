import rosbag, cv2, numpy as np, os, time

bag_path = 'data/AMtown02.bag'
out_dir = 'data/AMtown02_offline_full/mav0/cam0/data'
times_file = 'data/AMtown02_offline_full/times.txt'
os.makedirs(out_dir, exist_ok=True)

bag = rosbag.Bag(bag_path, 'r')
count = 0
timestamps = []
t0 = time.time()

for topic, msg, t in bag.read_messages(topics=['/left_camera/image/compressed']):
    ts_ns = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite(os.path.join(out_dir, f"{ts_ns}.png"), img)
    timestamps.append(str(ts_ns))
    count += 1
    if count % 500 == 0:
        elapsed = time.time() - t0
        rate = count / elapsed
        eta = (7500 - count) / rate if rate > 0 else 0
        print(f"  {count} images ({elapsed:.0f}s elapsed, ETA {eta:.0f}s)", flush=True)

bag.close()
with open(times_file, 'w') as f:
    f.write('\n'.join(timestamps) + '\n')

print(f"\nDone: {count} full-res images in {time.time()-t0:.0f}s")
