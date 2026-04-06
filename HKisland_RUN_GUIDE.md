# HKisland数据集运行指南

## 📋 数据集信息

- **数据集**: HKisland_GNSS03.bag
- **相机类型**: 单目相机（Monocular）
- **图像格式**: 压缩彩色图像（CompressedImage）
- **图像分辨率**: 2448 x 2048
- **帧率**: 10 Hz
- **时长**: 6分30秒 (390秒)
- **图像话题**: `/left_camera/image/compressed`

## ⚙️ 配置文件

已创建配置文件: `/root/ORB_SLAM3/Examples/Monocular/HKisland_Mono.yaml`

包含了正确的相机内参、畸变系数和图像分辨率。

## 🚀 运行步骤

### 方法1：使用新编译的Mono_Compressed节点（推荐）

这个节点专门支持CompressedImage格式。

**步骤**：

1. **终端1 - 启动roscore**
```bash
source /opt/ros/noetic/setup.bash
roscore
```

2. **终端2 - 运行ORB_SLAM3**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml
```

3. **终端3 - 播放bag文件**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play --pause data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
```

4. **等待加载**
   - ORB_SLAM3会先加载词汇表（需要几秒钟）
   - 看到 "ORB-SLAM3 is ready" 后，在终端3按**空格键**开始播放

### 方法2：使用image_transport解压缩（备选）

如果方法1有问题，可以先解压缩图像再处理。

1. **终端1 - roscore**
```bash
source /opt/ros/noetic/setup.bash
roscore
```

2. **终端2 - 图像解压缩节点**
```bash
source /opt/ros/noetic/setup.bash
rosrun image_transport republish compressed raw \
    in:=/left_camera/image \
    out:=/camera/image
```

3. **终端3 - 运行ORB_SLAM3（使用原始Mono节点）**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml
```

4. **终端4 - 播放bag**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play --pause data/HKisland_GNSS03.bag
```

## 📊 预期输出

- **可视化窗口**: 会显示当前帧、特征点、地图点
- **终端输出**: 实时跟踪状态、特征点数量、FPS等信息
- **保存结果**: 程序结束后会在当前目录生成 `KeyFrameTrajectory.txt`

## 🔧 调试选项

### 查看图像话题
```bash
rostopic list | grep image
```

### 查看bag信息
```bash
rosbag info data/HKisland_GNSS03.bag
```

### 可视化图像流
```bash
rosrun image_view image_view image:=/camera/image_raw _image_transport:=compressed
```

### 调整播放速度
```bash
# 慢速播放 (0.5倍速)
rosbag play --rate 0.5 data/HKisland_GNSS03.bag ...

# 快速播放 (2倍速)
rosbag play --rate 2.0 data/HKisland_GNSS03.bag ...
```

## ⚠️ 常见问题

### 1. "command not found: rosbag"
**解决**: 需要source ROS环境
```bash
source /opt/ros/noetic/setup.bash
```

### 2. "Failed to contact master"
**解决**: 确保roscore在运行

### 3. 跟踪丢失（LOST）
**可能原因**:
- 初始化失败：运动太快或特征点不足
- 调整配置文件中的`ORBextractor.iniThFAST`和`minThFAST`阈值
- 尝试慢速播放: `--rate 0.5`

### 4. 词汇表加载缓慢
这是正常的，词汇表文件很大，需要等待10-30秒。

## 📁 文件位置

- **可执行文件**: `/root/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/Mono_Compressed`
- **词汇表**: `/root/ORB_SLAM3/Vocabulary/ORBvoc.txt`
- **配置文件**: `/root/ORB_SLAM3/Examples/Monocular/HKisland_Mono.yaml`
- **数据集**: `/root/ORB_SLAM3/data/HKisland_GNSS03.bag`
- **输出轨迹**: `./KeyFrameTrajectory.txt`（运行目录）

## 💡 提示

1. **首次运行建议慢速播放**以确保初始化成功
2. **Ctrl+C** 可以优雅地停止程序并保存结果
3. 如果需要**更多特征点**，可以在配置文件中增加`ORBextractor.nFeatures`
4. 单目SLAM需要**相机运动**才能初始化，确保bag开始时有足够的运动

## ✅ 成功标志

- 终端显示 "New map created with X points"
- 可视化窗口显示绿色的地图点
- 特征点被正确跟踪（显示为彩色点）

祝运行顺利！🎉

