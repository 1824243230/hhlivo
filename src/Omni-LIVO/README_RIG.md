# Omni-LIVO 8C4L2I 可运行框架

`rig_mapping` 在当前 Omni-LIVO 包内提供 8 相机、4 雷达、2 IMU 入口，支持离线 bag 和实时 ROS 订阅。复用 Omni-LIVO 的体素八叉树平面地图、Vikit 相机模型及光度仿射变形代码；惯性传播、传感器调度、光度观测和完整协方差更新由新的联合估计器负责。原 `fastlivo_mapping` 入口继续保留。

GPS、磁力计、IMU 内部融合姿态和 HBS 内已有定位结果不参与估计。这里提供的 HBS 配置是示例参数；真实定位精度需在替换标定后评估。

## 构建

依赖 ROS Noetic、Eigen、PCL、OpenCV、旧版非模板 Sophus、yaml-cpp、Vikit 和 livox_ros_driver。当前机器已有这些依赖。

在工作区根目录执行：

```bash
bash src/Omni-LIVO/scripts/build_rig.sh
```

构建输出为 `build/omni/devel/lib/omni_livo/rig_mapping`。脚本默认复用当前机器 `~/OmniLIVO_ws/devel` 的依赖；其他机器用 `RIG_DEPENDENCY_PREFIX` 指定已构建的依赖前缀。也可将本包和依赖放入正常 catkin 工作区构建。

数据工具使用 ROS Python 3，推荐 `/usr/bin/python3`。额外依赖列在 `scripts/requirements-rig.txt`；本机转换器也能复用 `~/hubs2rosbag/.deps` 中兼容 Python 3.8 的 PyAV。

## 转换和回放当前 HBS

```bash
/usr/bin/python3 src/Omni-LIVO/scripts/hbs_multi.py \
  hbus_0007_20260917111850371LLmineLLliudongLL22.04_bak.hbs \
  --output artifacts/hbs/rig.bag --split-gb 2

/usr/bin/python3 src/Omni-LIVO/scripts/verify_rig_bag.py \
  artifacts/hbs/*.bag --output artifacts/hbs/validation.json

build/omni/devel/lib/omni_livo/rig_mapping \
  --config src/Omni-LIVO/config/rig_hbs.yaml \
  --bag artifacts/hbs/*.bag --output artifacts/hbs/run

/usr/bin/python3 src/Omni-LIVO/scripts/verify_rig_run.py \
  artifacts/hbs/run --conversion-report artifacts/hbs/rig.report.json --require-all
```

以上路径已经存在时，请选择新的输出目录/文件名。程序不覆盖已有结果。

当前 HBS 已转换到 `artifacts/hbs/rig.bag` 和 `rig_001.bag`～`rig_009.bag`。转换按相机实例保持独立 H.265 解码器，记录原始 PTS；采用磁盘中间文件，内存不会随所有解码图像增长。`--start`、`--duration` 可截取，但仍从前面的编码帧建立解码参考。

输出是标准化的 14 路 ROS 消息：

| 话题 | 消息 |
|---|---|
| `/rig/camera/0..7/image_raw` | 原始尺寸 `sensor_msgs/Image`，`bgr8` |
| `/rig/lidar/0..3/points` | `sensor_msgs/PointCloud2`，x/y/z/intensity/offset_time/ring |
| `/rig/imu/0..1/data_raw` | `sensor_msgs/Imu`，m/s²、rad/s |

雷达消息头为最早点时间，`offset_time` 为 uint32 纳秒。bag 记录时间是最后一点的时间，便于回放时模拟整帧可用性。雷达 0 及 IMU 1 的源时间按纳秒，其余按微秒；双 IMU 根据载荷 `instId` 分流。雷达厘米坐标转为米。角速度默认按 °/s 转换，各实例可用 `--imu0-gyro-unit`、`--imu1-gyro-unit` 独立改为 rad/s。

转换报告中的相机解码损失保留时间及原因，不能将编码包数量直接当作输出图像数量。GPS 及其他载荷被跳过。

## 合成数据和完整测试

```bash
/usr/bin/python3 src/Omni-LIVO/scripts/generate_rig.py \
  --output artifacts/demo --duration 3

build/omni/devel/lib/omni_livo/rig_mapping \
  --config artifacts/demo/config.yaml --bag artifacts/demo/synthetic.bag \
  --output artifacts/demo/run

/usr/bin/python3 src/Omni-LIVO/scripts/test_rig_runtime.py \
  --binary build/omni/devel/lib/omni_livo/rig_mapping \
  --work artifacts/runtime_tests_new
```

生成器构造带连续纹理的三维房间、扫描内运动、8 个不同相机方向和带杆臂的双 IMU；同时输出配置和真值。测试执行正常融合、相机/雷达缺帧、IMU 掉线与饱和恢复、双 IMU 停流、重复运行及关闭视觉/第二 IMU 的消融。正常合成序列要求八相机、四雷达和两 IMU 均有有效贡献。

C++/Python 单元测试：

```bash
(cd build/omni && ctest --output-on-failure)
```

## ROS 在线入口

```bash
source /opt/ros/noetic/setup.bash
source build/omni/devel/setup.bash
roslaunch omni_livo mapping_rig.launch output:=/tmp/rig_live_new
```

另一个终端回放：

```bash
source /opt/ros/noetic/setup.bash
rosbag play --clock --rate 0.2 artifacts/hbs/*.bag
```

先采用较低回放速度验证链路。`use_sim_time:=false` 用于实际驱动输入。在线输出 `/rig/odometry`、`/rig/local_map` 和 `/rig/diagnostics`；离线入口不依赖 roscore，并将输出写入 bag。

隔离本地 ROS master 的传输测试：

```bash
/usr/bin/python3 src/Omni-LIVO/scripts/test_ros_transport.py \
  --binary build/omni/devel/lib/omni_livo/rig_mapping \
  --config artifacts/demo/config.yaml --bag artifacts/demo/synthetic.bag \
  --output artifacts/ros_transport_new
```

该测试启动并清理自己的 master、节点和播放器，不操作已有 ROS master。

## 估计与故障行为

- 状态维数为 `15 × 活动 IMU 数 + 3 + 启用相机数`，默认 41。每路 IMU 独立偏置和噪声，通过刚体相对位姿约束和交叉协方差共同估计。
- 单路 IMU 失效时边缘化到 26 维；恢复需连续健康样本和可积分时间区间，再重新增广到 41 维。两路均失效时停止有效里程计并报告 `tracking=false`，需重新启动进行初始化。
- 相机/雷达缺帧只减少对应观测。相机实际拍摄时间通过轨迹相对位姿参与投影，缺帧不复制旧图像冒充新观测。
- 机体 `B` 固定为 IMU 0 的安装坐标系，切换基准 IMU不会改变输出坐标系。`T_B_sensor` 把传感器点变换到机体。
- 雷达按真实逐点时间补偿到机体参考时刻，采用 SE(3) 三次累计 B 样条；边界只在已覆盖 IMU 区间内插值，拒绝无支撑查询。
- 点运动不确定度使用条件过程噪声近似，避免将完整位姿先验重复加入观测噪声；外参协方差接口已提供。
- 几何使用点到平面约束及退化加权；视觉使用带曝光参数的光度 patch、可见性筛选和跨视角参考迁移。两者更新完整联合状态。
- 每轮迭代保留同一个先验，协方差只在测量收敛后提交一次。
- 地图按半径和体素数量裁剪，视觉引用保留时间和数量有界，传感器队列有容量上限。
- 整体数据时钟回跳超过 1 秒时清空地图并重新初始化，输出新的 `trajectory_N.txt` 和 `estimates_N.bag`，不将不同时间段强行拼接。

## 结果和真实参数替换

每次运行输出：

| 文件 | 内容 |
|---|---|
| `trajectory.txt` | TUM 轨迹 |
| `map.pcd` | 非空局部几何地图 |
| `estimates.bag` | 里程计、协方差和最终地图 |
| `frames.csv` | 每轮点数、相机数、活动维数、残差数和耗时 |
| `summary.json` | 各传感器贡献、剔除、恢复及全局统计 |

`config/rig_hbs.yaml` 为示例配置，`config/rig_synthetic.yaml` 对应生成器。真实标定到位后替换逐传感器 `rotation`、`translation`、相机模型/内参、`time_offset`、IMU 噪声及 `extrinsic_std`，并将 `calibration_mode` 改为 `validated`。时间单位转换只在 HBS 适配层执行，标定时延只在估计器输入层应用一次。

当前实现以可运行框架和离线验证为目标。默认点数和视觉预算在当前电脑上尚未达到 1 倍速实时；耗时报告与精度验收分别记录。
