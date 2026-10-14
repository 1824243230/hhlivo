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

在工作区根目录执行。转换器始终将所选时间范围内的全部传感器消息写入 **一个 bag 文件**，不按大小生成 `rig_001.bag` 等分段，原来的 `--split-gb` 参数已移除。不指定 `--start`、`--duration` 时转换完整 HBS。

### 1. 转换为单个 bag

```bash
/usr/bin/python3 src/Omni-LIVO/scripts/hbs_multi.py \
  hbus_0002_20260917111501752LLmineLLliudongLL22.04_bak.hbs \
  --output artifacts/hbs/rig.bag
```

成功后生成 `artifacts/hbs/rig.bag` 和 `artifacts/hbs/rig.report.json`。报告的 `status` 应为 `complete`，`output_bags` 仅包含这个 bag。转换过程中先写入 `rig.bag.partial`，全部消息写入并关闭后才改名为 `rig.bag`；失败时查看报告中的 `error`，未完成的 `.partial` 不能作为完整转换结果使用。

输出 bag、报告或 `.partial` 已存在时，程序拒绝覆盖，请选择新的文件名或目录。旧版本生成的分段不会自动合并或删除；按上述命令从源 HBS 重新转换，并仅使用新的单个 bag。

8 路相机解码会生成临时 BGR 图像，解码后的数据通常远大于压缩 HBS。请为临时图像和完整 bag 留出足够磁盘空间；可用 `--temp-dir /有足够空间的目录` 指定临时目录，默认由系统选择，转换结束后自动清理。`--compression` 默认 `lz4`，也可选 `none` 或 `bz2`；压缩方式不改变单文件输出。

### 2. 校验 bag

```bash
/usr/bin/python3 src/Omni-LIVO/scripts/verify_rig_bag.py \
  artifacts/hbs/rig.bag \
  --output artifacts/hbs/validation.json
```

校验通过时，`validation.json` 的 `status` 为 `passed`，包含 14 路传感器的消息数量、图像尺寸及雷达点数，并检查消息字段与时间顺序。

本次验证（2026-10-14）：上述 `hbus_0002_20260917111501752LLmineLLliudongLL22.04_bak.hbs` 已转换为唯一的 `artifacts/hbs/rig.bag`，大小约 15.98 GiB，14 路传感器共 14,118 条消息，全量读回校验通过。相机有 45 个时间戳未解码出图像，解码器报告部分 H.265 数据无效；各实例的具体时间戳和错误原因保存在 `rig.report.json` 的 `cameras` 字段中。

### 3. 离线定位并检查结果

```bash
build/omni/devel/lib/omni_livo/rig_mapping \
  --config src/Omni-LIVO/config/rig_hbs.yaml \
  --bag artifacts/hbs/rig.bag --output artifacts/hbs/run

/usr/bin/python3 src/Omni-LIVO/scripts/verify_rig_run.py \
  artifacts/hbs/run --conversion-report artifacts/hbs/rig.report.json --require-all
```

定位输出目录已经存在时，请选择新的输出目录。程序不覆盖已有结果。

转换按相机实例保持独立 H.265 解码器，记录原始 PTS；采用磁盘中间文件，内存不会随所有解码图像增长。`--start`、`--duration` 可截取，但仍从前面的编码帧建立解码参考。即使截取也只生成一个 bag；若所选区间没有可输出的消息，转换失败并在报告中说明原因。

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

仅验证 HBS 转换器，无需构建 C++：

```bash
/usr/bin/python3 -m unittest discover \
  -s src/Omni-LIVO/tests -p test_hbs_multi.py -v
```

覆盖单文件写入与 14 路消息读回、超过旧 2 GiB 阈值仍不分包、已有输出保护、失败时保留临时 bag，以及空时间区间报错。

## ROS 在线入口

在工作区根目录执行；首次使用或修改 C++ 后先运行 `bash src/Omni-LIVO/scripts/build_rig.sh`：

```bash
cd ~/hyhLIOV
source /opt/ros/noetic/setup.bash
source build/omni/devel/setup.bash
roslaunch omni_livo mapping_rig.launch output:=/tmp/rig_live_new
```

该命令默认同时打开 RViz，加载 `rviz_cfg/rig.rviz`，固定坐标系为 `rig_world`。默认显示按高度着色的局部地图、黄色连续轨迹、机体姿态坐标轴和相机 0 图像。左侧 `Other Cameras` 可勾选相机 1～7；`Raw LiDAR (optional)` 可勾选四路原始点云；`Sensor Frames (optional)` 可显示传感器坐标系。原始点云为未去畸变的传感器输入，定位地图显示在 `Local Map` 中。`Views` 的 `Follow Rig` 可切换为跟随机体的视角。

RViz 打开后需要传感器数据才能显示地图和轨迹。另一个终端从工作区根目录回放：

```bash
cd ~/hyhLIOV
source /opt/ros/noetic/setup.bash
rosbag play --clock --rate 0.2 artifacts/hbs/rig.bag
```

回放仅传入新的单个 `rig.bag`，先采用较低回放速度验证链路。启动后、回放前没有 `/clock` 和估计结果时，RViz 暂时提示无数据/无坐标系属于正常等待状态。输出目录已有结果时，请换一个新目录。

`use_sim_time:=false` 用于实际驱动输入。无需 GUI 时加 `rviz:=false`；自定义 RViz 配置时加 `rviz_config:=/绝对路径/custom.rviz`。在线输出 `/rig/odometry`、`/rig/local_map`、`/rig/path` 和 `/rig/diagnostics`，并发布 `rig_world → rig_body` 动态 TF，以及由配置中的 `T_B_sensor` 外参生成的 `rig_body → imu_N/lidar_N/camera_N` 静态 TF。轨迹可视化保留最近 2000 个位姿，完整轨迹仍保存于 `trajectory.txt`；数据时钟回跳重新初始化时清空显示轨迹和地图。示例配置的外参仍是占位值，原始点云显示也使用这些值。

离线入口不依赖 roscore，并将输出写入 bag。

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
