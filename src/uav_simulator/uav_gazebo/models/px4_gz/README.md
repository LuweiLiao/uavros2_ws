# PX4 Gazebo Sim / Harmonic 全机型

覆盖固定 PX4 版本中 **全部 25 个 `*_gz_*` SITL 机型配置**，包含 41 个
机体/共享传感器/辅助资源模型、25 个独立启动场景、airframe 参数快照和 PX4 专用插件。
范围只包含 Gazebo Sim/Harmonic，不含 Gazebo Classic、SIH、JSBSim 或 FlightGear。

模型和生成场景可离线加载，不依赖 PX4 源码目录或 Fuel 缓存。
闭环控制仍需另外编译的 PX4 SITL；本次完成模型移植和加载测试，**未进行飞行或驾驶验收**。

## 快速运行（ROS 2）

推荐沿用仓库的 ROS 2 Jazzy + Gazebo Harmonic 环境。在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --base-paths src \
  --packages-up-to uav_gazebo \
  --build-base .tmp/build/colcon --install-base .tmp/install/colcon
source .tmp/install/colcon/setup.bash
ros2 run uav_gazebo px4_sitl.py list
ros2 launch uav_gazebo px4_sitl.launch.py model:=standard_vtol
```

将 `model` 替换为下表任意名称。支持 `gui:=false`、`paused:=true`、
`partition:=自定义名称`。默认 partition 为 `tustin_px4`。
同一 partition 同时只运行本入口的一套场景；不同模型同时运行请使用不同 partition。
默认不启动飞控、不解锁、不执行飞行命令。

`uav_gazebo` 还声明了原仓库 RotorS/UAV 控制包依赖，因此首次 ROS 构建沿用
主 README 的 `--packages-up-to` 流程；不能在空安装目录只选这两个包。
只需要 PX4/Gazebo 且不想构建原 ROS 包时，使用下方“无 ROS 运行”。

另一终端连接已构建的 PX4（两个终端选择相同机型和 partition）：

```bash
source /opt/ros/jazzy/setup.bash
source .tmp/install/colcon/setup.bash
ros2 run uav_gazebo px4_sitl.py sitl --model standard_vtol \
  --px4-dir "$HOME/PX4-Autopilot"
```

先在匹配版本的 PX4 源码目录完成开发依赖安装和 `make px4_sitl`。
入口会查找 `build/px4_sitl_default/bin/px4` 与 airframe，自动设置
`PX4_GZ_STANDALONE=1`、正确的 `PX4_SYS_AUTOSTART`、世界名及已存在的模型实例名。
每次默认新建临时参数/日志目录；使用 `--run-dir` 可指定持久目录。
`--dry-run` 仅显示命令，不执行。不要再执行 `make px4_sitl gz_<model>` 生成第二架模型。

结束时先退出 PX4，再 Ctrl+C 关闭本轮 Gazebo/launch。PX4 操作使用对应机型的
QGroundControl/PX4 流程；仓库原有 ArduPilot GUIDED 和 UDP 9002/9003 参数不适用。

## 无 ROS 运行

先安装 Harmonic 开发依赖、CMake、Protobuf、OpenCV；GStreamer 开发库用于可选视频推流插件。
插件构建不需要整个 PX4 固件，也不会联网拉取子模块：

```bash
cmake -S src/px4_gz_plugins -B .tmp/px4-plugins-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/.tmp/px4-plugins-install"
cmake --build .tmp/px4-plugins-build -j4
cmake --install .tmp/px4-plugins-build
python3 src/uav_simulator/uav_gazebo/scripts/px4_sitl.py gazebo \
  --model x500_depth --plugin-dir .tmp/px4-plugins-install/lib/px4_gz_plugins
```

增加 `--headless` 可只运行 server 并采用 EGL 渲染传感器。连接飞控：

```bash
python3 src/uav_simulator/uav_gazebo/scripts/px4_sitl.py sitl \
  --model x500_depth --px4-dir "$HOME/PX4-Autopilot"
```

## 全部机型与验证记录

下表来自 `catalog.json`；编号从原 airframe 文件读取，不自行推测。
“默认”表示本机默认图形环境通过有限步数加载；“软件渲染”表示默认环境崩溃，
显式设置 `LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe` 后通过。
这两类结果都不是 PX4 飞行、驾驶或传感器精度验收。

| model | SYS_AUTOSTART | Gazebo 加载验证 |
| --- | ---: | --- |
| `x500` | 4001 | 默认 |
| `x500_depth` | 4002 | 软件渲染 |
| `rc_cessna` | 4003 | 默认 |
| `standard_vtol` | 4004 | 默认 |
| `x500_vision` | 4005 | 默认 |
| `px4vision` | 4006 | 默认 |
| `advanced_plane` | 4008 | 软件渲染 |
| `r1_rover` | 4009 | 默认 |
| `x500_mono_cam` | 4010 | 软件渲染 |
| `lawnmower` | 4011 | 默认 |
| `x500_lidar_2d` | 4013 | 软件渲染 |
| `x500_mono_cam_down` | 4014 | 软件渲染 |
| `x500_lidar_down` | 4016 | 软件渲染 |
| `x500_lidar_front` | 4017 | 软件渲染 |
| `quadtailsitter` | 4018 | 默认 |
| `x500_gimbal` | 4019 | 软件渲染 |
| `tiltrotor` | 4020 | 默认 |
| `x500_flow` | 4021 | 软件渲染 |
| `rover_differential` | 50000 | 默认 |
| `rover_ackermann` | 51000 | 默认 |
| `rover_mecanum` | 52000 | 默认 |
| `uuv_bluerov2_heavy` | 60002 | 默认 |
| `atmos` | 70000 | 默认 |
| `atmos_dual` | 70001 | 默认 |
| `omnicopter` | 8011 | 默认 |

多旋翼包含 X500 的深度、单目、下视、光流、雷达、云台和视觉里程计变体，
以及 PX4Vision 和 Omnicopter；固定翼含 Cessna/advanced_plane，VTOL 含
standard_vtol/quadtailsitter/tiltrotor；另有五类地面车、BlueROV2 Heavy 和两种 ATMOS。
`omniquad`、`spacecraft_2d` 等上游辅助/实验模型资源也已带入，但该固定 PX4 版本
没有对应 `*_gz_*` airframe，因此不伪造编号加入 SITL 启动菜单。

## 移植差异

- 保留上游质量、惯量、碰撞几何、执行器参数、控制顺序及传感器配置。
- 新增 `px4_gz_plugins` 独立包：光流（含 OpticalFlow/KLT）、空速、通用电机、
  航天器推力、电机失效、移动平台、浮力以及可选 GStreamer 系统。
  全量入口保留 X500 的 MotorFailurePlugin；之前的轻量 `px4_x500.launch` 仍保持原行为。
- Classic 材质脚本改为相同命名颜色的显式颜色；补充 `gz:` XML 命名空间，
  规范化 X500 depth 的 include URI。
- 修正上游 `rover_differential` 的后随动轮关节与 link 同名错误：只将 joint 改名为
  `cast_wheel_rear_joint`，link、位姿、动力学和驱动轮通道保持原值。
- ATMOS、ATMOS dual、freeflyer 和麦克纳姆轮网格固定 Fuel 版本并本地化。
  freeflyer 旧推力插件文件名映射为此次编译的 `libSpacecraftThrusterModelPlugin.so`。
- 场景从上游各机型默认 world 派生，显式加入所需世界系统；保留出生位姿。
  水下场景保留水密度和浮力，仅去掉需联网下载的 Portuguese Ledge 海底布景。
- `worlds/` 保留原始场景作为来源，其中部分仍引用远程地形；启动器只使用
  自包含的 `generated_worlds/`。这次没有移植所有 Fuel 地形库。
- 相机默认输出 Gazebo image/depth 话题；GStreamer 库可供另行配置推流，
  生成场景不自动向 QGroundControl 推送视频。光流世界显式加载 OpticalFlowSystem。

## 验证与环境限制

2026-10-02：8 个专用插件系统编译安装通过（包含本机可用的 GStreamer）；
25/25 SDF 解析、机型/airframe/实例映射和资源校验通过。
全部模型与生成场景的 `<uri>` 均可从仓库本地解析，无远程 URI。
ROS 2 Humble 下新 launch 的 25 个机型参数解析通过，安装后的 X500 launch
实际运行 10 秒并正常退出；仓库推荐环境仍为 Jazzy。
插件包另通过 colcon 构建；资源包通过直接 CMake 安装。当前 Humble 机器未完成
原工作区依赖链的全量 colcon 构建，Jazzy 全套构建需按主 README 的依赖说明执行。

本机默认 WSL D3D12/Ogre2 环境中 16 个机型加载通过，9 个渲染传感器机型出现
SIGSEGV；这 9 个在显式软件渲染对照中全部通过。默认启动器不设置软件渲染变量，
不宣称已通过 NVIDIA 原生 Linux GUI 验收。本机遇到同样问题时可在该终端显式设置：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
```

这是本机诊断的可用方式，性能不能代表硬件加速。SDF 的 `gz_frame_id` 扩展警告
来自上游，未删除传感器 frame 信息。完整机器可读结果见 `VALIDATION.json`。
没有已编译 PX4 SITL，本轮未进行闭环飞行、VTOL 转换、导航、驾驶或水下控制验收。

复查（仓库根目录）：

```bash
python3 tools/px4/check_gz_models.py
python3 -m unittest discover -s tools/px4/tests -v
python3 tools/px4/check_gz_models.py --runtime \
  --plugin-dir .tmp/px4-plugins-install/lib/px4_gz_plugins
(cd src/uav_simulator/uav_gazebo/models/px4_gz && sha256sum -c SHA256SUMS)
```

每个 runtime 测试使用独立 partition、100 个物理步和超时清理。
结果/日志默认保存在 `.tmp/px4_sitl/checks/`，不属于提交内容。

## 固定来源与许可证

- PX4-Autopilot：`23bd42a5bff02ce0d358877f852e04db879d59ec`。
- [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models/tree/bb0b9cf974acf4f1bcb5f5fcf80b88841562dea9)：
  `bb0b9cf974acf4f1bcb5f5fcf80b88841562dea9`。
- Fuel：proque 的 `atmos` / `atmos_dual` / `kth_freeflyer` v1（CC BY 4.0），
  OpenRobotics 的 Mecanum lift v5 左右轮网格（CC0）；各目录保留署名与许可证链接。
- `PROVENANCE.json` 记录下载来源、版本、原始归档 SHA-256 和修改文件列表；
  `SHA256SUMS` 校验移植后的资源。插件及其第三方依赖另有原许可证和来源记录。
- [PX4 官方 Gazebo 说明](https://docs.px4.io/main/en/sim_gazebo_gz/index)。

本目录是固定快照，不声称覆盖未来上游新增机型。维护者可使用
`tools/px4/import_gz_models.py --px4-source <checkout> --fuel-cache <cache>`
重新生成；cache 需包含 PROVENANCE 对应的三份 Fuel JSON/ZIP、Mecanum lift JSON
以及左右轮 STL。该命令会更新本目录的生成资源，修改后应重新运行检查并更新验证记录。
