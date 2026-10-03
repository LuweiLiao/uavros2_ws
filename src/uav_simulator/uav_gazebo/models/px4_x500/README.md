# PX4 X500 四旋翼

将 PX4 的 Gazebo Sim X500 接入本仓库。包含完整 `x500`、`x500_base`
模型及网格、纹理、许可证；无需从 Fuel 下载资源。使用 Gazebo 原生电机系统
和 PX4 `gz_bridge`，不使用本仓库的 ArduPilot UDP 9002/9003 接口。

## 来源与适配

- 来源：[PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models/tree/bb0b9cf974acf4f1bcb5f5fcf80b88841562dea9)，
  提交 `bb0b9cf974acf4f1bcb5f5fcf80b88841562dea9`。
- 从本机 PX4 子模块复制；对应 PX4-Autopilot 提交
  `23bd42a5bff02ce0d358877f852e04db879d59ec`。这是固定源码快照，不承诺自动跟随上游最新版本。
- 保留质量、惯量、碰撞体、四路电机参数、旋翼顺序和 IMU/气压计/磁力计/GNSS。
- 移除 `MotorFailurePlugin`：该插件需要 PX4 额外编译产物；此基础模型不提供电机故障注入。
- 四处 Classic `Gazebo/DarkGrey` 材质改为显式 ambient/diffuse，避免旧材质库依赖。
- `px4_x500.world` 基于上游 `worlds/default.sdf`；显式加入物理、场景、
  用户命令和四种传感器系统，避免依赖 PX4 的外部 `server.config`。
  世界名 `px4_x500`，顶层模型名 `x500_0`，出生高度 0.24 m。
- 原作者许可证保留于本目录及两个模型目录。运行不依赖本机 PX4 源码绝对路径。
- `SHA256SUMS` 记录移植后的模型资源校验值，在本目录运行 `sha256sum -c SHA256SUMS` 可检查。

## 直接运行 Gazebo（无需 ROS）

在本仓库根目录执行，要求已安装 Gazebo Harmonic：

```bash
export GZ_PARTITION=tustin_px4_x500
export GZ_SIM_RESOURCE_PATH="$PWD/src/uav_simulator/uav_gazebo/models/px4_x500/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r src/uav_simulator/uav_gazebo/worlds/px4_x500.world
```

无 GUI 可增加 `-s`。仅启动 Gazebo 时飞机停在地面，闭环控制需要下一节的 PX4。

## ROS 2 入口

按仓库主 README 配置 Jazzy + Harmonic。在仓库根目录构建资源包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select uav_gazebo \
  --build-base .tmp/build/colcon --install-base .tmp/install/colcon
source .tmp/install/colcon/setup.bash
export GZ_PARTITION=tustin_px4_x500
ros2 launch uav_gazebo px4_x500.launch
```

支持 `gui:=false`、`paused:=true`；连接 SITL 前应恢复仿真。
独立入口只调用 `ros_gz_sim`，无需构建 RotorS 或 ArduPilot 自定义插件。
完成原仓库全套所需包构建后，也可使用原入口：

```bash
ros2 launch uav_gazebo spawn.launch world_name:=px4_x500
```

两个入口任选其一。已有 CMake 安装规则会自动安装新增模型、world 和 launch。

## 连接 PX4 SITL

先在自己的 PX4 源码目录准备开发依赖并运行 `make px4_sitl`。
上述固定源码的 X500 airframe 为 `4001_gz_x500`；使用其他版本时检查该文件编号。
Gazebo 启动后，在第二个终端执行：

```bash
export GZ_PARTITION=tustin_px4_x500
export PX4_GZ_STANDALONE=1
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_WORLD=px4_x500
export PX4_GZ_MODEL_NAME=x500_0
unset PX4_SIM_MODEL
px4_source="$HOME/PX4-Autopilot"  # 按实际位置修改
px4_run_dir=$(mktemp -d /tmp/tustin-px4-x500-XXXXXX)
cd "$px4_run_dir"
"$px4_source/build/px4_sitl_default/bin/px4" \
  -s etc/init.d-posix/rcS -w "$px4_run_dir" \
  "$px4_source/build/px4_sitl_default/etc"
```

绑定的是世界中已存在的 `x500_0`，不要另用 `make px4_sitl gz_x500`
再生成一架飞机。飞行操作使用 PX4/QGroundControl 流程；原 README 的
ArduPilot GUIDED、MAVProxy 参数不适用于这个模型。
结束时先退出 PX4，再停止 Gazebo；不同终端保持同一 `GZ_PARTITION`。

参考：[PX4 官方 Gazebo 与 standalone 说明](https://docs.px4.io/main/en/sim_gazebo_gz/index)。

## 本次验证与限制（2026-10-02）

- Gazebo Sim 8.9.0：SDF 解析通过，独立 server 加载并推进仿真。
- 使用本机 Humble 的 ament_cmake 完成资源包配置与安装；网格/纹理与固定上游
  文件逐字节比对通过，仅两份 SDF 有上述适配。此检查不等同于 Jazzy 运行验收。
- 确认 IMU、气压计、磁力计、GNSS 四类话题及电机命令话题存在；
  实际读取 IMU 数据，静止时 z 加速度约 9.80 m/s²。
- `gz_frame_id` 是上游传感器扩展，SDF 校验会提示保留未知元素；不影响此次加载。
- 当前机器只有 ROS 2 Humble，缺少 `ros_gz_sim` 和已编译 PX4 SITL；
  未完成 Jazzy launch、GUI 视觉检查或 PX4 起飞/悬停/降落验证。

静态检查（仓库根目录）：

```bash
SDF_PATH="$PWD/src/uav_simulator/uav_gazebo/models/px4_x500/models" \
  gz sdf -k src/uav_simulator/uav_gazebo/worlds/px4_x500.world
```
