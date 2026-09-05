# uav_simulator

`uav_simulator` 是基于 ROS Noetic、Gazebo Classic 和 ArduPilot SITL 的无人机仿真模型仓库。仓库中包含多种飞行器模型、Gazebo 插件、仿真 world，以及用于从 `.rsdf` 模板生成 `.sdf` 模型文件的任务配置。

推荐运行环境：

- Ubuntu 20.04 / WSL2 Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11
- ArduPilot SITL
- `catkin_tools`
- `task` 和 `erb`

## 目录结构

```text
uav_simulator/
├── mav_msgs/                 # MAV 相关消息定义
├── uav_gazebo/               # Gazebo 模型、world 和 launch 文件
│   ├── launch/
│   │   └── spawn.launch      # 通用 Gazebo 启动入口
│   ├── models/
│   │   ├── pendulum_quadcopter/  # 倒立摆四旋翼模型
│   │   ├── tilt_tricopter/       # 倾转三旋翼模型
│   │   ├── tilt_quadcopter/      # 倾转四旋翼模型
│   │   ├── usl_bicopter/         # 双旋翼模型
│   │   └── ...
│   └── worlds/               # 各模型对应的 Gazebo world
└── uav_gazebo_plugin/        # ArduPilot/Gazebo 通信和模型控制插件
```

模型目录通常包含：

```text
model.rsdf     # ERB 模板，推荐修改这个文件
model.sdf      # 由 model.rsdf 生成，Gazebo 实际加载
model.config   # Gazebo 模型描述
meshes/        # STL/DAE 等网格文件
```

## 环境安装

安装 ROS Noetic 和常用依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-joy \
  ros-noetic-octomap-ros \
  python3-catkin-tools \
  python3-wstool \
  protobuf-compiler \
  libgeographic-dev \
  ros-noetic-geographic-msgs \
  libgoogle-glog-dev
```

安装 ArduPilot 依赖请参考 ArduPilot 官方文档，也可以在 ArduPilot 仓库中执行：

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

## 工作区构建

在 `uavros_ws` 根目录执行：

```bash
cd ~/uavros_ws
catkin init
catkin build
source devel/setup.bash
```

建议把环境写入 shell 配置：

```bash
echo "source ~/uavros_ws/devel/setup.bash" >> ~/.bashrc
```

如果不通过 `spawn.launch` 自动设置模型路径，也可以手动配置：

```bash
export GAZEBO_MODEL_PATH=~/uavros_ws/src/uav_simulator/uav_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/uavros_ws/src/uav_simulator/uav_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
```

## 模型生成

本仓库大量模型使用 `.rsdf` 模板维护，修改模型时优先改 `.rsdf`，然后重新生成 `.sdf`。

在 `uavros_ws` 根目录执行，例如：

```bash
task erb-pendulum_quadcopter:pendulum_quadcopter
task erb-usl_bicopter:usl_bicopter_multi
```

常用任务包括：

```text
erb-pendulum_quadcopter:pendulum_quadcopter
erb-pendulum_quadcopter:pendulum_pole
erb-pendulum_quadcopter:pendulum_quadcopter_base
erb-pendulum_quadcopter:pendulum_quadcopter_prop

erb-usl_bicopter:usl_bicopter
erb-usl_bicopter:usl_bicopter_multi

erb-tsduav_quad:tsduav_quad
erb-tilt_tricopter:tilt_tricopter
```

生成后建议重新编译相关插件：

```bash
catkin build uav_gazebo_plugin
```

## 启动 Gazebo 仿真

通用启动入口：

```bash
roslaunch uav_gazebo spawn.launch world_name:=<world_name>
```

`world_name` 对应 `uav_gazebo/worlds/` 下的 world 文件名，不包含 `.world` 后缀。

示例：

```bash
roslaunch uav_gazebo spawn.launch world_name:=pendulum_quadcopter
roslaunch uav_gazebo spawn.launch world_name:=tilt_tricopter
roslaunch uav_gazebo spawn.launch world_name:=usl_bicopter
```

如果 Gazebo 无法找到模型，优先检查：

- 是否已经 `source ~/uavros_ws/devel/setup.bash`
- `spawn.launch` 中对应模型目录是否加入 `GAZEBO_MODEL_PATH`
- `.rsdf` 修改后是否重新生成 `.sdf`

## 连接 ArduPilot SITL

以 Copter 为例，在 ArduPilot 仓库中启动 SITL：

```bash
cd <ardupilot>/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

不同模型需要匹配对应的 Gazebo 插件端口。多机 world 中通常会使用不同的 `fdm_port_in` / `fdm_port_out`。

倒立摆四旋翼使用圆杆 ODOMETRY 时，推荐让 ArduPilot SERIAL2 作为 TCP server：

```bash
../Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --console \
  --add-param=mav.parm \
  --add-param=pendulum.parm \
  -A "--serial2=tcp:14557"
```

Gazebo 侧 `PoleMavlinkPosePlugin` 会作为 TCP client 连接 `127.0.0.1:14557`，发送圆杆 `ODOMETRY`。

## 倒立摆四旋翼

倒立摆相关文件：

```text
uav_gazebo/models/pendulum_quadcopter/
├── models/pendulum_quadcopter/       # 四旋翼总装模型
├── models/pendulum_quadcopter_base/  # 机体和顶部圆槽
├── models/pendulum_quadcopter_prop/  # 桨叶模型
└── models/pendulum_pole/             # 独立圆杆模型
```

相关插件：

```text
PoleHoldReleasePlugin   # 锁住/释放圆杆，支持 set_position_ned 重置位置
PoleMavlinkPosePlugin   # 将圆杆位置速度通过 MAVLink ODOMETRY 发给 ArduPilot
```

常用 ROS 命令：

```bash
# 调整圆杆位置，坐标为 ArduPilot/Gazebo 本地 NED，z 为负表示上方
rostopic pub /pendulum_pole/set_position_ned geometry_msgs/PointStamped "header:
  frame_id: 'local_ned'
point:
  x: 0.0
  y: 0.0
  z: -5.0" -1

# 释放圆杆
rostopic pub /pendulum_pole/unlock std_msgs/Bool "data: true" -1

# 重新锁住圆杆
rostopic pub /pendulum_pole/unlock std_msgs/Bool "data: false" -1
```

注意：修改插件或模型后，需要重新编译/生成，并重启 Gazebo 或重新 spawn 模型。

## 双旋翼模型

双旋翼模型位于：

```text
uav_gazebo/models/usl_bicopter/
```

常用 world：

```bash
roslaunch uav_gazebo spawn.launch world_name:=usl_bicopter
roslaunch uav_gazebo spawn.launch world_name:=usl_bicopter_multi
```

对应插件：

```text
ArduRotorBicopter
```

该插件负责与 ArduPilot SITL 交换 FDM 数据，并发布电机转速和倾转舵机命令。

## 常见问题

### Gazebo 启动后找不到模型

检查 `GAZEBO_MODEL_PATH`，或确认 `spawn.launch` 中模型路径是否包含目标模型目录。

### 修改 rsdf 后 Gazebo 仍显示旧模型

需要重新生成 `.sdf`，并重启 Gazebo：

```bash
task erb-pendulum_quadcopter:pendulum_quadcopter
```

### 插件编译通过但 Gazebo 仍使用旧逻辑

Gazebo 已加载的 `.so` 不会自动刷新，需要重启 Gazebo。

### 圆杆 ODOMETRY 没有进入 ArduPilot

检查：

- SITL 是否使用 `-A "--serial2=tcp:14557"`
- `SERIAL2_PROTOCOL` 是否为 MAVLink2
- Gazebo 插件中 `tcpAddr/tcpPort` 是否为 `127.0.0.1/14557`
- `PEND_SYSID` 是否和 `PoleMavlinkPosePlugin` 的 `systemId` 一致

## 维护建议

- 修改模型优先改 `.rsdf`，再生成 `.sdf`。
- 新增模型后同步更新 `Taskfile.yml` 的生成任务。
- 新增模型目录后检查 `spawn.launch` 的 `GAZEBO_MODEL_PATH`。
- 新增插件后更新 `uav_gazebo_plugin/CMakeLists.txt` 并运行 `catkin build uav_gazebo_plugin`。
