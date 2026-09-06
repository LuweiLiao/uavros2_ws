# UAVROS 2

**保留 ROS 1 架构的无人机仿真工作区 · A ROS 2 UAV simulation workspace preserving the ROS 1 architecture**

UAVROS 2 将原 `uavros_ws` 迁移到 ROS 2 Jazzy 与 Gazebo Harmonic，结合原 RotorS
旋翼模型和 ArduPilot SITL，实现带完整 Gazebo GUI 的无人机闭环仿真。
原包名、相对目录、模型嵌套、launch 文件名、执行器顺序及插件职责保持不变；
只进行必要的兼容适配，并记录差异。

UAVROS 2 ports `uavros_ws` to ROS 2 Jazzy and Gazebo Harmonic, combining the
original RotorS motor models with ArduPilot SITL and the full Gazebo GUI.
Package names, relative paths, nested models, launch filenames, actuator ordering
and plugin responsibilities are preserved. Required compatibility changes are documented.

> 当前已验证 `tsduav_quad` 和 `tilt_quadcopter` 的 GUI + SITL 飞行切片，
> **不是整个 ROS 1 工作区已完成移植**。仅用于仿真，不代表实机安全认证。
>
> GUI + SITL flight has been validated for `tsduav_quad` and `tilt_quadcopter`.
> **The full workspace is not yet ported.** These are simulation results, not
> real-aircraft safety certification.

## 目录 / Contents

- [环境与范围 / Platform and scope](#platform)
- [安装与编译 / Installation and build](#installation)
- [准备 ArduPilot / ArduPilot setup](#ardupilot)
- [运行普通四旋翼 / Run tsduav_quad](#quad)
- [运行倾转四旋翼 / Run tilt_quadcopter](#tilt)
- [结束仿真 / Shutdown](#shutdown)
- [ROS 接口与记录 / ROS interfaces and recording](#interfaces)
- [架构与开发 / Architecture and development](#development)
- [验证与限制 / Validation and limitations](#validation)
- [常见问题 / Troubleshooting](#troubleshooting)
- [官方文档与来源 / References and provenance](#references)

<a id="platform"></a>
## 环境与范围 / Platform and scope

| 组件 / Component | 当前验证环境 / Validated environment |
| --- | --- |
| OS | Ubuntu 24.04 LTS，原生桌面 / native desktop |
| ROS | ROS 2 Jazzy |
| Gazebo | Harmonic，Gazebo Sim 8.11.0 |
| ArduPilot | `LuweiLiao/ardupilot`，`staging/tritilt-fixed`；验收提交 / tested commit: `99a9622610de489d65b441e2d9efe46f453bdb0d` |
| Plugins | 原 RotorS 和 UAVROS 插件的 Harmonic 移植 / Harmonic ports of the original RotorS and UAVROS plugins |
| Vehicles | `tsduav_quad`：四旋翼 / quadrotor；`tilt_quadcopter`：四臂八桨、四倾转舵机 / four arms, eight rotors, four tilt servos |

ROS 2 直接在宿主机运行，**不依赖 Docker**。历史 ROS 1 对照使用 Noetic / Classic
容器，不是本指南的运行环境。Jazzy + Harmonic 是官方配套组合，不混用 Garden、Jetty 或 Classic 插件。

ROS 2 runs **natively, without Docker**. Historical ROS 1 comparisons used a
Noetic / Classic container. Jazzy + Harmonic is the official pairing; Garden,
Jetty and Classic plugin binaries are not interchangeable with this build.

<a id="installation"></a>
## 安装与编译 / Installation and build

示例使用 Bash，两个仓库分别位于 `~/Projects/uavros2_ws` 和 `~/Projects/ardupilot`。
已有目录请跳过克隆，不覆盖原 ROS 1 工作区。已完成准备的用户可直接进入运行章节。

Examples use Bash and sibling checkouts under `~/Projects`. Skip cloning existing
repositories; do not overwrite ROS 1. If setup is complete, go directly to the run instructions.

### 1. 安装依赖 / Install dependencies

先按 [ROS 2 Jazzy Ubuntu 安装文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
配置官方软件源，再安装桌面环境与以下依赖。`ros-jazzy-ros-gz` 提供官方 ROS / Gazebo 集成。

First configure the official ROS apt repository using the linked guide, then
install Jazzy Desktop and these dependencies. `ros-jazzy-ros-gz` provides the official integration.

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
  ros-jazzy-mavlink git-lfs ripgrep build-essential cmake \
  libprotobuf-dev protobuf-compiler
source /opt/ros/jazzy/setup.bash
```

### 2. 获取源码与模型 / Clone source and models

```bash
mkdir -p ~/Projects
cd ~/Projects
git lfs install
git clone --branch codex/tilt-quadcopter-ros2 \
  https://github.com/LuweiLiao/uavros2_ws.git
cd uavros2_ws
git lfs pull
git lfs fsck
```

RotorS、`mav_comm`、`uav_simulator` 和 TSD 模型均按原路径直接纳入本仓库，
**没有 Git 子模块**。大型资源使用 Git LFS；ZIP 下载或跳过 LFS 可能只得到网格指针文本。

RotorS, `mav_comm`, `uav_simulator` and TSD models are vendored at their original
paths, with **no Git submodules**. Large assets use Git LFS; ZIP downloads or
checkouts without LFS may contain pointers instead of meshes.

### 3. 解析当前六包依赖 / Resolve the six-package dependency set

首次使用 rosdep 时先执行一次 `sudo rosdep init`；已初始化则跳过。
以下仅检查两个模型所需的六包，不把整个未完成移植的工作区当作已可构建。

Run `sudo rosdep init` once if not initialized. This checks the six packages
required by the two vehicles, not the incomplete full workspace.

```bash
cd ~/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths \
  src/mav_comm/mav_msgs \
  src/rotors_simulator/rotors_comm \
  src/rotors_simulator/rotors_gazebo_plugins \
  src/uav_simulator/uav_control \
  src/uav_simulator/uav_gazebo_plugin \
  src/uav_simulator/uav_gazebo \
  --ignore-src --rosdistro jazzy \
  --skip-keys "libprotobuf-dev protobuf-compiler" -y
```

两个 Protobuf rosdep 键在当前环境无法解析，已在第 1 步用 apt 安装；并非依赖可省略。

The two Protobuf rosdep keys do not resolve in the checked environment. Their
packages were explicitly installed in step 1; they are not optional.

### 4. 编译 / Build

使用只加载系统 ROS 的新终端，不加载正在重编译的工作区 overlay。

Use a fresh terminal with only the system ROS underlay, not the overlay being rebuilt.

```bash
cd ~/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
colcon build --base-paths src --packages-up-to uav_gazebo --symlink-install \
  --build-base .tmp/build/colcon \
  --install-base .tmp/install/colcon \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

这是标准 colcon 流程，仅保留已有 build/install 输出位置；`.tmp/install/colcon`
**不是 colcon 默认路径**。构建日志仍在默认 `log/`。运行时只需 source 一个统一安装空间，
不再叠加多个带日期的临时构建。

This is standard colcon with the existing project build/install locations;
`.tmp/install/colcon` is **not colcon's default path**. Build logs remain in `log/`.
Runtime terminals source one unified installation, not dated diagnostic overlays.

<a id="ardupilot"></a>
## 准备 ArduPilot / ArduPilot setup

ArduPilot 保持独立仓库；倾转控制来自指定分支。远端分支会更新，复现验收请固定到上表提交。
以下切换提交仅用于新克隆；已有仓库先检查本地修改，不直接照抄切换命令。

ArduPilot remains separate; tilt control comes from the specified branch. The
remote branch moves, so pin the tested revision for reproduction. The checkout
below is for a new clone only; inspect changes before switching an existing checkout.

```bash
cd ~/Projects
git clone --branch staging/tritilt-fixed \
  https://github.com/LuweiLiao/ardupilot.git
cd ardupilot
git checkout --detach 99a9622610de489d65b441e2d9efe46f453bdb0d
git submodule update --init --recursive
```

这里的子模块是 **ArduPilot 自身构建依赖**，不是在 `uavros2_ws` 重新引入子模块。
按 [ArduPilot Linux 构建指南](https://ardupilot.org/dev/docs/building-setup-linux.html)
安装环境；已有环境可跳过安装脚本。Ubuntu 24.04 通常创建 `~/venv-ardupilot`，
也可能复用已有虚拟环境，以脚本输出的激活路径为准，以下按默认路径示例。

These are **ArduPilot's own build dependencies**, not `uavros2_ws` submodules.
Follow the official Linux setup guide; skip the prerequisite script if already
configured. On Ubuntu 24.04 it normally creates `~/venv-ardupilot`, but may reuse
an existing environment. Use the activation path it reports; examples assume the default.

```bash
cd ~/Projects/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/venv-ardupilot/bin/activate
./waf configure --board sitl
./waf copter
```

**协议必须用 `--model Gazebo`**：本项目保留原插件的二进制 UDP 协议。
官方新 `ardupilot_gazebo` 示例的 `--model JSON` 对应另一套插件，不能直接照搬，
也不需要安装该插件来替换原插件。`-f gazebo-iris` 仅提供基础参数，不会将模型换成 Iris。

**Use `--model Gazebo`** for the original binary UDP protocol. The `--model JSON`
option in modern `ardupilot_gazebo` examples targets a different plugin; do not
install it as a replacement. `-f gazebo-iris` selects base defaults, not a different Gazebo model.

<a id="quad"></a>
## 运行普通四旋翼 / Run `tsduav_quad`

准备完成后日常只需两个终端：**先 Gazebo，后 SITL**。确认上一轮 GUI、server、SITL
已退出；每次只运行一个模型，需要可见桌面会话。

After setup, daily use needs two terminals: **Gazebo first, then SITL**. Confirm
the previous GUI, server and SITL have exited. Run one model at a time in a desktop session.

### 终端 1：Gazebo GUI / Terminal 1: Gazebo GUI

```bash
source ~/Projects/uavros2_ws/.tmp/install/colcon/setup.bash
ros2 launch uav_gazebo spawn.launch world_name:=tsduav_quad gui:=true paused:=false
```

原名 launch 调用官方 `ros_gz_sim/gz_sim.launch.py`，自动配置模型和插件路径，
启动完整默认 GUI 和 server；**不启动 SITL、MAVProxy 或 MAVROS**。
等待模型加载、仿真时间推进后再继续。可见世界坐标轴是视觉标记，不是 ROS TF。

The original launch invokes official `ros_gz_sim/gz_sim.launch.py`, sets resource
and plugin paths, and starts the default GUI and server. **It does not start SITL,
MAVProxy or MAVROS.** Wait for loading and advancing simulation time. Visible world axes are not ROS TF.

### 终端 2：SITL 与 MAVProxy / Terminal 2: SITL and MAVProxy

```bash
source ~/venv-ardupilot/bin/activate
cd ~/Projects/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model Gazebo --speedup 1 -N --use-dir tsduav_quad
```

官方 `sim_vehicle.py` 启动已编译的 SITL 和 MAVProxy；`-N` 跳过重编译。
官方 `--use-dir` 按模型隔离 `eeprom.bin` 等持久状态，不是自制启动器或统一日志收集。
不要再启动另一份 SITL、MAVProxy 或测试客户端抢占 TCP 5760。

Official `sim_vehicle.py` starts the built SITL and MAVProxy; `-N` skips rebuilding.
Official `--use-dir` isolates persistent SITL state by model, not centralized logging.
Do not start another SITL, MAVProxy or test client competing for TCP 5760.

### 起飞 / Take off

以下是 **MAVProxy 提示符命令，不是 Bash**。逐条执行，确认结果再继续。
核对原模型 yaw P=0.12，等待 GPS/Home/EKF 和预检通过，再切 GUIDED、解锁、起飞。
不要整份加载含实机标定的历史参数。

These are **MAVProxy prompt commands, not Bash**. Enter individually and check
results. Set the original yaw P to 0.12, wait for GPS/Home/EKF and pre-arm readiness,
then enter GUIDED, arm and take off. Do not load an entire legacy hardware parameter dump.

```text
param set ATC_RAT_YAW_P 0.12
param show ATC_RAT_YAW_P
mode GUIDED
arm throttle
takeoff 0.5
```

观察悬停后按[结束仿真](#shutdown)正常降落并关闭。

Observe hover, then [land and shut down](#shutdown) normally.

<a id="tilt"></a>
## 运行倾转四旋翼 / Run `tilt_quadcopter`

这是四臂八桨、四倾转舵机模型，**不是 `tilt_quadcopter_fix`**。使用上文指定固件和专用参数，
不要沿用普通四旋翼的 SITL 状态。

This eight-rotor, four-tilt-servo model is **not `tilt_quadcopter_fix`**. Use the
specified firmware and vehicle profile, with separate SITL state.

### 1. 一次性准备参数 / Prepare parameters once

打开[模型说明](src/uav_simulator/uav_gazebo/models/tilt_quadcopter/readme.md)
的“可重建的 SITL 参数参考”，用编辑器将该节完整代码块保存为
`~/Projects/uavros2_ws/.tmp/tilt_quadcopter/tiltquad_sitl.parm`，不要包含 Markdown 围栏。

Open the [model notes](src/uav_simulator/uav_gazebo/models/tilt_quadcopter/readme.md),
find “可重建的 SITL 参数参考” (reconstructible SITL parameter reference), and save
the complete parameter block, without Markdown fences, to the path above using an editor.

```bash
mkdir -p ~/Projects/uavros2_ws/.tmp/tilt_quadcopter
```

参数全文已纳入 Git，生成的本地 `.parm` 没有纳入；新克隆需保存一次。
此步骤保留原文件架构，不依赖某台机器的历史验收目录。原通道顺序不变；
当前固件的舵机反向为布尔语义：RF/RR/LR=1、LF=0，不能直接沿用旧 ±1。

The parameter text is tracked; the generated `.parm` is not. Prepare it once on a
fresh clone, preserving the source layout without relying on historical test folders.
Channel order is unchanged; current servo reversal is boolean: RF/RR/LR=1, LF=0, not old ±1.

### 2. 启动 / Launch

终端 1 / Terminal 1:

```bash
source ~/Projects/uavros2_ws/.tmp/install/colcon/setup.bash
ros2 launch uav_gazebo spawn.launch world_name:=tilt_quadcopter gui:=true paused:=false
```

模型加载并运行后，终端 2 / After Gazebo is ready, terminal 2:

```bash
source ~/venv-ardupilot/bin/activate
cd ~/Projects/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model Gazebo --speedup 1 -N --use-dir tilt_quadcopter \
  --add-param-file ~/Projects/uavros2_ws/.tmp/tilt_quadcopter/tiltquad_sitl.parm
```

`eeprom.bin` 中已保存的参数可能覆盖默认值。首次选没有旧状态的目录，以后核对实际参数；
不要盲目 `--wipe`。各工具保留正常日志行为，不设置集中输出环境变量。

Persisted `eeprom.bin` values may override defaults. Start without old state and
check effective parameters on later runs; do not blindly use `--wipe`. Tools retain
normal logging behavior without centralized output environment overrides.

### 3. 检查与起飞 / Check and take off

在 MAVProxy 确认 `FRAME_CLASS=7`、`FRAME_TYPE=23`、`MOT_TILT_EN=1`、
`MOT_PIT_OFF_MAX=90`、`ARMING_SKIPCHK=0` 和上述舵机方向。
等待 GPS/Home/EKF 和预检通过，以下逐条执行，不要整段粘贴跳过检查。

Verify these values and servo reversals in MAVProxy. Wait for GPS/Home/EKF and
pre-arm readiness. Execute individually, not as a pasted batch that skips checks.

```text
param show FRAME_CLASS
param show FRAME_TYPE
param show MOT_TILT_EN
param show MOT_PIT_OFF_MAX
param show MOT_SVO_*_REV
param show ARMING_SKIPCHK
rc 3 1000
rc 7 1500
rc 8 1000
mode GUIDED
arm throttle
rc 3 1500
takeoff 3
```

### 4. 分级俯仰测试 / Graduated pitch test

先在 3 m 稳定悬停，再依次测试；每阶段到位并保持后才发送下一条。

Establish stable hover at 3 m, then follow this sequence, settling and holding each stage before continuing.

```text
0° → +30° → 0° → −30° → 0° → +60° → 0° → −60° → 0° → +90° → 0° → −90° → 0°
```

MAVProxy 的默认 `cmdlong` 模块提供 `long`。例如 +30° / For example, request +30°:

```text
long 31010 1 30 5 0 0 0 0
```

`31010` 是该分支的 `MAV_CMD_USER_1`：param1=1，param2 为目标俯仰角（度），
param3=5°/s，其余为 0。把 `30` 换成当前阶段目标；回正如下。

`31010` is this branch's `MAV_CMD_USER_1`: param1=1, param2 is target pitch in
degrees, param3=5°/s, and the rest are zero. Replace `30` with the stage target; return upright with:

```text
long 31010 1 0 5 0 0 0 0
```

必须确认 ACK 接受，并检查真实姿态、高度与速度；**ACK、PitOff、虚拟 AHRS 或视觉倾转
不能单独作为 ±90° 验收**。每阶段需连续 10 秒墙钟及仿真时间满足：真实重力相对倾角
误差 ≤5°、高度 3±0.5 m、水平速度与垂向速度绝对值均 ≤0.5 m/s；每阶段上限 120 秒。
航向按用户要求只记录。异常停止后续阶段，尝试回正并 LAND；不关闭预检、不强制解锁或空中解除武装。

Confirm accepted ACKs and check true attitude, altitude and speed. **ACK, PitOff,
virtual AHRS or visual appearance alone cannot validate ±90°**. Each stage requires
10 continuous wall-clock and simulation seconds with true gravity-relative tilt
error ≤5°, altitude 3±0.5 m, horizontal and absolute vertical speed ≤0.5 m/s, within
120 seconds. Heading is recorded only, by user decision. On abnormal behavior,
stop advancing, attempt to return upright and LAND. Do not bypass checks, force-arm or force-disarm in flight.

<a id="shutdown"></a>
## 结束仿真 / Shutdown

1. MAVProxy 输入 `mode LAND`，观察落地并等待正常 `DISARMED`。
2. 停止 Gazebo 内置录像、ROS 录包，等待文件写完。
3. 退出本轮 `sim_vehicle.py` / MAVProxy，确认 SITL 子进程退出。
4. Gazebo launch 终端 Ctrl+C，确认 **GUI 与 server 均退出**。
5. 重启前检查残留，只清理明确属于本轮的进程；不要全局 `pkill`。

1. Enter `mode LAND`; observe touchdown and normal `DISARMED`.
2. Stop native Gazebo recording and ROS bags; allow files to flush.
3. Exit this session's `sim_vehicle.py` / MAVProxy and confirm SITL exits.
4. Press Ctrl+C in the launch terminal; confirm **both GUI and server exit**.
5. Before restarting, inspect leftovers and stop only owned processes; never use broad `pkill`.

只读检查示例，匹配项需确认归属 / Read-only checks; inspect ownership of each match:

```bash
pgrep -af 'gz sim|gz-sim|arducopter|sim_vehicle.py|mavproxy.py|ros2 launch'
ss -luntp | rg ':(9002|9003|5760|5762|5763)\b'
```

<a id="interfaces"></a>
## ROS 接口与记录 / ROS interfaces and recording

另开观察终端 / Open a monitoring terminal:

```bash
source ~/Projects/uavros2_ws/.tmp/install/colcon/setup.bash
ros2 topic list
ros2 topic info /prop_speed/motor_0 -v
ros2 topic echo /prop_speed/motor_0 --once
```

| 模型 / Vehicle | ROS 2 topic | 类型与含义 / Type and meaning |
| --- | --- | --- |
| `tsduav_quad` | `/prop_speed/motor_0` … `motor_3` | `std_msgs/msg/Float32`，原旋翼角速度反馈 / original rotor angular-velocity feedback |
| `tilt_quadcopter` | `/prop_speed/motor_0` … `motor_7` | `std_msgs/msg/Float32`，8 路电机反馈 / eight motor feedback channels |
| `tilt_quadcopter` | `/tilt_pos/servo_0` … `servo_3` | `std_msgs/msg/Float32`，**角速度 rad/s，不是角度** / **angular velocity in rad/s, not position** |
| `tilt_quadcopter` | `/tilt_quadcopter/imu` | `sensor_msgs/msg/Imu` |
| `tilt_quadcopter` | `/gazebo/command/prop_speed` | `mav_msgs/msg/Actuators`，8 路电机指令 / eight motor commands |
| `tilt_quadcopter` | `/gazebo/command/tilt1_pos` | `mav_msgs/msg/Actuators`，4 路倾转目标，沿用 `angular_velocities` 字段 / four tilt targets carried in `angular_velocities` |

仅 ROS 2 数字开头的反馈名进行了已批准的合法化映射，Gazebo 话题和通道顺序不变。
飞控接管时不要另发执行器指令，也不要重复启动已有世界桥。ROS 反馈不等于 MAVROS2 控制验收。

Only approved numeric-leading ROS feedback names are remapped; Gazebo topics and
channel order are unchanged. Do not inject actuator commands during ArduPilot
control or duplicate the world bridge. Feedback availability is not MAVROS2 flight-control validation.

### 日志、录包与截图 / Logs, bags and capture

ROS 日志默认 `~/.ros/log/`，Gazebo 使用自身默认位置，SITL 状态与 DataFlash 跟随
所选工作目录。录包需主动开启，例如普通四旋翼：

ROS logs default to `~/.ros/log/`; Gazebo uses its native locations. SITL state and
DataFlash follow its working directory. Recording is opt-in, for example for the quad:

```bash
ros2 bag record /prop_speed/motor_0 /prop_speed/motor_1 \
  /prop_speed/motor_2 /prop_speed/motor_3
```

录包在当前目录默认生成 `rosbag2_*`，Ctrl+C 正常结束。
不设置 `ROS_LOG_DIR`、`GZ_HOMEDIR`、`GZ_LOG_PATH` 集中收集。

Bags use the default `rosbag2_*` directory under the current directory; finish with
Ctrl+C. Do not centralize outputs through log/home environment overrides.

所有调试、文档和验收截图只用 Gazebo 内置 **Screenshot**，视觉录像只用 **VideoRecorder**；
从 GUI 插件菜单添加缺少的工具并使用原生保存位置。禁止桌面/窗口截图与外部录屏。
每个视觉证据目录必须有 `PROVENANCE.txt`，记录 world、实际 `GZ_PARTITION`、
内置工具/服务和输出路径。`gz sim --record` 是仿真状态记录，不是视觉录像。

All debugging, documentation and acceptance images must use Gazebo's built-in
**Screenshot**; visual videos must use **VideoRecorder**. Add missing tools from
the GUI plugin menu and use native save locations. Desktop/window capture and
external recorders are prohibited. Each evidence directory must include
`PROVENANCE.txt` identifying the world, actual `GZ_PARTITION`, native tool/service
and output path. `gz sim --record` records simulation state, not visual video.

<a id="development"></a>
## 架构与开发 / Architecture and development

仅展示关键路径，省略不代表删除。ROS 1 基线 `/home/llw/Projects/uavros_ws` 与迁移目录
`/home/llw/Projects/uavros2_ws` 同级，不嵌套工作区。

This abbreviated tree shows key paths, not the full inventory. The ROS 2 workspace
is a sibling of the ROS 1 baseline, not nested inside it.

```text
uavros2_ws/
├── README.md
└── src/
    ├── uav_simulator/
    │   ├── uav_control/
    │   ├── uav_gazebo/
    │   │   ├── launch/spawn.launch
    │   │   ├── models/
    │   │   │   ├── tsd_model/tsduav_quad/models/tsduav_quad/model.sdf
    │   │   │   └── tilt_quadcopter/models/tilt_quadcopter/model.sdf
    │   │   └── worlds/
    │   └── uav_gazebo_plugin/src/
    │       ├── ArduRotorNormPlugin.cc
    │       └── ArduRotorTiltQuadcopter.cc
    ├── rotors_simulator/
    │   ├── rotors_comm/
    │   ├── rotors_gazebo_plugins/src/gazebo_motor_model.cpp
    │   └── …
    ├── mav_comm/
    ├── glog_catkin/
    ├── catkin_simple/
    └── vrpn_mavros/
```

`uav_gazebo` 管模型、世界与启动，`uav_gazebo_plugin` 管原 ArduPilot 仿真接口，
`rotors_gazebo_plugins` 管原旋翼动力学、倾转关节控制与世界 ROS 桥。
普通四旋翼仍使用 4 个 `librotors_gazebo_motor_model.so` 和 `libArduRotorNormPlugin.so`，
没有新增 `TsdRotorAero` 替代实现。

`uav_gazebo` owns models, worlds and launch; `uav_gazebo_plugin` owns original
ArduPilot simulation interfaces; `rotors_gazebo_plugins` owns motor physics,
tilt-joint control and the world ROS bridge. The quad retains four original motor
plugin instances and `libArduRotorNormPlugin.so`, with no `TsdRotorAero` replacement.

开发先和 ROS 1 对应路径比较，再原位修改；一次性诊断脚本仅放 `.tmp/`。
C++ 修改需停止仿真后重新 colcon 构建，在新终端加载统一安装空间并复测。
不要新增启动包装器、改名包或搬动模型来绕过未移植功能。

Compare against matching ROS 1 paths before editing in place. Keep one-off
scripts in `.tmp/`. Stop simulation before rebuilding C++, source the unified
installation in a new terminal, and retest. Do not add startup wrappers, rename
packages or move models to hide incomplete ports.

<a id="validation"></a>
## 验证与限制 / Validation and limitations

2026-09-06 已有运行记录 / Recorded results on 2026-09-06:

| 验证 / Test | 结果 / Result |
| --- | --- |
| `tilt_quadcopter` | 连续两轮 13 阶段通过，包括 ±30/60/90°、每次回正与正常 LAND / Two complete 13-stage passes including ±30/60/90°, upright returns and normal LAND |
| Tilt hold windows | 最大重力相对倾角误差 / peak tilt error: 4.9743° / 4.9682°；高度 / altitude: 2.915–3.037 m；最大水平速度 / peak horizontal speed: 0.0421 m/s |
| `tsduav_quad` regression | 0.5 m 目标悬停 10 秒，高度 0.463–0.539 m，正常降落 / 10-second hover at 0.5 m target, altitude 0.463–0.539 m, normal landing |
| Build | 当前所需六包通过 / The required six-package set builds |

倾转记录：`tilt_standard_20260906_041319`、`tilt_standard_20260906_041934`；
四旋翼回归：`quad_standard_20260906_042606`。完整历史、ROS 1 对照和参数全文见
[模型说明](src/uav_simulator/uav_gazebo/models/tilt_quadcopter/readme.md)。
报告、录包、截图及临时自动验收工具保留在测试机器，**不随 Git 克隆提供**。

The run identifiers above refer to local test records. See the linked model notes
for history, ROS 1 comparisons and complete parameters. Reports, bags, screenshots
and temporary automated gates remain on the test machine, **not in a Git clone**.

必须保留的边界 / Important limits:

- **惯量 / Inertia：**前倾转臂 Izz 从 0.01 改为 0.06 kg·m²，保留质量 0.2 kg、
  Ixx=0.04、Iyy=0.10。用户批准的仿真近似，不是 CAD 测量；验收仅适用于该近似模型。
  Front tilt-arm Izz is approximated as 0.06 rather than 0.01 kg·m² to satisfy
  Harmonic's inertia constraint. Mass, Ixx and Iyy are retained. This is not CAD-derived.
- **航向 / Heading：**真值与估计仍有偏差，此轮只记录，不宣称修复。
  A heading estimation discrepancy remains; it was recorded, not fixed or certified.
- **传感器 / Sensors：**缺少 `front_rangefinder` 时只禁用测距发送，未新增传感器；
  GPU lidar 未验收。Missing rangefinder disables only range transmission; no sensor
  was added and GPU lidar remains unvalidated.
- **手动流程 / Manual workflow：**本文命令按官方工具与本地源码核对；已有飞行验收
  使用同一 SITL 二进制及临时 MAVLink 客户端，不是另行完成的 MAVProxy 手动或 MAVROS2
  飞行验收，也未做全新机器安装验收。Commands were checked against official tools and
  local source. Recorded flights used the same SITL binary with a temporary MAVLink
  gate, not separately validated manual MAVProxy/MAVROS2 flights or a clean-machine installation.
- **完整工作区 / Full workspace：**其它模型和全量包尚未整体验收。全量构建曾停在
  `rotors_hil_interface` 缺少 `mavconn/mavlink_dialect.hpp`；全量依赖检查还涉及
  `topic_tools`、`vrpn_mocap`、`mavros`、`mavros_extras`。原路径保留，不隐藏缺项。
  Other models and the full workspace remain unvalidated; the selected build does
  not resolve or hide the full-build and dependency gaps listed here.

<a id="troubleshooting"></a>
## 常见问题 / Troubleshooting

| 现象 / Symptom | 检查与处理 / Check and action |
| --- | --- |
| `Package 'uav_gazebo' not found` | 编译后在新终端 source `.tmp/install/colcon/setup.bash` / Build, then source the unified installation. |
| 网格缺失 / Missing meshes | `git lfs pull`、`git lfs fsck`；从原 launch 启动以设置资源路径 / Fetch LFS assets; use the original launch for resource paths. |
| 插件加载失败 / Plugin load failure | 确认 Jazzy/Harmonic、六包构建及无 Classic 库或旧 overlay 混入 / Check versions, build and stale overlays. |
| 默认启动错误世界 / Wrong default world | 显式选择 `world_name:=tsduav_quad` 或 `tilt_quadcopter`；原默认 `arduwoodpecker_demo` 未在此验收 / Select a validated world explicitly. |
| GUI 显示但不飞 / GUI visible, no flight | launch 不启动 SITL；确认模型就绪、仿真未暂停，再开终端 2 / Wait for Gazebo readiness, then start SITL separately. |
| SITL 无连接 / No SITL link | 使用 `--model Gazebo` 而非 JSON；检查 UDP 9002/9003 与残留进程 / Check protocol, ports and stale processes. |
| 参数不对或倾转反向 / Wrong parameters or tilt direction | 核对固件提交、参数全文、反向参数 0/1 语义与旧 `eeprom.bin` / Check revision, full profile, boolean reversals and saved state. |
| `PreArm` 拒绝 / Pre-arm rejection | 读取具体原因，等待 GPS/Home/EKF；不要关闭检查或强制解锁 / Resolve the reported cause, never bypass checks. |
| `long` 未识别 / Unknown `long` | 在 MAVProxy 输入 `module load cmdlong`，不是 Bash / Load `cmdlong` in MAVProxy. |
| 重复 GUI / Duplicate GUI | 确认旧 GUI/server/SITL 全部退出后再启动 / Verify complete cleanup before restarting. |

原参数名保留，但 `x/y/z`、`model` 等旧参数不构成已实现的动态 spawn 接口；
当前模型由 world SDF 加载。两架 ArduPilot 模型保持 `enable_uav_control:=false`，
不要额外启动 `tricopter_control_node`。

Legacy names remain, but `x/y/z` and `model` do not implement dynamic spawning
in this port; worlds load models through SDF. Leave `enable_uav_control:=false`
for these ArduPilot vehicles; do not additionally start `tricopter_control_node`.

<a id="references"></a>
## 官方文档与来源 / References and provenance

- [ROS 2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2: building with colcon](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Gazebo Harmonic: installation with ROS](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Official ros_gz_sim launch usage](https://docs.ros.org/en/jazzy/p/ros_gz_sim/)
- [ArduPilot Linux development environment](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) — 注意上述协议差异 / note the protocol distinction above.
- [Using ArduPilot SITL](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)

源码来源 / Source provenance:

| 来源 / Source | 参考提交 / Reference revision |
| --- | --- |
| ROS 1 `uavros_ws` | `ea17560f79c910623db00350d9b30e67784fc5e0` |
| `uav_simulator` | `4e433449d4b8b88ee104dbc2ff772fdf6685b645` |
| [RotorS fork](https://gitee.com/Luviewer/rotors_simulator) | `656e83a6154bf53e792b75df68c572c2e10a9ec0` |
| [TSD models](https://github.com/LuweiLiao/tsd_model) | `fa671a1cf23b21a8939ed52b607723d52fe62496` |

提交标识用于来源追溯，迁移比对仍以本地 ROS 1 实际文件为基线，包括既有未提交修改。
各组件保留原作者版权与许可证，按对应包查看（例如
[uav_simulator LICENSE](src/uav_simulator/LICENSE)），不另行声称整个聚合仓库统一许可。

Revisions record provenance; migration comparisons use actual local ROS 1 files,
including pre-existing changes. Original copyrights and licenses remain per
component; see each package, for example the linked `uav_simulator` license.
No replacement blanket license is claimed for this aggregate repository.
