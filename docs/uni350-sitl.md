# uni350 / ArduPilot SITL

ROS 2 Jazzy、Gazebo Harmonic 8.11.0。本页仅验证 uni350，不代表其他模型均已迁移完成。

## 本次适配

保留 ROS 1 原始 uni350/base/prop 文件位置、网格、质量惯量、关节、四个电机编号、旋转方向、动力系数和命令话题。
`librotors_gazebo_motor_model.so` 增加原实例名的 Harmonic 加载别名，仍运行同一个 RotorS 电机实现。
`libArduRotorNormPlugin.so` 保持原四电机映射 `[0, 3, 1, 2]`，配合 ArduPilot Quad X。

`uni350.world` 增加 Harmonic Physics、Imu、SceneBroadcaster、UserCommands；太阳和地面改为内联资源，避免依赖 Classic 内置资源。材质改为直接颜色，IMU 使用显式 `/uni350/imu` 话题。起始机体中心高度从 2 m 改为 0.12 m，让飞机在地面完成检查后起飞。
`model.rsdf` 原模板未修改，运行读取 `model.sdf`；若以后重新生成模型，需要保留这些 Harmonic 适配。

## 启动

首次在工作空间构建：

```bash
cd /home/llw/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rotors_gazebo_plugins uav_gazebo --symlink-install
```

终端一，必须先启动 Gazebo：

```bash
cd /home/llw/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_PARTITION=uni350_manual
ros2 launch uav_gazebo spawn.launch world_name:=uni350 gui:=true
```

看到模型及 IMU、电机关节解析成功后，在终端二启动 SITL。每次使用新目录，避免旧参数影响：

```bash
uni350_run_dir=$(mktemp -d "$HOME/uni350-sitl-XXXXXX")
cd "$uni350_run_dir"
/home/llw/Projects/ardupilot/build/sitl/bin/arducopter \
  --model Gazebo --speedup 1 \
  --defaults /home/llw/Projects/ardupilot/Tools/autotest/default_params/copter.parm,/home/llw/Projects/ardupilot/uni350/mav.parm \
  --sim-address=127.0.0.1 --sim-port-in 9003 --sim-port-out 9002 -I0
```

MAVLink 地址 `tcp:127.0.0.1:5760`。等待 GPS/EKF 就绪，正常解锁，GUIDED 起飞到 3 m，再切 LOITER 悬停；结束时 LAND，确认自动上锁后关闭 SITL 和 Gazebo。
仿真参数保存在 ArduPilot 的 `uni350/mav.parm`；未关闭正常解锁检查，未修改飞控控制算法。

可选终端三，连接 ROS 2 IMU：

```bash
source /opt/ros/jazzy/setup.bash
source /home/llw/Projects/uavros2_ws/install/setup.bash
export GZ_PARTITION=uni350_manual
ros2 run ros_gz_bridge parameter_bridge '/uni350/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
```

不要同时运行另一个占用 9002/9003/5760 的 SITL。

## 本机自动验收

诊断脚本位于 `.tmp/uni350/run.py`（不安装，不随 Git 分发），先 Gazebo、后 SITL，记录 MAVLink、飞行遥测、原生截图，并在退出时回收所启动的进程组。

```bash
cd /home/llw/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/home/llw/venv-ardupilot/bin/python3 .tmp/uni350/run.py
```

2026-09-18 首次完整验收：GUIDED 解锁起飞 → 约 20 秒 LOITER → LAND → 自动上锁。
LOITER 高度 2.979–3.050 m，水平位置偏移最大 0.035 m，滚转/俯仰最大绝对值 0.269°（飞控估计值）。
日志：`/home/llw/.ros/log/uni350_20260918_221222/`，包含 `result.json`、`mavlink.tlog`、`telemetry.jsonl`、SITL DataFlash 日志和 Gazebo 日志。
此结果仅覆盖无风起降和定点悬停，原始动力学参数未做实机标定。

已验收 SITL 二进制 SHA256：`01c9a97f9d630b06587d4fc5caf5a5a75294a52d0f52980b1dbbe717d50e55a9`。
uni350 参数 SHA256：`a57ae5c38b11e3e496bb6b9798e61f77d81c320fc02507189f6a564a2293650f`。

第二次复测（使用正式 ArduPilot `uni350/mav.parm`，加入 ROS 2 IMU 桥接）同样通过：

- 日志：`/home/llw/.ros/log/uni350_20260918_221441/`。
- 高度范围 2.981–3.064 m，最大水平偏移 0.034 m，最大滚转/俯仰绝对值 0.317°。
- `/uni350/imu` 已由 `ros2 topic echo --once` 验证，消息存于 `ros2_imu.txt`。
- 截图：`/home/llw/.gz/gui/pictures/2026-09-18T22:16:11.470772412.png`。
  由 Gazebo 内置 Screenshot 服务生成，同目录 `PROVENANCE.txt` 记录 world、partition、服务和路径。
- 两次成功运行均完成 LAND / 自动上锁，并确认所启动的进程组退出、9002/9003/5760 端口释放。
- 构建通过；`uav_gazebo` 仍有原有 CMake CMP0009 开发警告。

发布范围：ArduPilot master 仅加入 `uni350/mav.parm` 和说明。上述飞行验证使用
`codex/powerline-vision-perching` 工作区（HEAD `554891b013`）的现有二进制，
并非从 master 重新构建后的独立飞行验收；原有其他控制器改动未随本次参数发布合入。
