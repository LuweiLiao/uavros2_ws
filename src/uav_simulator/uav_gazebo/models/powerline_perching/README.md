# 输电线停靠四旋翼：上凸喇叭槽方案

用户于 2026-09-17 确认 HTML 三维概念后新增的 ROS 2 机型，名称为
`powerline_perching`。这是授权的新结构设计，**不是 ROS 1 机型的迁移或替换**。
沿用 `uav_gazebo` 模型目录、世界目录、XML 启动方式和现有 RotorS 电机插件；
未修改已有机型的几何、质量、执行器或飞控参数。

## 带输电塔的新场景

`powerline_corridor` 复用项目已有输电塔 CAD 网格，增加 100 m 跨距的
弧垂导线、分色绝缘子、悬垂线夹、防振锤、基础及简化碰撞体。
塔高约 17.64 m，最低导线中心约 11.04 m，导线直径 24 mm。
保留原 CAD 的双横担四线布局，是仿真概念场景，不是标准双回三相线路设计。

```bash
ros2 launch uav_gazebo powerline_perching.launch world_name:=powerline_corridor
```

该入口用于场景查看，不会启动 SITL 或自动飞行。原低空试验场景继续保留。
带输电塔场景的完整飞行、灰色导线识别和标注录像已通过一次端到端验证：

```bash
/home/llw/venv-ardupilot/bin/python3 .tmp/powerline_perching/record_full_flight.py --scene corridor
```

目标是靠近起飞侧的上层导线，跨中高度 13.53674 m；LOITER 从地面升至
约 15 m，再横移、切换 GUIDED，通过 RGB-D 灰色细线检测下降。
增加了旋翼反光的自身几何排除及上下层导线的深度分离。
2026-09-17 成功运行最终正常解除武装，仍有导线接触，机体高度
13.148827 m，横向偏差约 1.4 mm。该误差是理想仿真的单次结果。
完整录像：`/home/llw/powerline_perching_pip_20260917_215220.mp4`。
飞行记录：`/home/llw/.ros/log/perching_flight_20260917_215220/result.json`。
模型来源、参数和碰撞简化见 `models/powerline_corridor/README.md`。
Gazebo 原生全景/近景录像：`/home/llw/powerline_corridor_20260917_212249.mp4`。
验证记录：`/home/llw/.ros/log/corridor_preview_20260917_212249/result.json`。

## 结构与坐标

单位为 m / kg / s。机体坐标：X 沿导线、Y 向左、Z 向上。

- 左右两块蓝色挡板沿 X 延伸，共同构成开口向下的喇叭槽。
- 槽顶向上凸起，橙色承重横梁是中央通道上方唯一跨接件。
- 左右机身、电池、支撑脚和机臂都布置在导线通道两侧。
- 四个旋翼布置于低位，导线从下方进入时不穿过旋翼扫掠圆。
- 两块低位电池用于降低重心。槽顶承担竖向载荷，挡板引导和限制横向偏移。
- 目前没有防脱锁扣或主动夹紧件。

| 项目 | 第一版数值 |
| --- | --- |
| 总质量（含下视相机及支架，假定质量） | 2.963 kg |
| 总重心相对 base_link | (0.006392, 0.000693, 0.032977) m |
| 槽顶下表面高度 | 0.400 m |
| 导线直径 | 24 mm |
| 槽喉净宽 / 喇叭口净宽 | 32 mm / 约 333 mm |
| 承重横梁长度 | 420 mm |
| 停靠时导线中心高于重心 | 355.0 mm |
| 旋翼半径 | 145 mm |
| 旋翼平面高度 | 79 mm |
| 居中导线表面至旋翼扫掠圆横向间隙 | 213 mm |
| 包含旋翼及相机的约略长 × 宽 × 高 | 0.81 × 1.03 × 0.792 m |

间隙数值只描述居中且导线平行 X 的几何条件，不是自动降落的允许误差范围。
斜入槽、横向偏差、俯仰和导线摆动都需要单独验证。

## 文件

- `generate_model.py`：参数化几何与质量来源，运行后生成以下文件。
- `models/powerline_perching/model.sdf`：Gazebo Harmonic 动态模型，具有真实碰撞体。
- `models/powerline_perching/model.urdf`：相同结构与惯量的 ROS 2 / RViz 描述。
- `models/powerline_perching/model.config`：Gazebo 模型元数据。
- `design_parameters.json`：质量、重心、基础惯量及几何间隙。
- `../../worlds/powerline_perching.world`：刚性导线接触试验场景。
- `../../launch/powerline_perching.launch`：仿真与 ROS 2 桥接入口。

重新生成模型：

```bash
cd /home/llw/Projects/uavros2_ws
python3 src/uav_simulator/uav_gazebo/models/powerline_perching/generate_model.py
```

所有实体由尺寸明确的基本几何体组成，不依赖外部网格下载。
主体惯量根据每块部件的假定质量、局部旋转和平行轴定理合成，
四个旋翼分别具有独立惯量与旋转关节。URDF 用于描述；Gazebo 请加载 SDF，
直接把 URDF 转换为 SDF 不会自动带上本模型的电机和传感器插件。

## 启动

```bash
cd /home/llw/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uav_gazebo --symlink-install
source install/setup.bash
ros2 launch uav_gazebo powerline_perching.launch
```

需要工作区已有的 `rotors_gazebo_plugins` 动态库；本机已具备。
可加 `paused:=true` 检查结构，或 `gui:=false` 只运行服务器。
结束时 Ctrl+C，并检查本次服务器、GUI 和桥接均已退出。

默认场景把导线中心设在 1.6 m，高度 1.242 m 的机体已经对准导线，
槽顶距接触位置约 30 mm。零电机输入下，模型下降约 30 mm 后由导线承重，
机体最终高度约 1.212 m。**这是接触验证场景，不是自动飞行降落演示。**

## ROS 2 接口

| 话题 | 类型 | 方向 / 意义 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间 |
| `/powerline_perching/imu` | `sensor_msgs/msg/Imu` | 姿态传感器数据 |
| `/powerline_perching/odometry` | `nav_msgs/msg/Odometry` | 仿真真值，不能代替实机定位 |
| `/powerline_perching/contact` | `ros_gz_interfaces/msg/Contacts` | 导向板、槽喉和槽顶的碰撞接触 |
| `/powerline_perching/command/motor_speed` | `actuator_msgs/msg/Actuators` | 四路速度指令，`velocity` 含四项，单位 rad/s |
| `/powerline_perching/motor_speed/motor_0` … `motor_3` | `std_msgs/msg/Float32` | 带方向的仿真关节速度 rad/s |

电机使用现有 `librotors_gazebo_motor_model.so` 的 `prop_0_plugin` …
`prop_3_plugin` 注册别名，没有新增动力学插件。桥接为官方 `ros_gz_bridge`。

| 索引 | X | Y | 从 +Z 看旋转方向 |
| --- | --- | --- | --- |
| 0 | +0.26 | −0.37 | CCW |
| 1 | −0.26 | −0.37 | CW |
| 2 | −0.26 | +0.37 | CCW |
| 3 | +0.26 | +0.37 | CW |

指令速度非负；旋转方向由插件配置。保留 RotorS 的 10 倍可视化减速，
因此 200 rad/s 指令对应关节反馈约 ±20 rad/s，推力按实际转速计算。
假定推力系数为 `1.8e-5 N/(rad/s)^2`，最大速度 900 rad/s，
理论悬停速度约 627.7 rad/s，理论最大推重比约 2.06；这些数值尚未由实物标定。
接触测试入口不会连接 SITL。下面的完整飞行入口使用 ArduPilot、视觉状态机、
GUIDED 指令超时制动和失去目标后的退出流程。

## 验证边界

- 静态文件检查：模型与世界 SDFormat 校验、URDF 解析、包构建。
- 接触运行检查：电机关闭时的槽顶承重、ROS 2 接触/IMU/里程计和电机反馈。
- 扰动检查：绕导线释放约 6.9° 横滚，检查是否仍被支撑且摆幅有界。
  模型存在回正运动，但无专门阻尼，不能据此宣称快速收敛或稳定静止。
- 电机接口检查：支撑状态下四路 200 rad/s 低推力输入及归零。
- 原生 Gazebo 截图记录在 Gazebo 默认图片目录，相邻 `PROVENANCE.txt` 记录来源。

质量、惯量和推力参数均为第一版设计假设。导线为固定刚性圆柱，
尚未包含柔性、弧垂、风载、绝缘或电气效应。视觉落线仅针对下述理想场景，
不代表阵风防脱、真实灰色导线识别或实际线路作业已经验证。

## 初版无相机模型的接触验证（2026-09-17）

包构建、SDF 校验与 URDF 解析通过。Gazebo GUI 实际运行中，机体静止高度为
1.211999 m，接触消息非空，IMU、里程计和四路电机反馈均收到。
6.9° 横滚释放后仍保持导线支撑，末段一秒摆幅仍约
6.45°，因此只确认被动回正和保持支撑，
**未通过“快速衰减至静止”的期望**。没有为了通过测试而额外添加虚构阻尼。
四路 200 rad/s 输入得到约 ±20 rad/s 的减速关节反馈，归零后全部停转。
本次 Gazebo、GUI 和桥接均已关闭，未遗留所属进程。

本地详细结果：`/home/llw/Projects/uavros2_ws/.tmp/powerline_perching/result.json`。
Gazebo 原生截图：`/home/llw/.gz/gui/pictures/2026-09-17T17:57:47.224108891.png`。

## 下视视觉与 ArduPilot 完整流程

新增 `flight.sdf` 与 `powerline_perching_flight.world`，由同一个生成器生成。
下视 RGB-D 相机安装于槽前高位支架，光心相对机体为 `(0.32, 0, 0.568)` m，
分辨率 640×480、水平视场 1.3 rad、20 Hz，深度范围 0.04–12 m。
它能在导线高于低位旋翼时继续观察槽前的导线，避免机腹相机末段失去视野。

闭环流程为：定位与相机就绪 → LOITER 解锁起飞 → LOITER 横移到导线上方 →
GUIDED 视觉对准与下降 → 槽体接触确认 → LAND 降低推力 → 飞控正常解除武装 →
验证导线仍承重。无人机从距导线横向约 1 m 的地面起飞，没有瞬移、强制解锁、
关闭碰撞或直接指定模型轨迹。

图像识别使用橙色试验导线的像素分割、深度反投影及直线拟合，拒绝无效深度、
过期图像和时间不同步的图像。通过相机安装参数及飞控姿态转换到 NED，
经 `SET_POSITION_TARGET_LOCAL_NED` 发送横向、竖向速度和偏航角速度。
该消息的目标量为速度，线位置估计由伴随计算机转成闭环速度；不是修改飞控
使其直接解析图像。沿一根均匀直线的纵向位置无法仅由图像确定，纵向使用
飞控速度保持，不能据此宣称能停在导线上任意指定的纵向点。

现有 `ArduRotorNormPlugin` 转发 FRD 原始 IMU，因此仅 `flight.sdf` 的 IMU
绕 X 旋转 π，并使用 1000 Hz。ROS 接触模型维持原有 FLU IMU。
飞控 X 架构与插件 `[0,3,1,2]` 通道映射匹配。未修改插件或飞控控制律。

ArduPilot 位于 `/home/llw/Projects/ardupilot`，已创建分支
`codex/powerline-vision-perching`，新增 `powerline_perching/mav.parm`。
原有 UARTDriver.cpp 与 GCS_Common.cpp 未提交修改保留。未提交或推送本次改动。

### 一键运行全流程

本机 `/home/llw/venv-ardupilot` 已包含 pymavlink、OpenCV 和 NumPy。
全流程运行器仅使用本机 SITL TCP/UDP 端口，不接受实机连接地址。

```bash
cd /home/llw/Projects/uavros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select uav_control uav_gazebo --symlink-install
source install/setup.bash
source /home/llw/venv-ardupilot/bin/activate
ros2 run uav_control powerline_perching_sim.py
```

运行前确认 9002/9003 UDP 与 5760 TCP 空闲。程序会检查端口，使用独立的
Gazebo partition 和 ROS domain 73，并在结束时关闭自身启动的进程。
相机渲染较慢时，完整流程通常需要数分钟。

原生 Gazebo 截图位于 `~/.gz/gui/pictures`，相邻 `PROVENANCE.txt` 记录来源。
每次控制记录、MAVLink tlog、结果与独立 SITL 工作目录位于程序输出的
`~/.ros/log/perching_flight_时间/`，DataFlash 位于其 `sitl/logs`。
ROS bag 使用默认命名，保存在启动目录的工作区中。

视觉故障试验：

```bash
ros2 run uav_control powerline_perching_sim.py --inject-vision-loss
```

此试验故意在 GUIDED 中停止视觉估计，预期停止下降指令并退出为 LAND，
因此程序返回非零且结果 `passed=false`，不得作为成功落线。完整仿真仍需
检查正常解除武装与清理结果。它不代表所有故障或真实环境的安全保证。

### 附加话题与检查

| 话题 | 内容 |
| --- | --- |
| `/powerline_perching/down_camera/image` | 下视 RGB 图像 |
| `/powerline_perching/down_camera/depth_image` | 深度图 |
| `/powerline_perching/down_camera/camera_info` | 内参 |
| `/powerline_perching/vision/wire_offset_ned` | 从机体到导线的 NED 相对向量，Vector3Stamped |
| `/powerline_perching/phase` | 飞行阶段 |

控制脚本位于 `uav_control/scripts/powerline_perching_sim.py`。
Gazebo 真值只用于记录和最终验收，不参与视觉制导。
六项单元检查覆盖投影坐标正负号、姿态旋转、过期图像、不同步深度、
无目标与无效深度。仍需后续扩展真实灰色导线、复杂背景和光照变化的检测。

正式复跑结果位于 `/home/llw/.ros/log/perching_flight_20260917_182724/result.json`：
正常解除武装后，独立仿真测量的横向偏差约 1.6 mm、机体高度 1.211983 m，
接触保持且速度约 0.000785 m/s。该数值仅代表本次理想场景，不是精度保证。
视觉丢失退出试验位于 `/home/llw/.ros/log/perching_flight_20260917_183024/result.json`，
已观察到零速度指令、LAND 与正常解除武装。两次运行均完成所属进程清理。
详见工作区 `docs/concepts/powerline-flight-validation.md`。
