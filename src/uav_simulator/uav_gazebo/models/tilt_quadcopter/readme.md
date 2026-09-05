# tilt_quadcopter：ROS2 迁移与验证状态

基线：`/home/llw/Projects/uavros_ws` 中同名目录。此模型为四臂共轴八桨、
四个倾转舵机，不是 `tilt_quadcopter_fix` 或 `tsduav_quad`。

## 保持不变

- 原目录、模型嵌套、文件名、关节名、插件实例名及共享库名。
- 电机依次对应 SITL 通道 1–8；倾转舵机依次对应通道 9–12。
- `ArduRotorTiltQuadcopter` 使用原 Gazebo 二进制 UDP 协议，不使用 JSON 替代插件。
- RotorS `position` 模式负责倾转关节 PID；`velocity` 模式负责桨叶动力学。
- 默认 Harmonic GUI；原 `spawn.launch` 只启动 Gazebo，不隐式启动 SITL。

## 必要适配与用户批准的差异（2026-09-06）

- 补齐旧 XML 未加引号的属性；将 Classic Grey/Blue 材质的颜色值写入 SDF，
  不依赖 OGRE 材质脚本。SDF/RSDF 同步维护。
- 沿用原前倾转臂质量 0.2 kg、Ixx=0.04、Iyy=0.10 kg·m²，
  将 Izz=0.01 改为 0.06，使其满足惯量三角不等式。此项是用户批准的
  **仿真近似，不是 CAD 实测惯量**；测试结果仅适用于该近似模型。
- 当前模型未配置 `front_rangefinder`，允许只禁用测距发送，不新增传感器。
  原测距参数和 MAVLink UDP 发送职责保留，GPU lidar 路径尚未验收。
- 仅 ROS2 Float32 输出名改为 `/prop_speed/motor_0` 至 `motor_7`、
  `/tilt_pos/servo_0` 至 `servo_3`。Gazebo 话题、模型参数和通道顺序不变。
  注意：原 `motorSpeedPubTopic=tilt_pos/N` 的载荷仍是关节角速度，
  名称含 pos 不代表位置；不擅自改变载荷含义。
- 未被当前模型引用的 `tilt_quadcopter_tilt_back` 仍有不合法的原惯量，
  不因当前飞行切片迁移而擅自修正或宣称其已验收。

## 验收目标（用户修订范围内已完成）

ArduPilot：`staging/tritilt-fixed`，核对时提交
`99a9622610de489d65b441e2d9efe46f453bdb0d`。保持原飞控算法。

1. 默认 GUI 加载、12 路执行器与 IMU/ROS2 话题检查。
2. 协议、通道、倾转正负方向检查，再做起飞与水平稳定悬停。
3. 逐级 30°、60°、90°，正负方向分别验证，并在中间回到水平姿态。
4. ±90° 使用真实机体姿态（Gazebo/SITL 真值）验收，不能只看 PitOff、
   虚拟 AHRS 或指令 ACK。目标保持 10 秒、重力相对倾角误差 ≤5°（航向只记录）、
   高度误差 ≤0.5 m、水平速度 ≤0.5 m/s；异常立即终止后续倾转并尝试回正落地。
5. LAND 后正常解除武装，完成日志、Gazebo 原生视觉证据与进程/端口清理。

## 早期实测与参数语义核对（历史记录）

所需 6 个包编译通过；默认 GUI、12 路执行器与 ROS2 IMU 可用。
`tilt_standard_20260906_010851` 完成 3 m 水平悬停，连续 10 s 仿真时间内
高度 2.934–3.039 m、四元数姿态误差最大 3.912°、水平速度最大 0.014 m/s；
正常 LAND/解除武装。13 路 ROS2 录包均可解码，各有至少 85,698 条数据，
未检出非有限值。数值记录在 ROS 默认目录
`~/.ros/log/tilt_standard_20260906_010851/`，截图来自 Gazebo 内置 Screenshot。

随后 `tilt_standard_20260906_011736` 在 +30° 过渡期间水平速度超过 3 m/s，
中止后续倾转；回正与 LAND 获 ACK，但限时内未确认正常解除武装，因此该轮失败。
已关闭本轮全部仿真进程并验证端口释放。不得将这一结果视为 ±90° 飞行通过。

分支提交 `a9494701a727b35f681e518e0b3818b3df96aaa0` 将 `MOT_SVO_*_REV`
从旧版直接相乘的 ±1 改为布尔值（0 正向，非零反向）。原参数文件仍使用
RF/RR/LR=-1、LF=+1，直接加载会使 LF 的物理方向与旧版相反。
为保持原方向，临时 SITL 参数转换为 RF/RR/LR=1、LF=0；只转换数值语义，
保留参数名、关节轴、SITL 9–12 通道和飞控算法。ROS1 基线及 ArduPilot
原参数文件未修改。转换后 ±30°、±60° 分级飞行通过；±90° 单阶段保持
也已通过，但当时最后回正未满足完整姿态门限，旧口径整轮验收未通过。

`tilt_standard_20260906_013110` 的 +90° 和 -90° 各自完成 10 s 保持，
但最后回正时完整姿态误差在 5° 门限附近缓慢收敛，未在原脚本的 70 s
等待上限内完成连续保持，因此整轮仍判失败；随后正常落地、非强制解除武装，
所有进程和端口已清理。后续测试明确将每阶段最大等待时间设为 120 s，
保持姿态 ≤5°、高度误差 ≤0.5 m、水平速度 ≤0.5 m/s、垂向速度 ≤0.5 m/s
及连续 10 s 的标准不变，并记录每阶段耗时。这一调整允许较慢收敛，
不能据此宣称快速倾转动态性能通过；70 s 条件下的失败记录保留。

### 当前验收口径（用户修订，2026-09-06）

用户明确要求“航向误差当作合理范围”。因此本次正式复测不再以航向
偏差判失败，也不修改飞控/EKF/传感器参数去消除航向偏差。
角度要求为真实机体相对重力的倾角误差≤5°：使用归一化四元数将重力转入
机体系，与目标pitch对应的机体系重力向量比较。它对世界航向旋转不变，
不使用±90°处奇异的欧拉yaw/roll作判据；完整姿态误差和航向仍保留记录。
高度3±0.5m、水平/垂向速度≤0.5m/s、连续10秒墙钟及仿真时间、每阶段
120秒上限、全部13阶段、正常LAND和非强制解除武装保持不变。
新口径下的正常启动双次复测及综合审计均已通过；下文旧口径失败保留原判定。

| 正式复测（均13阶段通过） | 保持最大倾角误差 | 保持高度范围 | 保持最大水平速度 | 最后回正总耗时 |
| --- | ---: | ---: | ---: | ---: |
| `tilt_standard_20260906_041319` | 4.9743° | 2.915–3.037m | 0.04153m/s | 27.95s |
| `tilt_standard_20260906_041934` | 4.9682° | 2.932–3.036m | 0.04210m/s | 28.19s |

两轮均为相同库、SITL二进制、有效控制参数和门限脚本；没有诊断启动延时。
每段连续墙钟及仿真时间均≥10秒，保持垂向速度最大分别0.49183/0.45427m/s，
均≤0.5m/s。±90°与最后回正全部完成后正常LAND、非强制解除武装。
原ROS发布者→原世界桥→RotorS归属、13反馈+2指令MCAP读回、8/4路载荷、
独立13阶段重放、DataFlash角度/高度保持及解除武装均通过。
独立重放拒绝超过0.25秒的遥测间断；7组纯几何测试覆盖航向不变性和错误姿态拒绝。
每轮的 `FINAL_AUDIT.json` 所有检查通过，第二轮目录的 `GOAL_ACCEPTANCE.json`
另核验双轮库/参数/脚本一致及共用旋翼回归。报告根目录为 `/home/llw/.ros/log/`。

原生截图示例：`/home/llw/.gz/gui/pictures/2026-09-06T04:23:53.86053461.png`
（+90°）与 `2026-09-06T04:24:48.431063095.png`（-90°）；同目录有PROVENANCE。
全部自建Gazebo GUI/server、SITL、桥接、录包退出，端口已释放。
`quad_standard_20260906_042606` 对同一共用RotorS库的tsduav_quad回归通过：
0.5m目标悬停10.000s仿真/10.013s墙钟，高度0.463–0.539m，水平速度峰值
0.02828m/s，正常降落/非强制解除武装；原tsduav_quad模型字节未变。

本验收没有修复或认证航向精度：末10秒真航向均值约-5.98°/-5.91°，
估计航向约-0.24°；因不再等待航向收敛，不能与旧轮末端时刻直接等同。
沿用已批准的前倾转臂Izz=0.06近似，不宣称CAD惯量正确。
这只完成倾转仿真专项，不代表全部ROS1包、其它机型、MAVROS2或实机验收。

### 初期完整复测（未通过，2026-09-06；原记录保留）

记录：`~/.ros/log/tilt_standard_20260906_014008/`。
`FINAL_AUDIT.json` 汇总验收；`offline_analysis.json` 为 ROS2 录包读回，
`dataflash_analysis.json` 为原生飞控日志核验，`heading_analysis.json`
为最后回正阶段的估计/真值对照，`source_audit.json` 为原路径/插件契约审计。

| 阶段 | 连续保持（仿真 s） | 保持高度（m） | 最大水平速度（m/s） | 阶段总耗时（墙钟 s） |
| --- | ---: | ---: | ---: | ---: |
| +90° | 10.048 | 3.006–3.007 | 0.0313 | 27.44 |
| -90° | 10.000 | 3.001–3.002 | 0.00885 | 71.80 |
| 最后回正 | 未满足完整姿态门限 | 约 3.00 | 约 0.01 | 120 s 后超时 |

两端保持期间完整四元数误差分别不超过 4.905°、5.000°（未取整原值均 ≤5°）。
DataFlash 的 SIM 真值四元数及真实高度另行确认 +90°、-90° 分别存在
10.90 s、10.30 s 的连续合格区间；该独立检查不代替在线速度检查。
最后回正阶段末 10 s 的 195 对同步样本中：真实俯仰均值 0.020°、
真实横滚 -0.0005°，但真实航向 -5.019°、飞控估计航向 -0.219°。
这是约 4.8° 的估计/真值差，不是模型还停在 -90°。磁力计模拟偏置和
校正偏置已从 DataFlash 核对为一致，不能简单归因于漏载磁力计偏置；
具体根因未确定。尚未改动 ArduPilot 算法，也未用真值反馈去人为补偿指令。

尽管该轮最后正常 LAND、非强制解除武装，且所有自建进程/端口已清理，
**该历史轮按当时标准仍为失败**；原报告不追溯改判，也未作为完成版本推送。
用户后来修订航向门控后的双次正常流程验收见前面的“当前验收口径”。

本轮原生 Screenshot 图片：

- +90° 保持完成：`~/.gz/gui/pictures/2026-09-06T01:44:19.41256284.png`
- -90° 保持完成：`~/.gz/gui/pictures/2026-09-06T01:45:57.987748644.png`
- 来源记录：同目录 `PROVENANCE.txt`。截图只显示 Gazebo 原生渲染视图，
  不使用电脑窗口截屏；未录制视频。

参数参考 SHA256：`fbd897f110aca49626299c0cc274ee96076fa5ed7523c5b0aab129e95f36b10e`。
GUI-only 早期记录的惯量元数据澄清另见
`~/.ros/log/tilt_gui_20260906_005407/EVIDENCE_NOTE.md`，不覆盖原记录。

### 离线诊断补充：磁场偏置估计（未改飞控代码）

对上述同一轮数据进一步核验，未重新启动仿真，也未调整参数：

- `fdm_gyro_consistency.json`：20 Hz 真值角速度梯形积分与真值四元数比较，
  采样时间无间断；最终回正结束时累计差约 0.485°，整段最大约 0.698°。
  该检查有离散积分误差，只能说明未见轴向/符号级别的大幅不一致，
  不能证明全部 IMU 时序适配正确，也不能替代飞行验收。
- `magnetic_heading_analysis.json`：只比较倾转偏置为零的水平窗口，
  不在 ±90° 欧拉角奇异点解释航向。初始悬停（仿真 60–70 s）与最后
  回正（448–458 s）各取 100 对记录，Gazebo 真值换算的世界磁场方位均值
  分别为 12.724°、12.779°，相差仅约 0.055°。
- 同期飞控期望航向保持 -0.221°，最终估计航向约 -0.217°，但真实航向
  约 -5.025°。XKF2 中机体系磁场偏置估计由 `[0,0,0]` 变为
  `[-9,21,-4] mG`。已核对 `EK3_PRIMARY=0` 和日志 PI=0，所分析的核心
  与主核心一致。支持优先排查三轴磁场融合中的偏置估计，尚未证明具体触发机制。

当前 `EK3_MAG_CAL=3`，与指定分支原参数一致。源码说明 `2` 会改用持续
航向融合、不估计磁场状态，但同时会降低 Copter 对罗盘标定/对准错误的
检测灵敏度。因此未将其悄悄改为 `2` 来让测试通过；如需 A/B 验证，应作为
另行确认的仿真诊断，而非直接替代原配置交付。飞控算法修改范围仍待用户确认。

### ROS1 仿真飞行基线与迁移接口对照（2026-09-06）

当前 ROS1 原插件在启动时因模型没有 `front_rangefinder` 而退出；原模型
未进入飞行。记录为 `~/.ros/log/tilt_ros1_probe_20260906_021849/probe.log`。
追溯原仓库历史 `39a0ccc706e53b9f5c1ac69667699928a7697fde`，该版同名
插件尚无这一必需测距限制，当前原模型目录与该版本没有差异。
只在目标 `.tmp` 编译历史插件，在隔离 Noetic/Classic GUI 容器中优先加载；
原 ROS1 源码、模型和 devel 库均未改，实际进程库映射已核验。

所有对照使用相同 SITL 二进制、控制参数及未改动的动作/门限脚本。
ROS1 使用原 Izz=0.01；ROS2 使用上述已批准的 Izz=0.06 近似，
因此不能声称两个模型物理参数完全相同。记录目录均为 `~/.ros/log/`。

| 记录 | 改动或基线 | 完整结果 |
| --- | --- | --- |
| `tilt_ros1_flight_20260906_022636` | 原模型、历史原插件 | 水平悬停通过，最大姿态误差 0.853° |
| `tilt_ros1_flight_20260906_022856` | 同一 ROS1 基线 | 13 阶段全部通过，最后回正耗时 68.74 s |
| `tilt_ros1_flight_20260906_033140` | 原 ROS1 基线第二次复测 | 13 阶段全部通过，最后回正耗时 67.74 s |
| `tilt_ros1_inertia_006_20260906_034229` | 临时 Classic 副本仅改 Izz=0.06 | 13 阶段通过，最后回正耗时 75.95 s；不是原始基线验收 |
| `tilt_standard_20260906_023838` | ROS2 FDM 从 PostUpdate 恢复到 PreUpdate | 13 阶段通过，最后回正最大误差 4.9998707° |
| `tilt_standard_20260906_024714` | 上述 ROS2 版本复测 | 最后回正 120 s 超时，整轮失败 |
| `tilt_standard_20260906_030732` | 再将旋翼 JointVelocityCmd 换为 ResetVelocity | 最后回正 120 s 超时，整轮失败 |
| `tilt_standard_20260906_035242` | 再恢复原 ROS 指令通路与发布顺序 | 12阶段通过，最后回正120 s超时，整轮失败 |

这些飞行均正常 LAND、非强制解除武装，自建进程和端口已清理。
ROS1 数值基线已验证 GUI 运行，但未创建视觉截图，不宣称视觉证据齐全。
ROS2 Screenshot 沿用 Gazebo 原生默认位置，并保留 `PROVENANCE.txt`。
ROS1 数据包留在已停止的专用容器默认目录，容器与数据均保留。

速度接口差异有官方 API 依据：Classic `SetVelocity` 直接设置状态，
Gazebo Sim `JointVelocityCmd` 则由引擎施加追踪力矩，`ResetVelocity`
才对应直接状态更新。但这项语义修正尚未消除此次残差，不能单独称为根因。
ROS1 与 ROS2 末段磁场读回均显示体轴偏置约 X=-9、Y=21 mG，
航向估计/真值差约 4.78°；主偏差不是 ROS2 独有。没有更改飞控算法、
EKF 配置、真实航向指令或放宽门限来制造通过结果。

上述旧口径阶段目标为未完成；后来用户明确排除航向门控，当前双次验收
结果见前面的“当前验收口径”。未把一次临界通过或诊断副本冒充正式修复。

后续逐包 IMU 测量 `tilt_imu_timing_20260906_032109` 只在 `.tmp` 编译
测量副本，未替换安装库。473003 个发送包中重复 IMU 132 次（0.0279%），
IMU 时间通常比包时间早 1 ms，最大 4 ms；完整速率积分的每秒采样差值
最大 0.003867°、最终 0.001392°。不能把这一小误差直接解释成约 5° 主残差，
也不能把低频离线积分的数值误差冒充真实传感器失配。测量轮仍在回正失败，
正常落地/解除武装和清理完成，不作为正式版本通过记录。

源码职责复核又发现旧移植绕过原 ROS 指令通路。现在同名倾转插件恢复
`mav_msgs/msg/Actuators` 的 `/gazebo/command/prop_speed` 和
`/gazebo/command/tilt1_pos`，由原世界桥转换给 RotorS；保留原队列深度10、
先电机后舵机、8/4路顺序、原float中间舍入与倾转角换算。
原 CMake/package.xml 的 mav_msgs 依赖恢复，不新增包、插件或正式文件。
6包构建通过，运行时已核验发布者为倾转插件、订阅者为原世界桥；
`tilt_standard_20260906_035242` 的12阶段通过，最后回正仍失败。
末10墙钟秒真航向均值 -5.0209°、估计航向 -0.2121°，真pitch 0.0186°、
真roll -0.0014°；正常LAND、非强制解除武装与全部进程/端口清理完成。
临时录包读回13反馈+2原指令通过，各指令471817条，8/4路载荷及有限值
检查无违规；DataFlash另行确认±90°保持与非强制解除武装。
飞行门限未改。恢复原职责是已验证的兼容修正，但尚非残余偏差根因修复。

## 开发启动与验收参数

首次构建使用工作区根 README 的统一 colcon 流程。每次启动前确认上一轮
Gazebo GUI/server 和 SITL 已退出，不同时启动第二份仿真。

终端一（已实测的原 launch 入口）：

```bash
source /home/llw/Projects/uavros2_ws/.tmp/install/colcon/setup.bash
ros2 launch uav_gazebo spawn.launch world_name:=tilt_quadcopter
```

终端二使用 ArduPilot 官方 `Tools/autotest/sim_vehicle.py`：
先使用下面的参考参数创建本地 `.tmp/tilt_quadcopter/tiltquad_sitl.parm`，
并选择没有旧 eeprom 的仿真工作目录；不要覆盖原有 SITL 状态。
本机验收准备的参数文件已在该位置。参数是从指定分支的
`tilt_quadcopter/tiltquad.parm` 筛选控制和执行器项，再转换上述反向语义；
不能把包含实机传感器标定的整份旧参数直接当作仿真默认值加载。

```bash
source /home/llw/venv-ardupilot/bin/activate
/home/llw/Projects/ardupilot/Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f gazebo-iris --model Gazebo \
  --add-param-file /home/llw/Projects/uavros2_ws/.tmp/tilt_quadcopter/tiltquad_sitl.parm
```

这里 `gazebo-iris` 选择官方 Gazebo 后端基础参数，叠加参数中的
`FRAME_CLASS=7 / FRAME_TYPE=23` 才是本模型。显式 `--model Gazebo`
对应原插件的二进制 UDP 协议，不能照搬新 JSON 插件的 `--model JSON`。
官方入口参数已按此分支源码核对；自动验收直接启动同一个
`build/sitl/bin/arducopter --model Gazebo --speedup 1`，依次加载
`copter.parm,gazebo-iris.parm,tiltquad_sitl.parm`，不用 MAVProxy 抢占测试连接。
**尚未独立验收 MAVProxy 交互操作流程**，不要与自动测试客户端同时接管。

飞行前等待 GPS/Home/EKF 就绪，检查 `ARMING_SKIPCHK=0`。
RC7 保持 1500、RC8 保持 1000；解锁前 RC3=1000，解锁后 RC3=1500。
GUIDED 起飞到 3 m 并稳定后，分级发送已有
`MAV_CMD_USER_1(31010)`：param1=1、param2=目标俯仰角度、
param3=5°/s，其余参数为 0。每次倾转后回到 0°，检查真实姿态、
高度与速度；异常停止后续阶段，回正后 LAND 并观察正常解除武装。
不要关闭预检或强制解锁/解除武装。测试完成后关闭所有本轮仿真进程。

此轮 ROS2 接收的是原世界桥的 13 路反馈；飞行指令直接通过 SITL MAVLink
进入飞控，**不是已经验收 MAVROS2 控制链路**。
视觉证据仅用 Gazebo 原生 Screenshot，位置 `~/.gz/gui/pictures/`，
同目录 `PROVENANCE.txt` 记录来源。日志与录包仍用各工具默认位置。

### 可重建的 SITL 参数参考

为不新增 ROS1 不存在的正式参数文件，此处保留验收输入全文；
需要复现时将代码块保存到上述本地临时参数文件。只用于该仿真模型，
不用于实机，也不改变 ROS1 或 ArduPilot 原参数文件。

```text
# Simulation test defaults: selected control / actuator parameters from
# ardupilot/tilt_quadcopter/tiltquad.parm, excluding hardware sensor calibration.
# Load after official copter.parm and gazebo-iris.parm in a fresh SITL state.
ATC_ACC_P_MAX 1100
ATC_ACC_R_MAX 1100
ATC_ACC_Y_MAX 270
ATC_ANG_LIM_TC 1
ATC_ANG_PIT_P 4.5
ATC_ANG_RLL_P 4.5
ATC_ANG_YAW_P 4.5
ATC_ANGLE_BOOST 1
ATC_ANGLE_MAX 30
ATC_INPUT_TC 0.15
ATC_LAND_P_MULT 1
ATC_LAND_R_MULT 1
ATC_LAND_Y_MULT 1
ATC_RAT_PIT_D 0.0036
ATC_RAT_PIT_D_FF 0
ATC_RAT_PIT_FF 0
ATC_RAT_PIT_FLTD 20
ATC_RAT_PIT_FLTE 0
ATC_RAT_PIT_FLTT 20
ATC_RAT_PIT_I 0.4
ATC_RAT_PIT_IMAX 0.5
ATC_RAT_PIT_NEF 0
ATC_RAT_PIT_NTF 0
ATC_RAT_PIT_P 0.5
ATC_RAT_PIT_PDMX 0
ATC_RAT_PIT_SMAX 0
ATC_RAT_RLL_D 0.0036
ATC_RAT_RLL_D_FF 0
ATC_RAT_RLL_FF 0
ATC_RAT_RLL_FLTD 20
ATC_RAT_RLL_FLTE 0
ATC_RAT_RLL_FLTT 20
ATC_RAT_RLL_I 0.4
ATC_RAT_RLL_IMAX 0.5
ATC_RAT_RLL_NEF 0
ATC_RAT_RLL_NTF 0
ATC_RAT_RLL_P 0.25
ATC_RAT_RLL_PDMX 0
ATC_RAT_RLL_SMAX 0
ATC_RAT_YAW_D 0
ATC_RAT_YAW_D_FF 0
ATC_RAT_YAW_FF 0
ATC_RAT_YAW_FLTD 100
ATC_RAT_YAW_FLTE 2.5
ATC_RAT_YAW_FLTT 20
ATC_RAT_YAW_I 0.02
ATC_RAT_YAW_IMAX 0.5
ATC_RAT_YAW_NEF 0
ATC_RAT_YAW_NTF 0
ATC_RAT_YAW_P 0.3
ATC_RAT_YAW_PDMX 0
ATC_RAT_YAW_SMAX 0
ATC_RATE_FF_ENAB 1
ATC_RATE_P_MAX 0
ATC_RATE_R_MAX 0
ATC_RATE_WPY_MAX 60
ATC_RATE_Y_MAX 0
ATC_THR_G_BOOST 0
ATC_THR_MIX_MAN 0.1
ATC_THR_MIX_MAX 0.5
ATC_THR_MIX_MIN 0.1
FRAME_CLASS 7
FRAME_TYPE 23
MOT_ANTI_YAW_FAC 1
MOT_BAT_CURR_MAX 0
MOT_BAT_CURR_TC 5
MOT_BAT_IDX 0
MOT_BAT_VOLT_MAX 12.8
MOT_BAT_VOLT_MIN 9.6
MOT_BI_PIT_FAC 0
MOT_BOOST_SCALE 0
MOT_FORW_FACT 2
MOT_HOVER_LEARN 2
MOT_IDLE_SEC 0
MOT_LAT_ENABLE 0
MOT_LAT_FACT 1
MOT_OPTIONS 0
MOT_PIT_OFF_MAX 90
MOT_PWM_MAX 2000
MOT_PWM_MIN 1000
MOT_PWM_TYPE 0
MOT_SAFE_DISARM 0
MOT_SAFE_TIME 1
MOT_SLEW_DN_TIME 0
MOT_SLEW_UP_TIME 0
MOT_SPIN_ARM 0.1
MOT_SPIN_MAX 0.95
MOT_SPIN_MIN 0.15
MOT_SPOOL_TIM_DN 0
MOT_SPOOL_TIME 0.5
MOT_SVO_LF_OFF 0
MOT_SVO_LF_REV 0
MOT_SVO_LR_OFF 0
MOT_SVO_LR_REV 1
MOT_SVO_RF_OFF 0
MOT_SVO_RF_REV 1
MOT_SVO_RR_OFF 0
MOT_SVO_RR_REV 1
MOT_THST_EXPO 0.65
MOT_THST_HOVER 0.383784
MOT_TILT_ANG_MAX 135
MOT_TILT_EN 1
MOT_TILT_YAW_FAC 1
MOT_YAW_HEADROOM 200
MOT_YAW_SV_ANGLE 30
PSC_ANGLE_MAX 0
PSC_D_ACC_D 0
PSC_D_ACC_D_FF 0
PSC_D_ACC_FF 0
PSC_D_ACC_FLTD 0
PSC_D_ACC_FLTE 20
PSC_D_ACC_FLTT 0
PSC_D_ACC_I 0.1
PSC_D_ACC_IMAX 0.8
PSC_D_ACC_NEF 0
PSC_D_ACC_NTF 0
PSC_D_ACC_P 0.05
PSC_D_ACC_PDMX 0
PSC_D_ACC_SMAX 0
PSC_D_POS_P 1
PSC_D_VEL_D 0
PSC_D_VEL_FF 0
PSC_D_VEL_FLTD 5
PSC_D_VEL_FLTE 5
PSC_D_VEL_I 0
PSC_D_VEL_IMAX 10
PSC_D_VEL_P 5
PSC_JERK_D 5
PSC_JERK_NE 5
PSC_NE_POS_P 1
PSC_NE_VEL_D 0.25
PSC_NE_VEL_FF 0
PSC_NE_VEL_FLTD 5
PSC_NE_VEL_FLTE 5
PSC_NE_VEL_I 1
PSC_NE_VEL_IMAX 10
PSC_NE_VEL_P 2
RC7_DZ 0
RC7_MAX 2000
RC7_MIN 1000
RC7_OPTION 221
RC7_REVERSED 0
RC7_TRIM 1500
RC8_DZ 0
RC8_MAX 2000
RC8_MIN 1000
RC8_OPTION 222
RC8_REVERSED 0
RC8_TRIM 1500
SERVO1_FUNCTION 33
SERVO1_MAX 1900
SERVO1_MIN 1100
SERVO1_REVERSED 0
SERVO1_TRIM 1500
SERVO10_FUNCTION 185
SERVO10_MAX 2500
SERVO10_MIN 500
SERVO10_REVERSED 0
SERVO10_TRIM 1500
SERVO11_FUNCTION 186
SERVO11_MAX 2500
SERVO11_MIN 500
SERVO11_REVERSED 0
SERVO11_TRIM 1500
SERVO12_FUNCTION 187
SERVO12_MAX 2500
SERVO12_MIN 500
SERVO12_REVERSED 0
SERVO12_TRIM 1500
SERVO13_FUNCTION 0
SERVO13_MAX 1900
SERVO13_MIN 1100
SERVO13_REVERSED 0
SERVO13_TRIM 1500
SERVO14_FUNCTION 0
SERVO14_MAX 1900
SERVO14_MIN 1100
SERVO14_REVERSED 0
SERVO14_TRIM 1500
SERVO15_FUNCTION 0
SERVO15_MAX 1900
SERVO15_MIN 1100
SERVO15_REVERSED 0
SERVO15_TRIM 1500
SERVO16_FUNCTION 0
SERVO16_MAX 1900
SERVO16_MIN 1100
SERVO16_REVERSED 0
SERVO16_TRIM 1500
SERVO2_FUNCTION 34
SERVO2_MAX 1900
SERVO2_MIN 1100
SERVO2_REVERSED 0
SERVO2_TRIM 1500
SERVO3_FUNCTION 35
SERVO3_MAX 1900
SERVO3_MIN 1100
SERVO3_REVERSED 0
SERVO3_TRIM 1500
SERVO4_FUNCTION 36
SERVO4_MAX 1900
SERVO4_MIN 1100
SERVO4_REVERSED 0
SERVO4_TRIM 1500
SERVO5_FUNCTION 37
SERVO5_MAX 1900
SERVO5_MIN 1100
SERVO5_REVERSED 0
SERVO5_TRIM 1500
SERVO6_FUNCTION 38
SERVO6_MAX 1900
SERVO6_MIN 1100
SERVO6_REVERSED 0
SERVO6_TRIM 1500
SERVO7_FUNCTION 39
SERVO7_MAX 1900
SERVO7_MIN 1100
SERVO7_REVERSED 0
SERVO7_TRIM 1500
SERVO8_FUNCTION 40
SERVO8_MAX 1900
SERVO8_MIN 1100
SERVO8_REVERSED 0
SERVO8_TRIM 1500
SERVO9_FUNCTION 184
SERVO9_MAX 2500
SERVO9_MIN 500
SERVO9_REVERSED 0
SERVO9_TRIM 1500
TTLT_RATE_MAX 30
TTLT_RTZ_RATE 15
```
