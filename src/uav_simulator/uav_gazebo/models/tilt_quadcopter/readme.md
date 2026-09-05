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

## 验收目标（尚未完成）

ArduPilot：`staging/tritilt-fixed`，核对时提交
`99a9622610de489d65b441e2d9efe46f453bdb0d`。保持原飞控算法。

1. 默认 GUI 加载、12 路执行器与 IMU/ROS2 话题检查。
2. 协议、通道、倾转正负方向检查，再做起飞与水平稳定悬停。
3. 逐级 30°、60°、90°，正负方向分别验证，并在中间回到水平姿态。
4. ±90° 使用真实机体姿态（Gazebo/SITL 真值）验收，不能只看 PitOff、
   虚拟 AHRS 或指令 ACK。目标保持 10 秒、姿态误差 ≤5°、
   高度误差 ≤0.5 m、水平速度 ≤0.5 m/s；异常立即终止后续倾转并尝试回正落地。
5. LAND 后正常解除武装，完成日志、Gazebo 原生视觉证据与进程/端口清理。

## 实测与参数语义核对

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
也已通过，但最后回正未满足完整姿态门限，整轮验收仍未通过。

`tilt_standard_20260906_013110` 的 +90° 和 -90° 各自完成 10 s 保持，
但最后回正时完整姿态误差在 5° 门限附近缓慢收敛，未在原脚本的 70 s
等待上限内完成连续保持，因此整轮仍判失败；随后正常落地、非强制解除武装，
所有进程和端口已清理。后续测试明确将每阶段最大等待时间设为 120 s，
保持姿态 ≤5°、高度误差 ≤0.5 m、水平速度 ≤0.5 m/s、垂向速度 ≤0.5 m/s
及连续 10 s 的标准不变，并记录每阶段耗时。这一调整允许较慢收敛，
不能据此宣称快速倾转动态性能通过；70 s 条件下的失败记录保留。

### 最新完整复测（未通过，2026-09-06）

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
**整轮仍为失败**；不能只取两端角度结果忽略回正失败。等待进一步定位后再
决定修复范围，暂不将本轮作为完成版本推送 GitHub。

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
