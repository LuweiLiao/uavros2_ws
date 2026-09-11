# tilt_quadcopter_fix 接触仿真交付记录

## 范围和版本

运行环境为 ROS 2 Jazzy / Gazebo Sim 8.11.0，历史渲染设备为 NVIDIA RTX 2060。
正式入口仍为 `uav_gazebo/launch/spawn.launch`、`apm.launch` 和原模型路径。
[工作空间 README](../README.md)给出三个终端的启动步骤；编译与运行统一使用
`.tmp/install/colcon`。本机此前还存在根目录 `build/`、`install/`，整理时在该已有
构建空间做了增量检查；不要把两个安装空间同时叠加或混用旧产物。

- MAVROS：上游 `3a070636ec8943d3fffe6b67bc903c144f12fced` 加随仓库交付的参数补丁。
- ArduPilot 历史基线：`99a9622610de489d65b441e2d9efe46f453bdb0d`。
- 接触参数本地提交：`641838a7741a4e75805da4937bbe6cfbf7e78c4c`，不包含后续控制器调参。
- 参数 SHA256：`5eb5a424e86f85363f2b873fe72552f43d52fe044967581973adea9db6e1cf46`。
- 历史固件二进制 SHA256：`7dd2635d8e361f9a1d19be0b5632aa2796253171191a0eb7eaeb44ad60d366ec`。

固定版本、参数补丁及 SITL 测距端口适配见[依赖说明](dependencies/README.md)。
整理开始时 ArduPilot 工作树还有后续 `mode_impedance.cpp` / `mode.h` 调参、UART 适配和
测距接收计数诊断；这些改动不属于本次参数提交，最新状态以独立仓库 `git status` 为准。
本轮检查的二进制已经不同于历史二进制，
不能直接把当前代码宣称为下面的历史验收配置。

## 已保留的功能修复

- world 显式加载 Physics、UserCommands、SceneBroadcaster、Imu、Sensors、ForceTorque。
- 只在本机嵌套模型内绑定 IMU / lidar；保留显式传感器话题并支持 GPU / 普通 lidar。
- FDM 使用实体世界位姿；ADM002 订阅 ForceTorque 实测话题并保留原 UDP 协议。
- MAVROS 插件读取完整参数文件，保持原距离传感器 ROS 话题。
- SERVO1–8 为 33…40，SERVO9–12 为 184…187；保留原 PWM 范围。
- 原模型中的 XML 引号、Harmonic lidar 类型和插件别名适配。

`tilt_quadcopter_fix` 本来就引用 `tilt_quadcopter` 的 base、prop、tilt_front。
整理确认新增副本仅修改模型名和网格 URI，网格与物理参数一致，故归档副本及别名软链接，
使用原有 `tilt_quadcopter/models` 搜索路径；没有新增模型命名空间或扁平化模型。

## 2026-09-10 历史接触验收

记录为 Run 38，结果来自原 `analysis/acceptance.json`，不是整理后重飞。

| 指标 | 历史结果 |
|---|---:|
| 连续 FORCE_HOLD | 45.057 s |
| 目标力 | 1 N |
| 末 10 s 平均力 | 0.92084 N |
| 末 10 s RMS 力误差 | 0.07969 N |
| 保持高度范围 | 1.97722–1.98107 m |
| 全程原始力峰值 | 1.628 N |
| ±0.1 N 误差带收敛时间 | 34.36 s |
| LAND / disarm | 通过 |

原始证据按原位置保留，不随 Git 分发：

- ROS2 bag：工作空间根目录 `rosbag2_2026_09_10-00_18_14/`。
- 量化与曲线：上述目录 `analysis/acceptance.json`、`decision.md`、`timeseries.png`、`contact_zoom.png`。
- DataFlash：ArduPilot `tilt_quadcopter/logs/00000038.BIN`。
- 当时源码与二进制哈希：`.tmp/tilt_quadcopter_fix/tilt-takeover-885715-provenance.json`。
- 一次性诊断脚本与过程记录：`.tmp/tilt_quadcopter_fix/`。后续脚本已变化，需结合版本记录使用。

起飞前保留解锁检查 `ARMING_SKIPCHK=0`，核对 EEPROM 中的实际执行器映射；
模式 29 在该版 MAVROS 显示为 `CMODE(29)`。原始峰值中止限值为 8 N。
历史 MAVROS 测距约 4 Hz，传感器输入 50 Hz；不代表高频 MAVROS 链路已验收。
先检查姿态、定位、力流、测距与参数，再执行 GUIDED 起飞和模式切换。
结束时 LAND，确认 disarm，再停止录包和本轮进程。

## 2026-09-11 整理检查

接触核心 C++ 源码和参数在整理前与 Run 38 哈希匹配。整理仅补全两处纯色材质、
移除重复资源路径及修正文档；未调整控制器增益或接触几何，未重新执行飞行。
不再宣称当前正式源码提供自定义 MarkerManager 青色测距线。

这仅是该机型的仿真记录，不表示全工作空间迁移、其他工况或真机验收完成。
