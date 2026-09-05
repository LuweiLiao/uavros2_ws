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

阶段状态：所需 6 个包编译通过；GUI/动力学联调中。不得视为 ±90° 飞行通过。
