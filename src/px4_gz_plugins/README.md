# PX4 Gazebo Harmonic 插件

从 PX4-Autopilot `23bd42a5bff02ce0d358877f852e04db879d59ec` 的
`src/modules/simulation/gz_plugins` 和 `gz_msgs` 原位复制源码，提供独立
CMake/ament 构建，不需要固件构建系统。支持 Gazebo Sim 8 / Transport 13 /
Sensors 8 / Plugin 2；需要 Protobuf、OpenCV，GStreamer 为可选视频推流依赖。

顶层 CMake 固定 Harmonic 依赖，启用 PIC，并安装插件到 `lib/px4_gz_plugins`。
ament 环境钩子将此目录加入 `GZ_SIM_SYSTEM_PLUGIN_PATH`。
OpticalFlow 原 CMake 中的 PX4 子模块拉取调用已移除，OpticalFlow 和 KLT 完整源码
随包提供。第三方目录放置 `COLCON_IGNORE`，避免被误识别成独立 ROS 包。
其余上游 C++ 算法源码未修改。来源提交见 `PROVENANCE.json`，许可证见 LICENSE 及依赖目录。

```bash
cmake -S src/px4_gz_plugins -B .tmp/px4-plugins-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/.tmp/px4-plugins-install"
cmake --build .tmp/px4-plugins-build -j4
cmake --install .tmp/px4-plugins-build
```

完整机型入口和验证范围见
[PX4 模型说明](../uav_simulator/uav_gazebo/models/px4_gz/README.md)。
