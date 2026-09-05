# nonplant_hex

非共面（fully-actuated）六旋翼无人机的 Gazebo 模型。

- 6 条机臂以 60 度间隔均匀分布。
- 每个旋翼以固定倾角 `±α`（沿对应机臂轴向交替）安装，从而获得 6 自由度全驱动控制能力（Voliro 风格）。
- 通过 `libArduRotorNormPlugin.so` 与 ArduPilot SITL 通信，6 通道 motor，0 通道 servo。

## 子模型

| 子目录 | 作用 | mesh |
| --- | --- | --- |
| `models/nonplant_hex_base/` | 机体（含 IMU） | `meshes/body.STL` |
| `models/nonplant_hex_arm/`  | 单臂            | `meshes/arm.STL`  |
| `models/nonplant_hex_prop/` | 单旋翼          | `meshes/prop.STL` |
| `models/nonplant_hex/`      | 顶层装配（1 base + 6 arm + 6 prop + 插件） | - |

## 构建

```bash
task erb-nonplant_hex:nonplant_hex
```

## 启动

```bash
roslaunch uav_gazebo spawn.launch world_name:=nonplant_hex
```
