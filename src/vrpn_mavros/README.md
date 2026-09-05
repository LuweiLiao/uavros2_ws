# vrpn_mavros

**中文 | [English](#english)**

ROS Noetic 包：将 Motive/VRPN 动捕位姿与速度桥接到 MAVROS，供 ArduPilot 使用。

---

## 中文

### 数据流

```text
VRPN 刚体 (ENU, m / m/s)
  → /<tracker>/pose , /<tracker>/twist
  → vrpn_flu_to_enu
  → /vrpn_enu/pose  , /vrpn_enu/twist
  → /mavros/vision_pose/pose
  → /mavros/vision_speed/speed_twist
  → VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE
  → ArduPilot（例如 DroneBridge UDP 14550）
```

仅位置也可工作（EKF 差分求速，噪声更大）。若 Motive/VRPN 提供 twist，**建议开启速度桥接**。

### 坐标与单位转换

| | VRPN（Motive） | MAVROS 输入 |
|--|----------------|-------------|
| 轴向 | 东-北-天（ENU） | 东-北-天（ENU） |
| 单位 | 米 / 米每秒 | 米 / 米每秒 |

坐标轴直接透传，不做旋转：

- 东 = X_vrpn × scale
- 北 = Y_vrpn × scale
- 天 = Z_vrpn × scale

姿态四元数也直接透传。MAVROS 再将 ENU 转为 NED 发给飞控。

VRPN 输入和 MAVROS 输入统一使用米、米每秒；默认 `scale:=1.0`。角速度按轴旋转，视为 rad/s。

### 依赖

```bash
sudo apt install -y \
  ros-noetic-vrpn-client-ros \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  geographiclib-tools

# MAVROS 需要的大地水准面数据（装一次即可）
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 编译

将本包放入 catkin 工作区 `src/`：

```bash
cd ~/catkin_ws/src
git clone git@github.com:LuweiLiao/vrpn_mavros.git
cd ~/catkin_ws
catkin build vrpn_mavros   # 或 catkin_make
source devel/setup.bash
```

### 运行

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch \
  vrpn_server:=192.168.43.5 \
  tracker:=tiltquad \
  fcu_url:=udp://:14551@192.168.43.9:14550
```

TCP 备用（通常更慢）：

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch \
  fcu_url:=tcp://192.168.43.9:5760@
```

改用 ATT_POS_MOCAP：

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch use_mocap:=true
```

关闭速度桥接：

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch enable_twist:=false
```

### 检查

```bash
rostopic hz /tiltquad/pose
rostopic hz /tiltquad/twist
rostopic hz /vrpn_enu/pose
rostopic hz /vrpn_enu/twist
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/vision_speed/speed_twist
rostopic echo /mavros/state   # connected: True
```

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vrpn_server` | `192.168.43.5` | VRPN 服务器地址 |
| `vrpn_port` | `3883` | VRPN 端口 |
| `tracker` | `tiltquad` | 刚体名称 |
| `fcu_url` | `udp://:14551@192.168.43.9:14550` | MAVROS 飞控 URL |
| `scale` | `1.0` | 兼容用缩放，统一单位时保持为 `1.0` |
| `enable_twist` | `true` | 是否转发 VRPN 速度 |
| `use_mocap` | `false` | `true` 时走 `/mavros/mocap/pose` |
| `frame_id` | `map` | 输出坐标系名 |

### 许可

MIT

---

<a id="english"></a>

## English

ROS Noetic package that bridges Motive/VRPN motion-capture pose and twist into MAVROS for ArduPilot.

### Data flow

```text
VRPN tracker (ENU, m / m/s)
  → /<tracker>/pose , /<tracker>/twist
  → vrpn_flu_to_enu
  → /vrpn_enu/pose  , /vrpn_enu/twist
  → /mavros/vision_pose/pose
  → /mavros/vision_speed/speed_twist
  → VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE
  → ArduPilot (e.g. DroneBridge UDP 14550)
```

Position-only also works (EKF differentiates velocity, noisier). Velocity is **recommended** when Motive/VRPN provides twist.

### Coordinate conversion

| | VRPN (Motive) | MAVROS input |
|--|---------------|--------------|
| Axes | East-North-Up (ENU) | East-North-Up (ENU) |
| Units | meters / m/s | meters / m/s |

Coordinate axes are forwarded without rotation:

- east  = X_vrpn × scale
- north = Y_vrpn × scale
- up    = Z_vrpn × scale

The orientation quaternion is also forwarded unchanged. MAVROS converts ENU → NED for the FCU.

VRPN and MAVROS inputs use meters and meters/second. Default `scale:=1.0`. Angular rates are axis-rotated and assumed rad/s.

### Dependencies

```bash
sudo apt install -y \
  ros-noetic-vrpn-client-ros \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  geographiclib-tools

# one-time geoid dataset for MAVROS
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### Build

```bash
cd ~/catkin_ws/src
git clone git@github.com:LuweiLiao/vrpn_mavros.git
cd ~/catkin_ws
catkin build vrpn_mavros   # or: catkin_make
source devel/setup.bash
```

### Run

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch \
  vrpn_server:=192.168.43.5 \
  tracker:=tiltquad \
  fcu_url:=udp://:14551@192.168.43.9:14550
```

TCP fallback (usually slower):

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch \
  fcu_url:=tcp://192.168.43.9:5760@
```

Use ATT_POS_MOCAP instead of vision_pose:

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch use_mocap:=true
```

Disable velocity bridge:

```bash
roslaunch vrpn_mavros vrpn_to_mavros.launch enable_twist:=false
```

### Check

```bash
rostopic hz /tiltquad/pose
rostopic hz /tiltquad/twist
rostopic hz /vrpn_enu/pose
rostopic hz /vrpn_enu/twist
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/vision_speed/speed_twist
rostopic echo /mavros/state   # connected: True
```

### Launch args

| Arg | Default | Meaning |
|-----|---------|---------|
| `vrpn_server` | `192.168.43.5` | VRPN host |
| `vrpn_port` | `3883` | VRPN port |
| `tracker` | `tiltquad` | Rigid-body name |
| `fcu_url` | `udp://:14551@192.168.43.9:14550` | MAVROS FCU URL |
| `scale` | `1.0` | Compatibility scale; keep at `1.0` for SI units |
| `enable_twist` | `true` | Forward VRPN twist as vision speed |
| `use_mocap` | `false` | `true` → `/mavros/mocap/pose` |
| `frame_id` | `map` | Output frame id |

### License

MIT
