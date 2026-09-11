# ROS2 工作空间整理记录（2026-09-11）

## 版本边界

ROS2 主仓库保存源码、原模型资源、文档和固定依赖补丁。MAVROS 是通过
`dependencies.repos` 导入的独立仓库，补丁保存在 `docs/dependencies/`；不能只看
主仓库状态就判断 MAVROS 是否干净。ArduPilot 也保持独立仓库。

本次依次整理外部依赖及 MAVROS 接入、倾转接触控制、T4 原位适配和文档。
ArduPilot 只提交了与历史验收哈希相同的接触参数；后续控制器调参、UART 适配、
GCS 调试计数和本机参数转储未纳入这次 ArduPilot 提交。必要的 UART 适配以
明确标注用途的补丁随 ROS2 文档交付，不宣称为通用上游实现。

## 暂缓交付的改动

220 个模型文件的批量修改仅删除 Classic `<script>` 材质块及改动空白，
其中包括草地、树枝、树皮和木地板纹理。删除后的空 `<material>` 不代表材质移植完成。
完整差异已保存，相关文件恢复至整理前 HEAD；后续按颜色或纹理逐项迁移并做原生 GUI 对照。
T4 已明确转换为 SDF 颜色的八个文件保留，接触模型两处纯色材质也已明确表达。

本机归档位置：`.tmp/workspace-organize-20260911-140244`。

- `before.patch`：整理前全部已跟踪差异，包括暂缓的材质删除；不是单独的材质补丁，勿整份盲目应用。
- `deferred-material-files.json`：220 个暂缓文件清单，可配合补丁按路径恢复。
- `untracked/`：重复子模型、三个别名软链接、两个 `model.sdf.pre_*` 和未接入的 `common_models`。
- `obsolete-install/`：对应过期安装产物，避免旧模型资源混入运行搜索路径。
- `mavros-before.patch`、`ardupilot-before.patch`：各独立仓库整理前的工作树差异。

ROS1 原树未修改。重复子模型在归档前与原 canonical 模型逐字比较：归一化模型名及
网格 URI 后 SDF 完全相同，网格字节相同。原 launch 路径可解析两机型的全部资源。

## 忽略与证据

`.tmp/`、build/install、ROS 日志、SITL 地形缓存、EEPROM、遥测日志、rosbag 和
Python 缓存保持忽略；归档后备份不再留在正式模型目录。没有新增全局 `*.sdf`、
`*.parm`、图片或视频忽略规则。已有跟踪的测试 bag 保留。

本次没有删除历史 rosbag、DataFlash 或运行证据，也没有为清理 Git 状态移动它们。
忽略不等于释放磁盘：整理前 `.tmp` 约 25 GB，47 个 rosbag 目录约 8.2 GB。
后续磁盘清理应先确认需长期保存的基线与失败证据，再另行处理。

## 本轮检查及限制

- 7 个相关包在已有根目录构建空间增量构建成功：libmavconn、mavros_msgs、mavros、
  mavros_extras、rotors_gazebo_plugins、uav_gazebo_plugin、uav_gazebo。
- 12 项选定 CTest 通过：MAVROS 10 项、extras 的 SRTM、ADM002 协议；未运行完整工作空间测试或完整 lint/benchmark 集。
- 正式 `apm.launch` 在独立 ROS domain、无飞控连接情况下，成功加载 distance_sensor
  参数文件并发布原名 rangefinder 话题；测试进程正常退出。
- 依赖补丁在固定干净源文件上可应用，并与本机对应文件逐字一致；参数无重复项，
  12 路功能映射/PWM 范围及历史验收 SHA256 匹配。
- `gz sdf -p` 对接触 world 和 T4 world 均成功，include 全部展开，网格 URI 均可从
  原资源路径解析。命令行解析器使用与 launch 相同目录列表作为 `SDF_PATH`。
- 编译仍有系统 MAVLink 头文件的 packed-member 警告；T4 仍有原有
  `angular_velocity`、`linear_acceleration`、`use_parent_model_frame` SDF 字段警告。
- 本轮未重飞、未进行材质 GUI 对照；历史飞行验收不能等同于当前固件的重新验收。

检查细节保存在上述本机归档目录及默认 `log/` 中。所有提交均为本地提交，未推送。
