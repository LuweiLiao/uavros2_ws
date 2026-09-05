# UAVROS ROS 2 workspace

This workspace is the ROS 2 Jazzy / Gazebo Harmonic migration area. The ROS 1
catkin workspace at `/home/llw/Projects/uavros_ws` remains the reference implementation
and is not mixed with this workspace.

The migration preserves every ROS 1 package directory and original relative
file name.  This includes launch-file basenames and extensions: an original
`.launch` file is ported in place with ROS 2's supported XML launch syntax; it
is not replaced in the formal package by a newly named `.launch.py` file.
Temporary differently named launch adapters belong only under `.tmp/` and are
not part of the delivered source tree.

## Source layout

### 单仓库交付与本轮倾转模型迁移（2026-09-06）

目标远端：`git@github.com:LuweiLiao/uavros2_ws.git`。RotorS、mav_comm、
uav_simulator 及 TSD 模型均以原路径普通源码纳入，不使用子模块。
原 `src/uav_simulator/.gitmodules` 与 TSD 嵌套 Git 元数据保留在本机
`.tmp/migration/git-metadata-20260906/`，可恢复；它们不属于交付源码。
原有许可证和版权声明保留。大尺寸 STL 通过 Git LFS 保存，克隆后需
`git lfs pull`；这不是子模块。运行日志、构建结果和临时测试工具不提交。

来源记录：ROS1 工作区 `ea17560f79c910623db00350d9b30e67784fc5e0`；
uav_simulator `4e433449d4b8b88ee104dbc2ff772fdf6685b645`；
RotorS `https://gitee.com/Luviewer/rotors_simulator.git`，
`656e83a6154bf53e792b75df68c572c2e10a9ec0`；TSD 模型
`git@github.com:LuweiLiao/tsd_model.git`，
`fa671a1cf23b21a8939ed52b607723d52fe62496`。这些提交标识记录来源，
并不替代本地 ROS1 文件基线；已有未提交变更也须按实际文件比对保留。

提交身份仅在本仓库配置为 `codex <noreply@openai.com>`。邮箱核实自
[OpenAI Codex 官方 GitHub 署名测试](https://github.com/openai/codex/blob/52e73e3a548ae5310c7765995b9803dd538b82b0/codex-rs/app-server/tests/suite/v2/git_attribution.rs#L51)。
此工作由 AI 辅助完成，不表示 OpenAI 对飞控或飞行安全作出认证。

本轮 `tilt_quadcopter` 保留四臂八桨与四舵机的原结构，迁移差异和
±90° 验收标准见原目录的
[模型说明](src/uav_simulator/uav_gazebo/models/tilt_quadcopter/readme.md)。
用户随后明确将航向误差视为合理范围：当前验收改用重力相对倾角误差≤5°，
航向只记录；其余高度、速度、10秒保持及正常降落要求不变。
**新口径下连续两轮正常启动的13阶段完整验收通过**：
`tilt_standard_20260906_041319`、`tilt_standard_20260906_041934`。
使用相同插件库、同一SITL、相同参数和测试脚本，无诊断延时或航向补偿；
包含悬停、±30/60/90°、每次回正、正常LAND及非强制解除武装。
两轮保持窗口倾角误差最大分别4.9743°、4.9682°，高度2.915–3.037m，
水平速度最大0.0421m/s。独立逐阶段重放、15路ROS读回、DataFlash、
Gazebo原生截图来源、路径/插件契约及清理审计均通过。
综合证据：`/home/llw/.ros/log/tilt_standard_20260906_041934/GOAL_ACCEPTANCE.json`。
共用RotorS改动后的 `quad_standard_20260906_042606` 回归亦通过：
tsduav_quad悬停10.000s仿真/10.013s墙钟，高度0.463–0.539m，正常降落。
所需6包构建通过；自建GUI/server/SITL/桥接/录包均退出，端口释放。

以下航向相关失败均为旧口径记录，原始结果保留，不重新改判。
旧口径下最后回正约5.02°的完整姿态误差导致失败；现仍不宣称航向误差
已消除。新测试因不等待航向收敛，末10秒真航向均值约-5.98°/-5.91°，
飞控估计约-0.24°；航向数据没有隐去，未修改飞控/EKF/传感器参数。
按用户要求补做 ROS1 对照：当前原插件因必需测距仪缺失而退出；选择原仓库
历史 `39a0ccc` 同名插件后，未改的原模型已连续两次完成全部 13 阶段和正常降落。
此历史基线也有约 4.78° 的末段估计/真值航向差，不能把它全部归因于 ROS2。
ROS2 恢复原更新阶段后一次完整通过、复测失败；恢复原旋翼直接速度状态
接口后旧航向门限仍在最后回正失败。不能据此声称这些接口修正消除了航向偏差。
具体基线选择、成功与失败记录均见模型说明，未修改 ROS1 源码或飞控算法。
临时 Classic 副本仅改已批准的 Izz=0.06 后也通过全部阶段，未把该诊断副本
混作原始基线。另已恢复原 ROS 指令→世界桥→RotorS 路径及原发布顺序，
不再由倾转飞控插件绕过 ROS 直接发 Gazebo 指令。新版 `035242` 的两条
ROS 指令及13路反馈读回通过，±90°保持通过，但最后回正仍失败；
末10秒真航向均值 -5.0209°、估计航向 -0.2121°，正常降落和清理完成。
因此指令通路职责修正已验证，不代表残余航向问题已解决。
本次完成的是倾转模型仿真稳定性专项；不代表其它模型、MAVROS2控制链路、
整个工作区移植或实机安全验收。倾转模型仍使用已批准的Izz=0.06仿真近似。
远端SSH连接仍被关闭；本地验收通过不等于代码已成功推送。
本轮路径审计：ROS1 1417 个普通文件加 1 个原符号链接，ROS2 1416 个普通文件
加同一符号链接；唯一缺项是已取消子模块的 `.gitmodules`，没有新增源路径，
全部源文件已纳入主仓库且不存在 Git 子模块索引项。倾转顶层模型的 include、
joint、plugin 契约与原文件一致（比较时仅规范原 XML 属性引号）。

### 原包边界

Only ROS 1-derived packages belong under `src/`. Upstream reference checkouts
are retained under `.tmp/deps/upstream_reference_checkouts/`; their repository
list is `.tmp/deps/ardupilot_ros2.repos`. No replacement package boundary is introduced for the
TSD model or its plugins:

| Package | Responsibility |
| --- | --- |
| `uav_simulator/uav_gazebo` | Vehicle descriptions, SDF models and launch entry points (same role as ROS 1) |
| `uav_simulator/uav_gazebo_plugin` | ROS 1 plugin sources mirrored for the Gazebo Harmonic port |
| `rotors_simulator/rotors_gazebo_plugins` | Original RotorS motor and world-level ROS bridge ports; remaining original plugins retain their paths |

## Current standard workflow validation (2026-09-05)

The quad dependency set is now built together at `.tmp/install/colcon` from
a clean environment containing only the system ROS 2 Jazzy underlay. Colcon's
generated `setup.bash` sources that underlay automatically; no dated workspace
overlays, custom environment wrapper, or shell startup-file changes are needed.

The GUI/SITL regression `quad_standard_20260905_233359` passed with the original
launch and all four checked libraries actually mapped from this new build:

* 0.5 m target, 10.000 s simulated / 10.008 s wall continuous hover;
  measured altitude 0.463–0.539 m, horizontal speed peak 0.0283 m/s.
* LAND acknowledged, followed by automatic non-forced disarm; DataFlash
  confirms the original yaw gain 0.12 and non-forced disarm event.
* Four original world-bridge ROS motor outputs recorded and read back:
  285,714 MCAP messages total, at least 71,428 per channel.
* Three built-in Screenshot PNGs saved at Gazebo's default location,
  `/home/llw/.gz/gui/pictures/`, with `PROVENANCE.txt`. No video was recorded
  in this regression and no external capture was used.
* All owned process groups exited; ports 9002/9003/5760/5762/5763 released.
* 1417/1417 source file paths match the ROS 1 baseline, with no added/missing
  paths. The installed top-level quad model is byte-identical to ROS 1.
  Sixteen layout checks pass. Motor physics and bridge contract suites each
  pass three repetitions against the new libraries; the package's five
  ADM002 protocol unit cases also pass.

Runtime result and independent numeric analysis:
`/home/llw/.ros/log/quad_standard_20260905_233359/RESULT.json` and `metrics.json`.
Rosbag used its default output URI:
`/home/llw/Projects/uavros2_ws/rosbag2_2026_09_05-23_34_04/`.
SITL used its normal working directory, `/home/llw/Projects/ardupilot/ArduCopter/`;
DataFlash is in `logs/00000001.BIN`. ROS/Gazebo log/home and partition variables
were not overridden. Source/model/launch implementations were unchanged in
this standard-workflow test.

An earlier attempt, `quad_standard_20260905_233249`, failed only the temporary
log-readiness parser because Gazebo splits colored text with ANSI sequences.
It started no SITL and cleaned up fully. The temporary parser was corrected
without editing the original launch, model or plugins; its failed result is
retained, not counted as acceptance.

Scope remains **quad GUI + SITL + motor ROS telemetry**, not ROS 2 application
control through MAVROS or full workspace migration. The clean all-package
build stopped at `rotors_hil_interface`: the installed MAVROS conversion header
requires `mavconn/mavlink_dialect.hpp`, and system `libmavconn` is absent.
Its original source remains untouched; its incomplete generated install was
moved recoverably to `.tmp/build/colcon/rotors_hil_interface/incomplete_install`
so sourcing the valid install does not advertise a broken package. No source
package was deleted or marked ignored. The subsequent official
`--packages-up-to uav_gazebo` build succeeded for all six required packages.
Whole-workspace rosdep checking also reports missing `topic_tools`,
`vrpn_mocap`, `mavros`, `mavros_extras` and an unresolved `libprotobuf-dev`
rosdep key. These remain explicit follow-up items, not hidden dependencies
of the accepted quad startup.

## Historical quad/bridge validation (2026-09-05)

Historical accepted quad/bridge build: `.tmp/install/rotors_bridge_mapping_20260905`,
then layered with `.tmp/install/quad_acceptance_20260905` for ArduRotorNorm.
These dated overlays are superseded for daily startup by the unified build above. The clean
GUI + SITL run `quad_acceptance_ros_mapping_20260905_r3` passed takeoff,
10.000 s simulated / 10.026 s wall continuous hover, LAND and automatic
non-forced disarm. Hover altitude was 0.465–0.539 m. All owned processes
exited and simulator ports were released.

The original world-level `librotors_gazebo_ros_interface_plugin.so` now
automatically forwards the four original motor measurements to ROS2.
The only approved naming exception is `/prop_speed/0..3` →
`/prop_speed/motor_0..3` on the ROS2 side: numeric-leading name tokens are
invalid in ROS2. SDF/Gazebo paths, model nesting, command topics and motor
indices remain unchanged. No general numeric-token rewrite is applied.

Evidence: `.tmp/runtime/quad_acceptance_ros_mapping_20260905_r3/FINAL_AUDIT.json`,
`recording_validation.json`, `analysis/metrics.json` and `gazebo_builtin/`.
508,311 MCAP payloads and 439 native video frames were read successfully.
Each motor topic has over 72,000 samples; signed command/direction agreement
is 100% for all four channels. The original normalizer allows negative
startup commands, so verification compares measured direction with the
signed command instead of clipping or changing the ROS1 behavior.

The real-library bridge suite covers all 15 GZ→ROS and 4 ROS→GZ message
classes, duplicate registration, TF, reload, approved output mapping and
bounded bidirectional forwarding. Motor/bridge and seven motor-contract
scenarios pass five repetitions; all 16 model/layout checks pass, with
1417/1417 source paths preserved. See `.tmp/rotors_bridge_20260905/contract_audit.md`
for scope and historical failed/intermediate runs.

This accepts the quad bridge slice, not the full workspace migration.
Other original controller/sensor plugins, package runtime roles and models
still require migration and acceptance. There are 21 actual packages plus
three original catkin_simple test-fixture manifests.

The generic `copter.parm` sets `ATC_RAT_YAW_P=0.3`, unlike the original
quad's `tstuav_t4_adrc.param`, which specifies `0.12`. The accepted runs restore
that original value using a temporary, source-derived `ros1_yaw.parm` overlay.
Other parameters in the legacy file have **not** been fully validated against
the current ArduPilot version; do not silently load it wholesale. The UDP
plugin also restores ROS 1's bounded receive wait. Motor mapping, SDF model
resources and the original motor physics equations are unchanged.

The official GUI entry point now needs only the unified setup in a clean
terminal (do not load ROS 1 or the old dated overlays in that terminal):

```bash
source /home/llw/Projects/uavros2_ws/.tmp/install/colcon/setup.bash
ros2 launch uav_gazebo spawn.launch world_name:=tsduav_quad
```

`gui` defaults to true and `paused` to false. If an existing terminal still has
the previous manual output overrides, clear `ROS_LOG_DIR`, `GZ_HOMEDIR`,
`GZ_LOG_PATH` once, or use a clean terminal. `GZ_PARTITION` is optional for
isolated sessions, not required by this single-instance workflow.

`tsduav_quad.world` does not override the GUI plugin list. Gazebo Harmonic
loads its normal GUI configuration (`~/.gz/sim/8/gui.config`), retaining the
official default entity tree, inspector, context menu, visualization tools,
transform controls and world controls. On 2026-09-06 the local configuration
was verified byte-identical to the installed Harmonic default. No custom GUI
configuration or launch argument is needed. Changes take effect on the next
launch; do not start a second Gazebo instance alongside an existing run.

World axes (user-requested display addition, 2026-09-06): the existing static
`ground_plane/link` in `tsduav_quad.world` carries three GUI-visibility-only
visuals. Red is +X, green +Y, blue +Z (up), each extending one metre from the
world origin. There are no added models, links, collisions or physics plugins;
the vehicle and default GUI configuration are unchanged. These are visual
references, not ArduPilot's NED axes or a new ROS TF publisher.

The historical temporary, bounded acceptance test used that same official
entry point (reference only, not the new default-location workflow):

```bash
python3 /home/llw/Projects/uavros2_ws/.tmp/tsduav_quad/tools/run_acceptance.py quad_acceptance_manual_001 --ros1-yaw --record --motor-overlay /home/llw/Projects/uavros2_ws/.tmp/install/rotors_bridge_mapping_20260905
```

This runner still hard-codes centralized `.tmp/runtime` outputs. It must be
adapted before reuse under the user's new default-location rule; the command
above records the historical acceptance procedure, not a recommended daily
startup command. Existing evidence is retained unchanged. Any future runner
must close previous owned Gazebo server, GUI, SITL and recorders before
starting another test. This is diagnostic infrastructure in `.tmp`, not a
new package or replacement launch file.
The diagnostic runner uses isolated `ROS_DOMAIN_ID=175`; the manual GUI
command uses the terminal's ROS domain. GUI startup alone does not start
SITL: its separate-process responsibility is unchanged from ROS1.

`GZ-LIFE-001`: after every test, LAND and observe disarm, save recordings,
close all owned simulation processes, and verify they have exited before
opening another Gazebo. The migration skill enforces this cleanup on success,
failure and retries. A closed launch wrapper alone is not proof that its
Gazebo children have exited.

The `tsduav_quad` model remains under the original relative path in both
workspaces:
`src/uav_simulator/uav_gazebo/models/tsd_model/tsduav_quad/`. The ROS 2 copy is
not moved into a new description package and is not flattened. The plugin
package remains a mirror of the ROS 1 `uav_gazebo_plugin`; for the current
`tsduav_quad` slice, `ArduRotorNormPlugin.cc` is the Gazebo Sim API port under
the same class, source and library names, and the RotorS motor model is ported
under the same names in `rotors_gazebo_plugins`. No model tree is renamed.

## `tsduav_quad` plugin contract

The authoritative model is
`src/uav_simulator/uav_gazebo/models/tsd_model/tsduav_quad/models/tsduav_quad/model.sdf`.
Its plugin list is deliberately unchanged from ROS 1:

* four `librotors_gazebo_motor_model.so` instances, one for each
  `tsduav_quad_prop_[0-3]_joint`;
* one `libArduRotorNormPlugin.so` instance for the ArduPilot FDM/UDP link.

The four SDF instance names are unchanged.  Gazebo Sim needs a class alias for
each name, so the same `rotors_gazebo_motor_model` library registers the four
`tsduav_quad_prop_[0-3]_plugin` aliases; this is a loader compatibility detail,
not four new plugins.

`TsdRotorAero.cc` and `libTsdRotorAero.so` are not ROS 1 files. They are absent
from the formal `src/`, `build/`, `install/`, model, and launch contract of this
workspace. A historical prototype carrying that name is retained only under
`.tmp/migration/rejected/` for audit history; it is not a package, is not
installed, and must never be referenced by a formal SDF or launch file. If an
older sibling build directory still contains that library, it is stale
generated output from the rejected experiment and must not be added to
`GZ_SIM_SYSTEM_PLUGIN_PATH`.

The ROS 1 `tsduav_quad` model directory contains no vehicle-specific SITL
defaults file beyond its original `tstuav_t4_adrc.param`.  The optional
`tsduav_quad_sitl.parm` overlay used during ROS 2 diagnostics is kept under
`.tmp/migration/experimental/` and is never installed as part of the model.
The ROS 1-style package has no additional package-level `config/` model
directory. Planning maps for vehicles not yet in scope live only under
`.tmp/migration/`.

## Gazebo-native screenshots and recordings (GZ-CAP-001, hard rule)

**新增项目硬约束：截图和录像一律使用 Gazebo/Gazebo Sim GUI 内置工具；禁止
电脑桌面/窗口截图和任何外部录屏工具。** 这条约束适用于调试、开发、对比、
文档和验收，不因文件只是临时产物而放宽。

这条规则现在是本迁移项目的强制执行门槛：

* 截图只能由 Gazebo GUI 的 `Screenshot` 插件通过 `/gui/screenshot` 生成。
* 视觉录像只能由 Gazebo GUI 的 `VideoRecorder` 插件生成并保存。
* 不得使用电脑桌面/窗口截图、系统录屏、OBS、`ffmpeg`、GStreamer 或其他
  外部捕获链路；也不得把外部相机流编码文件当作 Gazebo 视觉证据。
* 如果 Gazebo GUI 没有加载这些内置工具，本次运行只能报告“没有视觉证据”，
  不得用电脑截屏或外部录像补齐。

`GZ-CAP-001` is mandatory and has no desktop-capture exception: all project
screenshots and visual recordings, including temporary debugging artifacts,
must be produced inside the running Gazebo GUI by its built-in capture tools.
This user-specified rule takes precedence over a test script's default capture
path or any convenience tooling; a run without Gazebo capture support simply
has no visual evidence yet.

This is a project-level hard constraint for every run, including debugging,
comparison, documentation, and acceptance. Any screenshot or visual recording
that is not created by a Gazebo/Gazebo Sim built-in tool is rejected and must
not be kept as project evidence. This applies even when the file is only a
temporary diagnostic artifact.

All screenshots and videos must be made by Gazebo/Gazebo Sim itself. A
computer desktop screenshot, window capture, screen recorder, or camera-stream
recording is not accepted.

This is also an execution constraint on the development workflow: the
assistant must not invoke a desktop/window capture API or an external recorder
even for a temporary inspection image. If Gazebo's GUI capture tool is not
available, the run has no visual evidence yet; do not create a substitute.

This rule also governs upstream package examples: a camera stream shown with
GStreamer or QGroundControl is for live monitoring only. It must never be
recorded or submitted as the project's visual evidence.

* For a PNG, use the built-in GUI `Screenshot` plugin and its native save
  location. With the GUI running, its official service is `/gui/screenshot`
  (`gz.msgs.StringMsg` request, `gz.msgs.Boolean` response). The request `data`
  value is an output directory; when using this service, resolve the native
  destination first instead of forcing a `.tmp/runtime` path. Record the
  actual file path reported by Gazebo.

* For a visual video, use Gazebo Sim's built-in GUI `VideoRecorder` plugin and
  save the file through its Gazebo save dialog at its normal destination.
  Harmonic's default GUI includes `Screenshot`; add `VideoRecorder` from
  Gazebo's GUI plugin menu when recording is needed. The world deliberately
  does not override the default GUI layout. If a GUI configuration does not
  provide these tools, enable them through Gazebo's official GUI plugin menu;
  do not substitute a desktop recorder, `ffmpeg`, GStreamer, OBS, or a
  recording of a camera stream. Camera streaming plugins remain available for
  the simulation, but their externally encoded output is not flight-test
  evidence.
* For a replayable simulation-state log (not a visual video), use Gazebo's
  own `gz sim --record` option and default recording location; no centralized
  `--record-path` is required.

Every evidence directory must include a `PROVENANCE.txt` with the exact world,
`GZ_PARTITION`, Gazebo plugin/service used, and output path. A visual artifact
without that provenance file is invalid and must not be cited or retained as
project evidence. Files such as `gazebo_gui.png` made by a desktop/window
capture are invalid and must not be cited for acceptance.
Telemetry plots or other offline analysis figures are not screenshots; they
must be clearly labeled as analysis outputs, and must never
be presented as a Gazebo view or used to replace the required Gazebo capture.

## Generated and temporary data

User revision (2026-09-05): the centralized runtime-output constraint is
cancelled. Use each tool's normal default locations, rather than creating
`.tmp/runtime` or overriding paths to collect all outputs together:

* ROS 2 normally writes logs under `~/.ros/log/` when no log/home override is set.
* Gazebo uses its component-specific native locations, commonly under `~/.gz/`;
  screenshots and videos use the built-in tools' normal save locations.
* SITL state and logs, and MAVProxy logs, normally follow their working
  directory. Check that directory and existing state before launch; do not
  blindly use `--wipe` outside a verified disposable SITL directory.
* Rosbag and other tools retain their own output defaults. Choosing default
  locations does not automatically enable recording.

Temporary diagnostic scripts, one-off gates, shell wrappers and experimental
source still belong under `.tmp/`, not in deliverable packages. Existing
`.tmp/build/` and `.tmp/install/` staging is unchanged. Historical logs and
acceptance evidence remain in place; no move or deletion is implied. Native
capture provenance and verified cleanup remain mandatory. Runtime outputs
must not be added to a package's installed files. The final ROS 2 deliverable
retains the original source architecture under `src/`.

## Official runtime entry point

The migrated UAVROS entry point keeps the ROS 1 launch basename and is invoked
with the original generic launcher:

```bash
source /home/llw/Projects/uavros2_ws/.tmp/install/colcon/setup.bash
ros2 launch uav_gazebo spawn.launch world_name:=tsduav_quad
```

This starts the official Gazebo Sim server and GUI through
`ros_gz_sim/launch/gz_sim.launch.py` while preserving the ROS 1 argument names
and model search groups. ArduPilot SITL and MAVProxy remain separate processes,
as they were in the ROS 1 workspace; a temporary diagnostic runner may compose
them under `.tmp/`, but no new formal `tsduav_quad` launch basename is added.

### SITL and flight operation

After the Gazebo model has loaded, start SITL separately. The exact defaults
used in this regression were the two upstream defaults plus the historical
source-derived single-parameter overlay; that overlay is an input, not a
runtime output directory:

```bash
cd /home/llw/Projects/ardupilot/ArduCopter
/home/llw/Projects/ardupilot/build/sitl/bin/arducopter \
  --model Gazebo --speedup 1 \
  --defaults /home/llw/Projects/ardupilot/Tools/autotest/default_params/copter.parm,/home/llw/Projects/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/home/llw/Projects/uavros2_ws/.tmp/runtime/quad_acceptance_ros_mapping_20260905_r3/ros1_yaw.parm \
  --sim-address=127.0.0.1 --sim-port-in 9003 --sim-port-out 9002 -I0
```

No `--wipe` is used. The successful run left normal SITL `eeprom.bin` state in
that directory. On subsequent sessions, saved values can override defaults;
verify `ATC_RAT_YAW_P` is 0.12 before flight. Do not load the entire legacy
parameter dump or erase saved state to make a test pass.

For interactive control, connect in another terminal:

```bash
cd /home/llw/Projects/ardupilot/ArduCopter
/home/llw/venv-ardupilot/bin/mavproxy.py --master=tcp:127.0.0.1:5760
```

Wait for EKF/home and pre-arm checks, verify the yaw parameter, then enter
`mode GUIDED`, `arm throttle`, and `takeoff 0.5` one at a time, checking each
result. Finish with `mode LAND`, wait for `DISARMED`, save any native capture,
and close MAVProxy, SITL and the Gazebo launch, verifying both GUI/server exit.
This interactive MAVProxy sequence is guidance; the measured acceptance above
used the bounded temporary MAVLink flight gate directly on TCP 5760, not
MAVProxy or MAVROS. Never run two flight controllers against the same test.

## Build the accepted quad dependency set

Use a clean build terminal containing only system ROS 2, not the workspace
overlay being rebuilt. System glog/MAVLink/message dependencies are now
installed; full-workspace dependency gaps are listed above. Build/install stay
under `.tmp` as previously agreed; logs use colcon's default location.

```bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
cd /home/llw/Projects/uavros2_ws
colcon build --base-paths src --packages-up-to uav_gazebo --symlink-install \
  --build-base /home/llw/Projects/uavros2_ws/.tmp/build/colcon \
  --install-base /home/llw/Projects/uavros2_ws/.tmp/install/colcon \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Then open a separate run terminal and use the two-line GUI command above.
`--symlink-install` links supported resources to source; C++ changes still need
recompilation and running simulators must be stopped before rebuilding plugins.

The explicit paths preserve existing build/install staging. Colcon logs now
use the tool's default `log/` directory in the workspace; runtime logs and
Gazebo captures follow their respective native defaults as described above.
