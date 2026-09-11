# 固定外部依赖 / Pinned external dependencies

MAVROS is imported separately rather than embedded as an unregistered Git
repository. `dependencies.repos` pins upstream commit
`3a070636ec8943d3fffe6b67bc903c144f12fced`. The adjacent patch preserves global
parameter files for dynamically created plugins while overriding node identity
remaps locally. It is required together with `uav_gazebo/launch/apm.launch`.

在新的工作空间根目录运行（已有 checkout 先检查修改，不覆盖本地工作）：

```bash
vcs import src < dependencies.repos
git -C src/mavros apply --check ../../docs/dependencies/mavros-plugin-parameters.patch
git -C src/mavros apply ../../docs/dependencies/mavros-plugin-parameters.patch
```

`vcs` is provided by `python3-vcstool`. Build the imported packages before using
the workspace; system MAVROS alone does not include this patch. The workspace
ignores `/src/mavros/` only because its upstream revision and patch are tracked
here. Check that nested repository separately before updating the dependency.
After applying the patch, a local commit may be made there to keep its status clean.

For the contact simulation, ArduPilot remains a separate repository. The
`ardupilot-range-endpoint.patch` records the existing SITL-only UDP adaptation
on base `99a9622610de489d65b441e2d9efe46f453bdb0d`: a client connecting to port
9025 binds loopback port 9026, matching the Gazebo rangefinder destination.
It is specific to this single-vehicle simulation and is not a general upstream
UART fix. For the historical Run 38 baseline, apply it to a clean checkout of
that base together with the parameter patch, then rebuild SITL:

```bash
git -C /path/to/ardupilot apply --check /path/to/uavros2_ws/docs/dependencies/ardupilot-range-endpoint.patch
git -C /path/to/ardupilot apply /path/to/uavros2_ws/docs/dependencies/ardupilot-range-endpoint.patch
```

The historical run also had a diagnostic `DISTANCE_SENSOR` receive counter in
`GCS_Common.cpp`; it does not change the sensor data path and is not a required
dependency. Newer local `mode_impedance.cpp` tuning is outside this pinned
historical baseline. Do not label a newly built binary as the historical binary.

这些补丁仅用于复现本机仿真环境。本次没有推送依赖分支。

The accepted parameter file is also carried as `ardupilot-contact-parameters.patch`
against the same base, corresponding to local ArduPilot commit
`641838a7741a4e75805da4937bbe6cfbf7e78c4c`. On a fresh base checkout, apply it
with `git apply --check` followed by `git apply`, using the same absolute-path
pattern above. Do not apply it again if checking out that parameter commit.
The parameter commit has parent `e3d1e0511c48102ac40a0c4b5bcfea6b6896d6e9`,
a separately committed controller update. Checking out the parameter commit
therefore also includes that update. To reproduce historical Run 38 source, use
base `99a9622610de489d65b441e2d9efe46f453bdb0d` plus the two supplied patches;
do not substitute the newer branch tip for that historical baseline.

The local MAVROS checkout records its applied patch at
`95de1a72983f6e0f393d02db5f0e6f6c89d89b6a`; the import manifest deliberately pins
its upstream parent, so a fresh import does not require this unpublished commit.
