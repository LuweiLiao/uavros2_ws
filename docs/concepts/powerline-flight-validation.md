# Visual powerline perching: ROS 2 / ArduPilot SITL

Validated on 2026-09-17 with ROS 2 Jazzy and Gazebo Harmonic.
This is an AI-assisted simulation implementation, not hardware acceptance.

## Implemented sequence

1. Start Gazebo and the RGB-D camera, then start ArduPilot SITL.
2. Wait for estimator/GPS and camera readiness; arm normally in LOITER.
3. Take off in LOITER and approach the wire using RC inputs.
4. Acquire the wire in camera pixels; switch to GUIDED with acknowledged mode change.
5. Use image/depth line estimation and MAVLink NED velocity/yaw-rate targets to descend.
6. Confirm wire contact near the saddle, switch to LAND, and observe normal disarm.
7. Verify that the disarmed vehicle is still supported by the wire.

No forced arm/disarm, model teleport, kinematic flight animation, or collision disabling
is used. Gazebo model position is logged only for evaluation, not visual guidance.

## Full-scale tower corridor

Run `record_full_flight.py --scene corridor` from `.tmp/powerline_perching` with
ROS and the workspace sourced. Successful result:
`/home/llw/.ros/log/perching_flight_20260917_215220/result.json`.
Video: `/home/llw/powerline_perching_pip_20260917_215220.mp4` (44.96 seconds).
Bag: `/home/llw/Projects/uavros2_ws/rosbag2_2026_09_17-21_52_28`.
The near-side upper gray conductor is 13.536740 m above the ground at midspan.
LOITER takeoff targets 15 m before lateral approach and GUIDED visual descent.
Final height is 13.148827 m, lateral offset 1.4 mm, speed 0.000084 m/s;
target-wire contact and normal disarm are verified, with all owned groups closed.

This scene uses low-saturation connected components, depth separation of stacked
wires, self-airframe rejection and the existing NED line fit. A rotor-reflection
false detection observed in an interrupted trial is covered by a regression test.
Ten vision tests pass. Invalid vision requests zero velocity before resuming;
prolonged loss retains the existing LAND abort. The camera runs at 10 Hz, and the
video inset labels invalid/stale images explicitly. The ideal RGB-D, rigid wire
and known coarse approach do not establish field-flight accuracy or robustness.
No new ArduPilot control-law changes were required.

## Nominal run

- Result: `/home/llw/.ros/log/perching_flight_20260917_182724/result.json`
- Numeric control trace: same directory, `telemetry.jsonl`.
- MAVLink recording: same directory, `mavlink.tlog`.
- DataFlash: same directory, `sitl/logs/00000001.BIN`.
- ROS bag: `/home/llw/Projects/uavros2_ws/rosbag2_2026_09_17-18_27_30`.
- Gazebo-native screenshot: `/home/llw/.gz/gui/pictures/2026-09-17T18:29:51.223007948.png`.
- Screenshot provenance: `/home/llw/.gz/gui/pictures/PROVENANCE.txt`.
- Gazebo partition: `perching_flight_2231517`; ROS domain: 73.

Final independent Gazebo measurement, after normal disarm and four seconds of observation:

| Quantity | Value |
| --- | --- |
| Base-link height | 1.211983 m |
| Wire-centre height | 1.600000 m |
| Lateral displacement from wire | -0.001589 m |
| Speed | 0.000785 m/s |
| Wire contact | Present |
| Armed | False |
| Owned processes remaining | None |

The 1.6 mm displacement is one ideal simulation outcome, not an accuracy guarantee.
The expected support height is 1.212 m. The ROS bag contains 92,744 messages,
including 564 visual line estimates, IMU, contact, odometry and phase updates.
A prior full-flow run at `perching_flight_20260917_182023` also reached normal disarm
with wire contact, before independent ground-truth acceptance logging was added.

## ArduPilot state

- Repository: `/home/llw/Projects/ardupilot`.
- New branch: `codex/powerline-vision-perching`.
- Base commit: `641838a7741a4e75805da4937bbe6cfbf7e78c4c`.
- Tested SITL SHA-256: `01c9a97f9d630b06587d4fc5caf5a5a75294a52d0f52980b1dbbe717d50e55a9`.
- New profile: `powerline_perching/mav.parm`; no flight-control-law modification.
- Pre-existing UARTDriver.cpp and GCS_Common.cpp changes are preserved.
- No commit or push was performed.

## Scope and remaining limits

The detector segments the orange test wire and fits its image/depth points. It has
not been validated for unmarked grey conductors, clutter, glare, darkness or noise.
RGB-D depth is ideal Gazebo depth. The conductor is rigid, fixed and unenergized.
The scenario starts one metre to the side, uses a known coarse LOITER approach,
and relies on vision for final cross-track/yaw/height guidance. Along-wire position
is not observable from a uniform line and is held using autopilot velocity feedback.
No wind, flexible cable, latch or realistic powerline electrical interaction is modeled.

Six isolated tests verify projection signs, NED rotation, stale image rejection,
RGB/depth synchronization, absent targets and invalid depths. SDF checks, URDF parsing,
and builds of `uav_control` and `uav_gazebo` passed.

See the model README for the executable command and the deliberate vision-loss test.

## Annotated camera recording

The companion publishes `/powerline_perching/vision/annotated_image` at up to
20 Hz using the source RGB timestamp. Green pixels and the fitted image line
come from points accepted by the flight detector after depth and forward-region
filtering. A white cross marks the image center; a yellow arrow shows image
offset. Text reports NED horizontal distance to the line and yaw error. Invalid
detections clear the overlay and show `NO VALID WIRE`; the raw RGB stays intact.
The display is diagnostic and does not change the MAVLink control law.

The local native VideoRecorder extension in
`.tmp/powerline_perching/pip_recorder` records this image at bottom-right via
`record_full_flight.py`. It retains the 200 ms age gate and shows WAITING / STALE
instead of displaying older frames. This is a local plugin extension, not a stock
Gazebo GUI feature. It uses the native encoder, without offline video merging.

Validated run: `/home/llw/.ros/log/perching_flight_20260917_200819/result.json`.
Video: `/home/llw/powerline_perching_pip_20260917_200819.mp4` (803 decoded frames,
32.08 seconds). Provenance: `/home/llw/PROVENANCE.txt`. Visual inspection confirms
accepted wire markings during descent. The plugin logged 2120 composed and 23
stale render frames, maximum displayed age 198 ms. Render counts differ from
encoded frames because the native encoder resamples to the recording frame rate.
The full flight passed with contact, normal disarm and all owned processes closed.
Seven unit tests pass, including overlay clearing after invalid depth and source
image preservation.

## Vision-loss run

`/home/llw/.ros/log/perching_flight_20260917_183024/result.json` records deliberate
loss of vision two seconds after GUIDED starts. The control trace ends with
zero north/east/down/yaw-rate commands, followed by a `Visual target lost` abort.
The cleanup path requests LAND and observes normal disarm. All four owned process
groups exited. The run intentionally returns 1 / `passed=false`; it is a verified
abort-path test, not a successful precision landing. ROS bag:
`/home/llw/Projects/uavros2_ws/rosbag2_2026_09_17-18_30_30`.
