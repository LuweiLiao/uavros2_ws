# Gazebo live camera picture-in-picture recorder

Local extension of Gazebo Sim 8 `src/gui/plugins/video_recorder/VideoRecorder`.
Upstream source: https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/gui/plugins/video_recorder
The original Apache-2.0 notices are retained. System libraries are not replaced.

Changes: subscribe to `/powerline_perching/vision/annotated_image`, keep a bounded
timestamped queue, and draw the latest non-future RGB image in the bottom-right
38% of the native captured frame before the original VideoEncoder.AddFrame call.
The companion publishes actual accepted wire pixels in green, a fitted line,
image-center crosshair, pixel-offset arrow and metric/yaw error. Rejected frames
explicitly say NO VALID WIRE. Source timestamps are preserved; raw images are
unchanged. A 20 Hz timer keeps the view annotated before arming and after landing.
Sensor age is shown in milliseconds. Images older than 200 ms are not displayed;
the inset says WAITING / STALE instead. No desktop recording, external encoder,
offline video merging, artificial camera images, or flight-control changes.

This is a modified native VideoRecorder, not an unmodified built-in feature.
The inset is composed into recorded video frames, not the interactive 3D viewport.
The viewport and sensor are asynchronous; the displayed age reports that limit.

Build:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S .tmp/powerline_perching/pip_recorder -B .tmp/powerline_perching/pip_recorder/build
cmake --build .tmp/powerline_perching/pip_recorder/build -j4
```

Run from the workspace with ROS and the workspace sourced:

```bash
/home/llw/venv-ardupilot/bin/python3 .tmp/powerline_perching/record_full_flight.py
```

The wrapper explicitly loads the local library by absolute path only in this Gazebo launch,
uses a local copy of the helper to invoke native OnStart/OnStop/OnSave slots, and records
before arming until after normal disarm. Output is in the native save dialog's
home directory. PROVENANCE.txt explicitly identifies this local extension.
