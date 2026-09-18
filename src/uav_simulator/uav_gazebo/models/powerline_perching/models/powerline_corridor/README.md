# Full-scale transmission corridor

This scene reuses the repository's existing lattice-tower CAD geometry:
`models/tsd_model/tsd_electric_tower/models/tsd_electric_tower_base/meshes/base.STL`.
No downloaded third-party asset or FreeCAD installation is required. The original
mesh is untouched. `meshes/steel.stl` and `meshes/insulators.stl` partition its
28,528 triangles for separate material colors; their union preserves the source
geometry. Original asset authorship/licensing follows the parent repository;
no new external license is asserted for these derived meshes.

## Geometry

- Two towers at x = -50 m and +50 m, approximately 17.64 m tall.
- Four suspended 24 mm diameter conductors at y = +/-3.35 m, following the
  existing CAD asset's two crossarm levels. This inherited four-wire layout is
  a conceptual test corridor, not a certified three-phase/double-circuit design.
- Suspension heights 12.287 m and 14.787 m, matched to the CAD insulator ends.
- Catenary z(x) = attachment - sag + a * (cosh(x/a) - 1), a = 1000 m;
  sag = 1.250260 m. Lowest conductor center = 11.036740 m.
- 16 mm diameter overhead earth wire, approximately 0.75 m sag.
- Existing disc insulators individually colored green, steel suspension cores,
  clamps, eight simplified Stockbridge-type dampers, concrete footings.
- Each conductor uses 100 one-metre chord segments with matching cylindrical
  collision shapes. Maximum catenary/chord vertical deviation is about 0.125 mm.

Conductor visuals are merged into five continuous STL meshes (12-sided circular
sections) to reduce render overhead. Cylindrical collisions retain their original
dimensions and are grouped into five-metre static links for broad-phase rejection.

Tower visuals retain the detailed CAD lattice. DART does not create these STL
collision shapes in this setup, so primitive stepped envelopes conservatively
bound the tower core and crossarms. These solid envelopes prohibit flying through
lattice openings. Footings, suspension cores, clamps and wires have primitive
collisions; small damper weights and detailed insulator skirts are visual only.
The wire is rigid/static: sag is geometric, not a flexible cable solver. No wind,
energization, conductor temperature, strand grooves, or structural design check
is modeled.

## Open In Gazebo

From `/home/llw/Projects/uavros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch uav_gazebo powerline_perching.launch world_name:=powerline_corridor
```

The world is for corridor inspection; it does not launch SITL or fly a vehicle.
The `powerline_corridor_flight` world adds the original dynamic vehicle at
(0, -4.35, 0.174). The controller's `--scene corridor` selects a 15 m LOITER climb
and 10 Hz gray RGB-D line detection. It targets the upper near-side wire, y = -3.35 m,
whose midpoint is 13.536740 m high. Ground truth is used only for final evaluation.
The previous `powerline_perching_flight` scene remains available.

Full sequence with annotated native camera-inset recording:

```bash
/home/llw/venv-ardupilot/bin/python3 .tmp/powerline_perching/record_full_flight.py --scene corridor
```

Rebuild the generated scene with:

```bash
/usr/bin/python3 .tmp/powerline_perching/build_corridor.py
colcon build --packages-select uav_gazebo --symlink-install
```

Native screenshot/video inspection and owned-process cleanup:
`.tmp/powerline_perching/preview_corridor.py`. Screenshots use Gazebo's
`/gui/screenshot` and native Pictures directory; video uses the unmodified native
VideoRecorder and its home-directory save destination. Each output directory
contains `PROVENANCE.txt`. No desktop capture or offline compositing is used.

## Verified Output

Run: `/home/llw/.ros/log/corridor_preview_20260917_212249/result.json`.
Video: `/home/llw/powerline_corridor_20260917_212249.mp4` (191 decoded frames,
7.6 simulation seconds). Tower/hardware view:
`/home/llw/.gz/gui/pictures/2026-09-17T21:23:21.935718362.png`.
Build and SDF validation pass; 546 visuals and 554 uniquely named collisions.
Binary facet comparison confirms exact preservation of all source CAD triangles
across the two material meshes. The final Gazebo log has no failed collision
construction or missing resources. All owned process groups exited.

## Full Flight Validation

Successful flight: `/home/llw/.ros/log/perching_flight_20260917_215220/result.json`.
Annotated PiP video: `/home/llw/powerline_perching_pip_20260917_215220.mp4`
(1125 decoded frames, 44.96 simulation seconds). The successful version has
51 visuals, 554 collisions and 23 static links after visual batching and collision
partitioning. PiP reports 9487 composed render frames, 73 stale render frames,
and a maximum displayed sensor age of 200 ms. Stale frames are explicitly hidden.

Ground takeoff in LOITER, approach, GUIDED visual descent, target-wire contact,
LAND and normal disarm all completed. Final Gazebo evaluation: x=0.115405 m,
y=-3.351401 m, z=13.148827 m, speed=0.000084 m/s, contact=true, armed=false.
All owned processes exited. No ArduPilot flight-law change was made.
Ten isolated vision tests pass, including nearest gray-wire selection, rejection
of broad gray surfaces and rejection of front-rotor reflections.

Detection uses low-saturation image components, measured depth, known self-body
exclusion volumes and a horizontal 3D line fit. It is not general real-world
conductor recognition. Brief invalid detections cause zero velocity requests,
and a prolonged loss aborts to LAND. The 10 Hz camera, rigid catenary, ideal depth,
known coarse approach and inherited four-wire tower layout remain limitations.
The ground-truth height is used only for final support evaluation, not descent
guidance. Earlier runs (21:32, 21:33, 21:40 and 21:43) were interrupted diagnostic
attempts and are not successful-flight evidence.
