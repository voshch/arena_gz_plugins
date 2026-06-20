# arena_gz_plugins

gz-sim 8 (Harmonic) C++ plugins for Arena-Rosnav: a server-side **system** plugin
(`PedSkeletonPlugin`) and a **GUI** plugin (`ViewportCamera`).

## PedSkeletonPlugin

Drives Gazebo **actor** pedestrians from the `arena_peds` topic: it positions
each actor and plays the `walk.dae` clip phased to the ped's speed, so the
in-sim body walks in step with the canonical gait.

### What it does

`Configure` starts an `rclcpp` node on a dedicated spin thread that discovers and
subscribes to every `*/arena_peds` topic (any number of env namespaces, no SDF
config). Each `PreUpdate` (sim thread) snapshots the latest pedestrian state and,
for every actor whose `Name` matches a `Pedestrian.name`:

1. **Position** — sets `components::TrajectoryPose` to the ped's world pose and
   zeroes `components::Pose`. gz renders an actor at `Pose * TrajectoryPose`, so
   zeroing `Pose` makes `TrajectoryPose` the absolute pose. Between the ~10 Hz
   `arena_peds` samples the position is dead-reckoned by the ped's twist (re-anchored
   on each fresh sample, clamped to `kMaxExtrap`) so the body slides smoothly instead
   of stepping.
2. **Animation** — when moving, sets `components::AnimationName` to the `walk` clip
   and advances `components::AnimationTime` by `speed / kWalkRefSpeed` per second,
   where `kWalkRefSpeed` is walk.dae's natural stride speed (1.384 m over its 5.79 s
   clip = 0.239 m/s), keeping the feet planted at the ped's ground speed. Below
   `kIdleSpeed` it switches to the stationary `idle` (stand.dae) clip, advanced in
   real time, so the body stands with light motion instead of a frozen walk frame.

`arena_peds` carries the full active ped set per message, so a name that was present
and is now absent has been **ejected**: the plugin drops its state and removes the
actor entity (`RequestRemoveEntity`), so despawn is driven entirely by topic absence
(no `set_pose`/delete from the runtime needed).

Both components flow through gz's `RenderUtil` to the GUI **and** `gpu_lidar`.

### Why clip-scrub and not per-bone

gz-sim 8 has no supported path for external per-bone skeleton control on actors:
`RenderUtil::UpdateAnimation` re-enables skeleton animation every frame and clears
any manual-bone flags, and the only real per-bone path (`actorManualSkeletonUpdate`)
has no public setter. So the in-sim mesh plays the canned walk clip. The true
articulated gait (`GaitGenerator` joint angles) remains the ROS4HRI/rviz and Isaac
view; Gazebo's body is a clip-scrub approximation of it.

### gpu_lidar rationale

Arena uses `gpu_lidar`, a rendering sensor that forces a server-side render scene
even headless. The actor (positioned and animated by this plugin) is therefore in
the scene during the lidar sweep, so the robot perceives a moving, articulated body
rather than a static capsule. That makes the in-sim animation perceptually
load-bearing in headless RL training, not just cosmetic.

### Launch toggle

```
ped_skeleton_enabled:=true   # default
ped_skeleton_enabled:=false  # skip plugin injection
```

When enabled, `arena_bringup/launch/simulator/sim/gazebo/gazebo.launch.py` injects a
`<plugin filename="PedSkeletonPlugin" .../>` element into the world SDF and points
`GZ_SIM_SYSTEM_PLUGIN_PATH` at the built library.

### Actor asset

The pedestrian is `arena_simulation_setup/assets/Common/Pedestrian/arenian/arenian.sdf`:
an `<actor>` with the walk.dae `<skin>` and `<animation name="walk">` /
`<animation name="idle">` (stand.dae) clips. The `walk` and `idle` names are the
contract the plugin passes to `AnimationName`.

## ViewportCamera

Exposes the GUI **viewport user-camera** (the interactive 3D view) over ROS 2, so
external scripts can frame and record scenes without touching gz. It is a
`gz::sim::GuiSystem`: it runs in the GUI process, drives the user-camera on the
render thread, and samples reference-frame entity poses from the ECM every frame.

### ROS interface

All under the global `/arena/viewport` namespace (one viewport per runtime,
regardless of how many envs are attached):

| Endpoint | Type | Direction | Effect |
| --- | --- | --- | --- |
| `set_view` | `arena_runtime_msgs/srv/ViewportSetView` | call | one-shot: look from `eye` toward `target` (points in the current reference frame); `fov` (rad) `<= 0` keeps the current value. Snaps immediately, then releases the camera back to manual control. |
| `set_reference_frame` | `arena_runtime_msgs/srv/ViewportSetReferenceFrame` | call | sets the frame that `set_view` and `cmd_view` poses are expressed in. |
| `cmd_view` | `arena_runtime_msgs/msg/ViewportView` | subscribe | keyframe stream; plugin buffers and interpolates by time each render frame. QoS: best-effort, keep-last-64. |
| `set_projection` | `arena_runtime_msgs/srv/ViewportSetProjection` | call | `"perspective"` or `"orthographic"`. |
| `capture` | `arena_runtime_msgs/srv/ViewportCapture` | call | snap to an exact pose, render, and return the frame (`rgb8`). The deterministic-recording primitive: it composes the world pose (`reference * local`, the same composition as the live drive), renders that frame on the render thread, reads the pixels back, and returns them. Synced to the render thread via a condition variable, the call blocks (up to 5 s) until the render thread renders the pose and hands back the pixels. Request also carries `world_orientation` (true keeps the aim world-stable) and `fov` (rad, `<= 0` keeps the current value). |
| `camera_pose` | `geometry_msgs/msg/PoseStamped` | publish | live camera pose, ~10 Hz, in the `map` frame. |

**Pose composition.** Each render frame the camera world pose is `reference_frame * local_pose`. The reference frame is one of:
- World origin (default, identity).
- A constant pose (supplied via `set_reference_frame` with `has_pose: true` and `entity` empty).
- A tracked scene entity: the plugin looks up the entity's live world pose from the ECM every frame (`entity` non-empty).

The local pose comes from `set_view` (one-shot) or the `cmd_view` stream (continuous).

**`set_reference_frame` fields:**
- `string entity`: sim_path to track (e.g. `env_0/jackal`). Empty selects a constant or latched frame.
- `geometry_msgs/Pose pose`: constant reference pose, used when `entity` is empty and `has_pose` is true.
- `bool has_pose`: see above.
- `uint8 mode`: `FULL`=0 / `YAW_ONLY`=1 / `POSITION_ONLY`=2. Controls how much of a tracked entity's rotation the local offset inherits. Ignored when no entity is tracked.
- Latch semantics: `entity` empty and `has_pose` false freezes the reference at its current world pose. The camera stays put (local offset intact) without a jump. Supplying an explicit pose is the same operation with a pose you provide instead of a snapshot.

**`cmd_view` fields** (`ViewportView` message):
- `geometry_msgs/Pose pose`: camera pose in the current reference frame.
- `builtin_interfaces/Time target_time`: wall time the pose should be reached (keyframe timestamp).
- `bool world_orientation`: true keeps the camera aim world-stable, false rotates the aim with the reference frame.
- `float64 fov`: radians, `<= 0` keeps the current value.

**Keyframe jitter buffer.** The helper stamps each keyframe `now + LEAD` (0.3 s) and publishes best-effort keep-last-64. Each render frame the plugin samples the buffer at clock-now by lerp/slerp between the two bracketing keyframes, clamped at the ends. Publish rate and transport gaps up to ~300 ms are invisible because keyframes lead the clock, and there is no per-frame smoothing. Trade-off: the live view trails real time by ~LEAD, fine for scripted playback. A live stream (or tracked entity) drives the camera every frame and holds it (last pose persists if the stream stops), issue `set_view` to release back to manual control. A pending `capture` request is fulfilled on the render thread (`MaybeCapture`) like the live drive: the render frame composes and renders the requested pose, grabs the pixels, and signals the waiting service call.

### Sim portability

Interpolation runs inside the render loop of each simulator's plugin (sampling the buffer at the exact render instant, next to the camera, no extra transport). The logic is sim-agnostic: the only irreducibly sim-specific surface is a thin **CameraBackend** (set/get camera world pose, set projection, set fov, resolve a named entity's world pose). Everything else (keyframe buffer + interpolation, reference-frame composition, drive model, grab-to-release, the ROS services/topics) belongs to a sim-agnostic **ViewportController**. A new sim = implement the ~5-method backend, not the controller. Current status: `ViewportCamera` holds this logic inline as the reference implementation with the interpolation core (`SampleBuffer` + `Keyframe`) already factored out; the full ViewportController/CameraBackend extraction is the next step. Honest caveat: gz plugins are C++ and Isaac is Python, so "shared" means two direct-mirror controllers (~80 lines each), not one binary.

`entity` is the scoped model name (sim_path, e.g. `env_0/jackal`). Poses are in the world frame, which Arena pins to `map`.

The same surface is mirrored in-process by `ViewportITF` on `BaseSim`
(`viewport_set_view` / `viewport_set_reference_frame` / `viewport_stream_view` /
`viewport_set_projection` / `viewport_camera_pose`); the Gazebo adapter implements it
by calling the services and publishing to `cmd_view`.

### Examples

```bash
# point the camera at the origin from above (one-shot, snaps)
ros2 service call /arena/viewport/set_view arena_runtime_msgs/srv/ViewportSetView \
  "{eye: {x: 0, y: 0, z: 12}, target: {x: 0, y: 0, z: 0}, fov: 0.0}"

# attach the reference frame to env_0/jackal (camera now follows the robot)
ros2 service call /arena/viewport/set_reference_frame \
  arena_runtime_msgs/srv/ViewportSetReferenceFrame \
  "{entity: 'env_0/jackal', has_pose: false, mode: 0}"

# stream a smooth offset pose (publish continuously for orbits / flybys)
ros2 topic pub --once /arena/viewport/cmd_view arena_runtime_msgs/msg/ViewportView \
  "{pose: {position: {x: -4, y: 0, z: 3}, orientation: {w: 1}}, world_orientation: false, fov: 0.0}"
```

### Launch

`gazebo.launch.py` derives a gui.config at launch from gz's own default
(`~/.gz/sim/8/gui.config`): it appends this plugin, pins the render engine, and passes
the result via `--gui-config`, with `GZ_GUI_PLUGIN_PATH` pointed at the built library.
`--gui-config` replaces the whole plugin list (gz does not merge), so starting from the
default is what keeps the rest of the GUI intact. gz only writes that default on its
first GUI run, so on a pristine container the first launch loads gz's built-in default
without this plugin and it loads from the next launch on. Only active when the GUI runs,
so `headless:=true` (server-only `-s`) skips it and the services are absent.
