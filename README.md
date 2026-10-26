# arena_gz_plugins

gz-sim 8 (Harmonic) C++ plugins for Arena-Rosnav: a server-side **system** plugin
(`PedSkeletonPlugin`) and a **GUI** plugin (`ViewportCamera`).

## PedSkeletonPlugin

Drives Gazebo **actor** pedestrians from the `arena_peds` topic: it positions
each actor and poses its skeleton per bone from the wire joint angles, so the
in-sim body renders the same articulated gait as Isaac.

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
2. **Skeleton**: sets `components::BoneTransforms`, the local transform of every
   skin bone, from the ped's `joint_state` (see below). Templates that cannot be
   bone-driven fall back to the clip scrub: when moving, `components::AnimationName`
   is the `walk` clip and `components::AnimationTime` advances by
   `speed / kWalkRefSpeed` per second, where `kWalkRefSpeed` is walk.dae's natural
   stride speed (1.384 m over its 5.79 s clip = 0.239 m/s), keeping the feet planted
   at the ped's ground speed. Below `kIdleSpeed` it switches to the stationary `idle`
   clip, advanced in real time, so the body stands with light motion instead of a
   frozen walk frame.

`arena_peds` carries the full active ped set per message, so a name that was present
and is now absent has been **ejected**: the plugin drops its state and removes the
actor entity (`RequestRemoveEntity`), so despawn is driven entirely by topic absence
(no `set_pose`/delete from the runtime needed).

Both components flow through gz's `RenderUtil` to the GUI **and** `gpu_lidar`.

### Per-bone driving

`components::BoneTransforms` (gz-sim) carries each skeleton bone's local transform
by name and takes precedence over `AnimationName`/`AnimationTime` on the render side.
The plugin fills it the way the Isaac provider does: the `<bone_map>` JSON maps each
wire joint (`joint_state.name`) to bone targets (`bone`, bone-local `axis`, `sign`,
`scale`), the rest pose is the `idle` clip's first frame per bone (the skin
transform for the root and for bones without a channel), the neutral stance is the
first frame of an optional `neutral` clip (the rest pose without one) with the legs
(`UpLeg`, `Leg`, `Foot`, `ToeBase`) at their skin transform, thighs and shanks then
swung to hang straight down and the feet keeping their skin orientation, and each tick
the wire angles, buffered per ped and sampled 1.5 periods behind sim time, rotate
their target bones about their axes on top of the neutral stance. A blend weight
slerps from the rest pose to that wire pose and ramps to zero when the wire is older
than 0.5 s, so a ped without a fresh wire shows the rest pose.
The actor then stands on its lowest sole point: a heel and a toe point per foot, the
lowest skin vertices behind and ahead of the foot's middle, ride on their bones, and
the actor moves along the wire pose's up axis each tick until the lowest of them is level
with the pose, the floor for a standing ped. A rig on wheels
(`WheelL`, `WheelR` bones) has none and keeps the SDF ground offset. A template without a
bone map, a `.dae` idle clip or a skin skeleton keeps the clip-scrub path.

`BoneTransforms` is not in stock gz-sim 8. It comes from the gz-sim overlay that
`arena feature gazebo update` builds, which then rebuilds this package. CMake checks
for the component, and a build against stock gz-sim warns and clip-scrubs every
actor.

```xml
<plugin filename="PedSkeletonPlugin" name="arena_gz_plugins::PedSkeletonPlugin">
  <enabled>true</enabled>
  <bone_map>/abs/path/bone_map.json</bone_map>
</plugin>
```

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
an `<actor>` with a skinned `<skin>` mesh and `<animation name="walk">` /
`<animation name="idle">` clips. `idle` is the rest pose of the per-bone path
and, with `walk`, the clip the fallback passes to `AnimationName`. A bundle whose
rest pose is not the stance its wire angles assume (a seated wheelchair user) adds
`<animation name="neutral">` with that stance.

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
| `set_view` | `arena_runtime_msgs/srv/ViewportSetView` | call | one-shot: look from `eye` toward `target` (points in the current reference frame); `fov` (rad) `<= 0` keeps the current value, `clip_near` (m) 0 keeps the current value and `< 0` restores the camera's own. Snaps immediately, then releases the camera back to manual control. |
| `set_reference_frame` | `arena_runtime_msgs/srv/ViewportSetReferenceFrame` | call | sets the frame that `set_view` and `cmd_view` poses are expressed in. |
| `cmd_view` | `arena_runtime_msgs/msg/ViewportView` | subscribe | keyframe stream; plugin buffers and interpolates by time each render frame. QoS: best-effort, keep-last-64. |
| `set_projection` | `arena_runtime_msgs/srv/ViewportSetProjection` | call | `"perspective"` or `"orthographic"`. |
| `capture` | `arena_runtime_msgs/srv/ViewportCapture` | call | snap to an exact pose, render, and return the frame (`rgb8`). The deterministic-recording primitive: it composes the world pose (`reference * local`, the same composition as the live drive), renders that frame on the render thread, reads the pixels back, and returns them. Synced to the render thread via a condition variable, the call blocks (up to 15 s) until the render thread renders the pose and hands back the pixels. Request also carries `world_orientation` (true keeps the aim world-stable), `fov` (rad, `<= 0` keeps the current value), `clip_near` (m, as in `set_view`) and `min_sim_time`: a non-zero value defers the grab until the GUI's sim time reaches it (up to 15 s). Such a gated capture then pulls one full state from the server (`/world/<name>/state_async`) and hands it to the GUI before grabbing, because the scene broadcaster throttles pose updates on wall time and a stepped sim's single iteration is rarely published, which left the rendered scene several frames stale. |
| `camera_pose` | `geometry_msgs/msg/PoseStamped` | publish | live camera pose, ~10 Hz, in the `map` frame. |

**Pose composition.** Each render frame the camera world pose is `reference_frame * local_pose`. The reference frame is one of:
- World origin (default, identity).
- A constant pose (supplied via `set_reference_frame` with `has_pose: true` and `entity` empty).
- A tracked entity: every frame the plugin looks up the entity's live world pose, from the ECM by name, else from TF in `map` (`entity` non-empty).

The local pose comes from `set_view` (one-shot) or the `cmd_view` stream (continuous).

**`set_reference_frame` fields:**
- `string entity`: what to track, a TF frame id (e.g. `env_0/jackal/base_link`, the same string every backend accepts) or a gz entity name (e.g. the model `env_0/jackal`). Empty selects a constant or latched frame.
- `geometry_msgs/Pose pose`: constant reference pose, used when `entity` is empty and `has_pose` is true.
- `bool has_pose`: see above.
- `uint8 mode`: `FULL`=0 / `YAW_ONLY`=1 / `POSITION_ONLY`=2. Controls how much of a tracked entity's rotation the local offset inherits. Ignored when no entity is tracked.
- Latch semantics: `entity` empty and `has_pose` false freezes the reference at its current world pose. The camera stays put (local offset intact) without a jump. Supplying an explicit pose is the same operation with a pose you provide instead of a snapshot.

**`cmd_view` fields** (`ViewportView` message):
- `geometry_msgs/Pose pose`: camera pose in the current reference frame.
- `builtin_interfaces/Time target_time`: wall time the pose should be reached (keyframe timestamp).
- `bool world_orientation`: true keeps the camera aim world-stable, false rotates the aim with the reference frame.
- `float64 fov`: radians, `<= 0` keeps the current value.
- `float64 clip_near`: near clip distance in m, 0 keeps the current value, `< 0` restores the user camera's own.

**Keyframe jitter buffer.** The helper stamps each keyframe `now + LEAD` (0.3 s) and publishes best-effort keep-last-64. Each render frame the plugin samples the buffer at clock-now by lerp/slerp between the two bracketing keyframes, clamped at the ends. Publish rate and transport gaps up to ~300 ms are invisible because keyframes lead the clock, and there is no per-frame smoothing. Trade-off: the live view trails real time by ~LEAD, fine for scripted playback. A live stream (or tracked entity) drives the camera every frame and holds it (last pose persists if the stream stops), issue `set_view` to release back to manual control. A pending `capture` request is fulfilled on the render thread (`MaybeCapture`) like the live drive: the render frame composes and renders the requested pose, grabs the pixels, and signals the waiting service call.

### Sim portability

Interpolation runs inside the render loop of each simulator's plugin (sampling the buffer at the exact render instant, next to the camera, no extra transport). The logic is sim-agnostic: the only irreducibly sim-specific surface is a thin **CameraBackend** (set/get camera world pose, set projection, set fov, resolve a named entity's world pose). Everything else (keyframe buffer + interpolation, reference-frame composition, drive model, grab-to-release, the ROS services/topics) belongs to a sim-agnostic **ViewportController**. A new sim = implement the ~5-method backend, not the controller. Current status: `ViewportCamera` holds this logic inline as the reference implementation with the interpolation core (`SampleBuffer` + `Keyframe`) already factored out; the full ViewportController/CameraBackend extraction is the next step. Honest caveat: gz plugins are C++ and Isaac is Python, so "shared" means two direct-mirror controllers (~80 lines each), not one binary.

`entity` is resolved as a scene entity name first (sim_path, e.g. `env_0/jackal`) and as a TF frame otherwise (e.g. `env_0/jackal/base_link`). Poses are in the world frame, which Arena pins to `map`.

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
