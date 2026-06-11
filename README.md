# arena_gz_plugins

gz-sim 8 (Harmonic) C++ system plugins for Arena-Rosnav.

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
