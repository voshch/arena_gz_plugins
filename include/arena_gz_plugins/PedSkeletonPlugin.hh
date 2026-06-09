#pragma once

#include <memory>
#include <string>

#include <gz/sim/System.hh>

namespace arena_gz_plugins
{

class PedSkeletonPluginPrivate;

/// System plugin that drives each Gazebo actor pedestrian from the arena_peds
/// ROS 2 topic. In PreUpdate (sim thread) it sets every matching actor's world
/// pose via components::TrajectoryPose and scrubs the walk.dae clip via
/// components::AnimationTime. The motion reaches both the GUI and gpu_lidar
/// through the shared RenderUtil path.
///
/// SDF element (inside <world> or as a server plugin):
///   <plugin filename="PedSkeletonPlugin"
///           name="arena_gz_plugins::PedSkeletonPlugin">
///     <!-- Set to false to disable actor driving entirely -->
///     <enabled>true</enabled>
///   </plugin>
///
/// Topic discovery is automatic: the plugin scans all advertised ROS 2 topics
/// for names ending in "/arena_peds" and subscribes to each. No namespace
/// configuration needed for multi-env launches.
class PedSkeletonPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  PedSkeletonPlugin();
  ~PedSkeletonPlugin() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr<PedSkeletonPluginPrivate> dataPtr;
};

}  // namespace arena_gz_plugins
