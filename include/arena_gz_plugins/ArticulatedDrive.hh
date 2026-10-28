#pragma once

#include <memory>

#include <gz/sim/System.hh>

namespace arena_gz_plugins
{

class ArticulatedDrivePrivate;

/// Model plugin that switches a position drive on and off per joint of an
/// articulated object and reports the joint positions. While a joint's drive
/// is on it applies a force of stiffness times the position error minus
/// damping times the velocity, limited to the effort; off, the joint moves
/// only under its SDF dynamics and contacts. Each joint starts at its initial
/// value at rest.
///
///   <plugin filename="ArticulatedDrive" name="arena_gz_plugins::ArticulatedDrive">
///     <joint name="door" stiffness="100" damping="12" effort="10" wraps="false" initial="0"/>
///   </plugin>
///
/// Commands arrive on /model/<model>/articulated/command as gz.msgs.Double_V,
/// three values per configured joint in order: reference, drive (0 or 1) and
/// place (1 sets the joint onto the reference at rest). Positions leave on
/// /model/<model>/articulated/state as gz.msgs.Double_V in the same order.
class ArticulatedDrive
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  ArticulatedDrive();
  ~ArticulatedDrive() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr<ArticulatedDrivePrivate> dataPtr;
};

}  // namespace arena_gz_plugins
