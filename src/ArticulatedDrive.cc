#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/double_v.pb.h>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/transport/Node.hh>

#include "arena_gz_plugins/ArticulatedDrive.hh"

namespace arena_gz_plugins
{

namespace
{
constexpr std::chrono::duration<double> kStatePeriod{1.0 / 60.0};

struct DriveJoint
{
  std::string name;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  double stiffness{0.0};
  double damping{0.0};
  double effort{0.0};
  bool wraps{false};
  double initial{0.0};
  double reference{0.0};
  bool drive{false};
};

double Error(const DriveJoint & _joint, double _position)
{
  const double error = _joint.reference - _position;
  return _joint.wraps ? std::remainder(error, 2.0 * M_PI) : error;
}
}  // namespace

class ArticulatedDrivePrivate
{
public:
  void OnCommand(const gz::msgs::Double_V & _msg);

  gz::transport::Node node;
  gz::transport::Node::Publisher statePub;
  std::vector<DriveJoint> joints;
  std::mutex mutex;
  std::optional<gz::msgs::Double_V> pending;
  std::optional<std::chrono::steady_clock::duration> lastState;
  bool placed{false};
  bool initialized{false};
};

void ArticulatedDrivePrivate::OnCommand(const gz::msgs::Double_V & _msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->pending = _msg;
}

ArticulatedDrive::ArticulatedDrive()
: dataPtr(std::make_unique<ArticulatedDrivePrivate>())
{
}

ArticulatedDrive::~ArticulatedDrive() = default;

void ArticulatedDrive::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  const gz::sim::Model model(_entity);
  for (auto element = _sdf->FindElement("joint"); element; element = element->GetNextElement("joint")) {
    DriveJoint joint;
    joint.name = element->Get<std::string>("name");
    joint.entity = model.JointByName(_ecm, joint.name);
    joint.stiffness = element->Get<double>("stiffness");
    joint.damping = element->Get<double>("damping");
    joint.effort = element->Get<double>("effort");
    joint.wraps = element->Get<bool>("wraps");
    joint.initial = element->Get<double>("initial");
    joint.reference = joint.initial;
    if (joint.entity == gz::sim::kNullEntity) {
      gzerr << "ArticulatedDrive: model " << model.Name(_ecm) << " has no joint " << joint.name << std::endl;
      continue;
    }
    _ecm.CreateComponent(joint.entity, gz::sim::components::JointPosition());
    _ecm.CreateComponent(joint.entity, gz::sim::components::JointVelocity());
    this->dataPtr->joints.push_back(joint);
  }
  const std::string prefix = "/model/" + model.Name(_ecm) + "/articulated/";
  this->dataPtr->node.Subscribe(prefix + "command", &ArticulatedDrivePrivate::OnCommand, this->dataPtr.get());
  this->dataPtr->statePub = this->dataPtr->node.Advertise<gz::msgs::Double_V>(prefix + "state");
}

void ArticulatedDrive::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager & _ecm)
{
  std::optional<gz::msgs::Double_V> command;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    command.swap(this->dataPtr->pending);
  }
  auto & joints = this->dataPtr->joints;
  if (!this->dataPtr->initialized) {
    for (const auto & joint : joints) {
      _ecm.SetComponentData<gz::sim::components::JointPositionReset>(joint.entity, {joint.initial});
      _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint.entity, {0.0});
    }
    this->dataPtr->initialized = true;
    this->dataPtr->placed = true;
  }
  if (command && command->data_size() == static_cast<int>(3 * joints.size())) {
    for (std::size_t i = 0; i < joints.size(); ++i) {
      auto & joint = joints[i];
      joint.reference = command->data(3 * i);
      joint.drive = command->data(3 * i + 1) > 0.5;
      if (command->data(3 * i + 2) > 0.5) {
        _ecm.SetComponentData<gz::sim::components::JointPositionReset>(joint.entity, {joint.reference});
        _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint.entity, {0.0});
        this->dataPtr->placed = true;
      }
    }
  }
  for (const auto & joint : joints) {
    double force = 0.0;
    if (joint.drive) {
      const auto position = _ecm.Component<gz::sim::components::JointPosition>(joint.entity);
      const auto velocity = _ecm.Component<gz::sim::components::JointVelocity>(joint.entity);
      if (position && velocity && !position->Data().empty() && !velocity->Data().empty()) {
        force = joint.stiffness * Error(joint, position->Data()[0]) - joint.damping * velocity->Data()[0];
        force = std::clamp(force, -joint.effort, joint.effort);
      }
    }
    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint.entity, {force});
  }
}

void ArticulatedDrive::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  auto & data = *this->dataPtr;
  if (!data.placed && data.lastState && _info.simTime >= *data.lastState && _info.simTime - *data.lastState < std::chrono::duration_cast<std::chrono::steady_clock::duration>(kStatePeriod)) {
    return;
  }
  data.placed = false;
  data.lastState = _info.simTime;
  gz::msgs::Double_V state;
  for (const auto & joint : data.joints) {
    const auto position = _ecm.Component<gz::sim::components::JointPosition>(joint.entity);
    state.add_data(position && !position->Data().empty() ? position->Data()[0] : 0.0);
  }
  data.statePub.Publish(state);
}

}  // namespace arena_gz_plugins

GZ_ADD_PLUGIN(
  arena_gz_plugins::ArticulatedDrive,
  gz::sim::System,
  arena_gz_plugins::ArticulatedDrive::ISystemConfigure,
  arena_gz_plugins::ArticulatedDrive::ISystemPreUpdate,
  arena_gz_plugins::ArticulatedDrive::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  arena_gz_plugins::ArticulatedDrive,
  "arena_gz_plugins::ArticulatedDrive")
