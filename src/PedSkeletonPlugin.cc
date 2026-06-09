/// PedSkeletonPlugin drives Gazebo actor pedestrians from arena_peds. Each tick,
/// in PreUpdate (sim thread), it sets every matching actor's world pose via
/// components::TrajectoryPose and scrubs the walk.dae clip via
/// components::AnimationTime. gz-sim 8 exposes no supported path for arbitrary
/// per-bone skeleton control on actors, so the in-sim mesh plays the walk clip
/// phased by ped speed; the true 15-joint gait stays the ROS4HRI/rviz view.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <arena_people_msgs/msg/pedestrians.hpp>

#include "arena_gz_plugins/PedSkeletonPlugin.hh"

namespace arena_gz_plugins
{

using namespace std::chrono_literals;

// walk.dae zeroes its root under followTrajectory, dropping the ~1.01 m hips onto
// the actor origin, so lift the origin to plant the feet at the ped ground z.
static constexpr double kZOffset = 1.01;
// Animation clip name, must match <animation name="..."> in the actor SDF.
static constexpr const char * kAnimName = "walk";
// walk.dae advances its root 1.384 m over its 5.79 s clip, a 0.239 m/s natural
// stride, so the cursor advances speed/0.239 per second to keep the feet planted.
static constexpr double kWalkRefSpeed = 0.239;
// Below this speed the ped is idle and the clip is held still.
static constexpr double kIdleSpeed = 0.05;
// Max seconds to dead-reckon a stalled ped.
static constexpr double kMaxExtrap = 0.5;

/// Create-or-update a component and flag it for SceneBroadcaster propagation.
template <typename ComponentT, typename ValueT>
static void SetComponent(
  gz::sim::EntityComponentManager & _ecm,
  const gz::sim::Entity & _entity,
  const ValueT & _value)
{
  auto comp = _ecm.Component<ComponentT>(_entity);
  if (!comp)
    _ecm.CreateComponent(_entity, ComponentT(_value));
  else
    comp->Data() = _value;
  _ecm.SetChanged(
    _entity, ComponentT::typeId, gz::sim::ComponentState::OneTimeChange);
}

class PedSkeletonPluginPrivate
{
public:
  rclcpp::Node::SharedPtr rosNode;
  std::unordered_map<
    std::string,
    rclcpp::Subscription<arena_people_msgs::msg::Pedestrians>::SharedPtr
  > subscriptions;

  std::mutex pedsMutex;
  std::unordered_map<std::string, arena_people_msgs::msg::Pedestrian> pedState;
  std::unordered_map<std::string, uint64_t> pedSeq;

  std::unordered_map<std::string, std::chrono::steady_clock::duration> animTime;

  std::unordered_map<std::string, uint64_t> anchorSeq;
  std::unordered_map<std::string, double> anchorTime;

  // ROS spin thread, keeps rclcpp work off the sim thread.
  std::thread spinThread;
  std::atomic<bool> stopSpin{false};
  bool enabled{true};

  ~PedSkeletonPluginPrivate()
  {
    stopSpin = true;
    if (spinThread.joinable())
      spinThread.join();
  }

  /// ROS executor loop: owns subscription creation and periodic discovery.
  void SpinLoop()
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(rosNode);
    int discoveryClock = 40;  // trigger discovery on the first iteration
    while (rclcpp::ok() && !stopSpin)
    {
      exec.spin_some(50ms);
      if (++discoveryClock >= 40)  // ~2 s at the 50 ms cadence
      {
        discoveryClock = 0;
        DiscoverTopics();
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  void SubscribeTopic(const std::string & topic)
  {
    if (subscriptions.count(topic))
      return;
    gzmsg << "[PedSkeletonPlugin] subscribing to " << topic << "\n";
    subscriptions[topic] =
      rosNode->create_subscription<arena_people_msgs::msg::Pedestrians>(
        topic, rclcpp::QoS(10),
        [this](arena_people_msgs::msg::Pedestrians::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(pedsMutex);
          for (const auto & ped : msg->pedestrians)
          {
            pedState[ped.name] = ped;
            ++pedSeq[ped.name];
          }
        });
  }

  /// Scan all advertised topics ending in "/arena_peds" and subscribe.
  void DiscoverTopics()
  {
    if (!rosNode)
      return;
    static const std::string suffix = "/arena_peds";
    for (const auto & [topic, types] : rosNode->get_topic_names_and_types())
    {
      if (topic.size() >= suffix.size() &&
          topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0)
        SubscribeTopic(topic);
    }
  }
};

PedSkeletonPlugin::PedSkeletonPlugin()
: dataPtr(std::make_unique<PedSkeletonPluginPrivate>())
{
}

PedSkeletonPlugin::~PedSkeletonPlugin() = default;

void PedSkeletonPlugin::Configure(
  const gz::sim::Entity & /*_entity*/,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/,
  gz::sim::EventManager & /*_eventMgr*/)
{
  if (_sdf->HasElement("enabled"))
    dataPtr->enabled = _sdf->Get<bool>("enabled");

  if (!dataPtr->enabled)
  {
    gzmsg << "[PedSkeletonPlugin] disabled via SDF.\n";
    return;
  }

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  dataPtr->rosNode = std::make_shared<rclcpp::Node>("ped_actor_plugin");
  dataPtr->spinThread =
    std::thread(&PedSkeletonPluginPrivate::SpinLoop, dataPtr.get());

  gzmsg << "[PedSkeletonPlugin] configured, driving actor pose + walk clip.\n";
}

void PedSkeletonPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!dataPtr->enabled || _info.paused)
    return;

  std::unordered_map<std::string, arena_people_msgs::msg::Pedestrian> snapshot;
  std::unordered_map<std::string, uint64_t> seqSnapshot;
  {
    std::lock_guard<std::mutex> lock(dataPtr->pedsMutex);
    snapshot = dataPtr->pedState;
    seqSnapshot = dataPtr->pedSeq;
  }
  if (snapshot.empty())
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();
  const double simTimeSec = std::chrono::duration<double>(_info.simTime).count();

  _ecm.Each<gz::sim::components::Actor, gz::sim::components::Name>(
    [&](const gz::sim::Entity & entity,
        const gz::sim::components::Actor *,
        const gz::sim::components::Name * name) -> bool
    {
      auto it = snapshot.find(name->Data());
      if (it == snapshot.end())
        return true;
      const auto & ped = it->second;
      const std::string & pedName = name->Data();

      const uint64_t seq = seqSnapshot[pedName];
      if (dataPtr->anchorSeq[pedName] != seq)
      {
        dataPtr->anchorSeq[pedName] = seq;
        dataPtr->anchorTime[pedName] = simTimeSec;
      }
      const double elapsed =
        std::clamp(simTimeSec - dataPtr->anchorTime[pedName], 0.0, kMaxExtrap);

      gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x + ped.twist.linear.x * elapsed,
          ped.pose.position.y + ped.twist.linear.y * elapsed,
          ped.pose.position.z + kZOffset),
        gz::math::Quaterniond(
          ped.pose.orientation.w, ped.pose.orientation.x,
          ped.pose.orientation.y, ped.pose.orientation.z));
      SetComponent<gz::sim::components::Pose>(_ecm, entity, gz::math::Pose3d::Zero);
      SetComponent<gz::sim::components::TrajectoryPose>(_ecm, entity, worldPose);

      SetComponent<gz::sim::components::AnimationName>(
        _ecm, entity, std::string(kAnimName));

      const double speed = std::hypot(ped.twist.linear.x, ped.twist.linear.y);
      auto & cursor = dataPtr->animTime[pedName];
      if (speed > kIdleSpeed)
      {
        const double advance = dt * (speed / kWalkRefSpeed);
        cursor += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(advance));
      }
      SetComponent<gz::sim::components::AnimationTime>(_ecm, entity, cursor);
      return true;
    });

  // Drop cursors for peds that despawned.
  for (auto it = dataPtr->animTime.begin(); it != dataPtr->animTime.end(); )
  {
    if (snapshot.count(it->first) == 0)
      it = dataPtr->animTime.erase(it);
    else
      ++it;
  }
}

}  // namespace arena_gz_plugins

GZ_ADD_PLUGIN(
  arena_gz_plugins::PedSkeletonPlugin,
  gz::sim::System,
  arena_gz_plugins::PedSkeletonPlugin::ISystemConfigure,
  arena_gz_plugins::PedSkeletonPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  arena_gz_plugins::PedSkeletonPlugin,
  "arena_gz_plugins::PedSkeletonPlugin")
