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
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <filesystem>
#include <system_error>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <sdf/Actor.hh>
#include <sdf/Root.hh>

#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <arena_people_msgs/msg/pedestrians.hpp>

#include "arena_gz_plugins/PedSkeletonPlugin.hh"

namespace arena_gz_plugins
{

using namespace std::chrono_literals;

// Fallback ground offset, used only when no actor template is cached yet.
// Each model's real offset is read from its actor SDF's authored pose z
// (walk.dae's followTrajectory zeroes its root, so arenian authors 1.01 m
// there to plant the feet; this fallback matches that).
static constexpr double kZOffset = 1.01;
// Clip played while moving, must match <animation name="..."> in the actor SDF.
static constexpr const char * kWalkAnimName = "walk";
// Stationary stand clip played while idle, so the body stands instead of holding
// a frozen walk frame. Advances in real time so its light motion keeps playing.
static constexpr const char * kIdleAnimName = "idle";
// walk.dae advances its root 1.384 m over its 5.79 s clip, a 0.239 m/s natural
// stride, so the cursor advances speed/0.239 per second to keep the feet planted.
static constexpr double kWalkRefSpeed = 0.239;
// Below this speed the ped is idle and the clip is held still.
static constexpr double kIdleSpeed = 0.05;
// Max seconds to dead-reckon a stalled ped.
static constexpr double kMaxExtrap = 0.5;
// 2*pi, converts a gait phase in radians to a fraction of the walk clip.
static constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

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

/// Parsed actor template plus per-model metadata resolved once at load time.
struct CachedActor
{
  sdf::Actor actor;
  // Authored actor SDF pose z, the per-model ground offset (see kZOffset).
  double groundZ{0.0};
  // Walk-cycle duration in seconds, from the walk animation's BVH header. 0
  // for non-BVH clips (e.g. arenian's walk.dae), which keep legacy dead
  // reckoning instead of phase lock.
  double walkClipSeconds{0.0};
};

/// Parse a BVH MOTION header for "Frames:" and "Frame Time:" and return their
/// product, the clip duration in seconds. 0.0 if the file is missing or the
/// header cannot be parsed.
static double ParseBvhDuration(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open())
    return 0.0;

  std::string line;
  bool inMotion = false;
  long frames = -1;
  double frameTime = -1.0;
  while (std::getline(file, line))
  {
    if (!inMotion)
    {
      if (line.find("MOTION") != std::string::npos)
        inMotion = true;
      continue;
    }
    const auto framesPos = line.find("Frames:");
    if (frames < 0 && framesPos != std::string::npos)
    {
      try
      {
        frames = std::stol(line.substr(framesPos + 7));
      }
      catch (const std::exception &)
      {
        return 0.0;
      }
    }
    const auto timePos = line.find("Frame Time:");
    if (frameTime < 0 && timePos != std::string::npos)
    {
      try
      {
        frameTime = std::stod(line.substr(timePos + 11));
      }
      catch (const std::exception &)
      {
        return 0.0;
      }
    }
    if (frames >= 0 && frameTime >= 0)
      break;
  }
  return (frames >= 0 && frameTime >= 0) ? static_cast<double>(frames) * frameTime : 0.0;
}

/// Resolve the "walk" animation's clip relative to the actor SDF's directory
/// and, if it is a BVH, return its duration. 0.0 for non-BVH clips (walk.dae)
/// or a missing walk animation.
static double WalkClipDuration(const sdf::Actor & actor, const std::string & actorFile)
{
  const sdf::Animation * walkAnim = nullptr;
  for (uint64_t i = 0; i < actor.AnimationCount(); ++i)
  {
    const sdf::Animation * anim = actor.AnimationByIndex(i);
    if (anim != nullptr && anim->Name() == kWalkAnimName)
    {
      walkAnim = anim;
      break;
    }
  }
  if (walkAnim == nullptr)
    return 0.0;

  std::filesystem::path clipPath(walkAnim->Filename());
  if (!clipPath.is_absolute())
    clipPath = std::filesystem::path(actorFile).parent_path() / clipPath;
  if (clipPath.extension() != ".bvh")
    return 0.0;

  return ParseBvhDuration(clipPath.string());
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

  // Last ped-name set seen on each topic. arena_peds carries the full active set
  // per message, so a name that was present and is now absent has been ejected.
  std::unordered_map<std::string, std::unordered_set<std::string>> topicPeds;
  // Ejected ped names awaiting actor removal, drained each PreUpdate.
  std::unordered_set<std::string> pendingRemovals;

  std::unordered_map<std::string, std::chrono::steady_clock::duration> animTime;

  std::unordered_map<std::string, uint64_t> anchorSeq;
  std::unordered_map<std::string, double> anchorTime;

  // name -> resolved actor SDF path, carried once on arena_peds (model_uri).
  std::unordered_map<std::string, std::string> pedModelPath;
  // name -> the SDF path the live actor was actually spawned from. Ped names
  // repeat across episodes while their models change, so a mismatch against
  // pedModelPath means the actor carries a stale skin and must be respawned.
  std::unordered_map<std::string, std::string> actorModelPath;
  // SDF path -> parsed actor template (+ ground offset, walk clip duration),
  // cached to avoid re-parsing on the sim thread.
  std::unordered_map<std::string, CachedActor> actorTemplates;
  // SDF paths that failed to load, warned once and skipped thereafter.
  std::unordered_set<std::string> failedPaths;
  // World entity new actors parent to, and the manager used to create them.
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  gz::sim::EventManager * eventMgr{nullptr};

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
        [this, topic](arena_people_msgs::msg::Pedestrians::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(pedsMutex);
          std::unordered_set<std::string> names;
          for (const auto & ped : msg->pedestrians)
          {
            names.insert(ped.name);
            pedState[ped.name] = ped;
            ++pedSeq[ped.name];
            pendingRemovals.erase(ped.name);  // reappeared before removal ran
            if (!ped.model_uri.empty())
              pedModelPath[ped.name] = ped.model_uri;
          }
          // Names present last message but gone now are ejected: drop their
          // state and queue the actor for removal.
          auto & prev = topicPeds[topic];
          for (const auto & name : prev)
          {
            if (!names.count(name))
            {
              pedState.erase(name);
              pedSeq.erase(name);
              pedModelPath.erase(name);
              pendingRemovals.insert(name);
            }
          }
          prev = std::move(names);
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

  /// Lazily load + cache the actor template at an SDF path (file, or a model dir
  /// holding <dir>.sdf or a single *.sdf). nullptr (warned once) if none loads.
  const CachedActor * ActorTemplate(const std::string & path)
  {
    auto cached = actorTemplates.find(path);
    if (cached != actorTemplates.end())
      return &cached->second;
    if (failedPaths.count(path))
      return nullptr;

    std::error_code ec;
    std::string file = path;
    if (std::filesystem::is_directory(file, ec))
    {
      const std::filesystem::path dir(file);
      const std::filesystem::path named = dir / (dir.filename().string() + ".sdf");
      if (std::filesystem::exists(named, ec))
      {
        file = named.string();
      }
      else
      {
        file.clear();
        for (const auto & entry : std::filesystem::directory_iterator(dir, ec))
        {
          if (entry.path().extension() == ".sdf")
          {
            file = entry.path().string();
            break;
          }
        }
      }
    }

    sdf::Root root;
    if (file.empty() || !root.Load(file).empty() || root.Actor() == nullptr)
    {
      gzerr << "[PedSkeletonPlugin] no loadable actor at '" << path << "'\n";
      failedPaths.insert(path);
      return nullptr;
    }

    CachedActor entry;
    entry.actor = *root.Actor();
    entry.groundZ = entry.actor.RawPose().Pos().Z();
    entry.walkClipSeconds = WalkClipDuration(entry.actor, file);
    return &actorTemplates.emplace(path, std::move(entry)).first->second;
  }
};

PedSkeletonPlugin::PedSkeletonPlugin()
: dataPtr(std::make_unique<PedSkeletonPluginPrivate>())
{
}

PedSkeletonPlugin::~PedSkeletonPlugin() = default;

void PedSkeletonPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/,
  gz::sim::EventManager & _eventMgr)
{
  if (_sdf->HasElement("enabled"))
    dataPtr->enabled = _sdf->Get<bool>("enabled");

  if (!dataPtr->enabled)
  {
    gzmsg << "[PedSkeletonPlugin] disabled via SDF.\n";
    return;
  }

  // The plugin attaches to the world, so its entity is the actors' parent.
  dataPtr->worldEntity = _entity;
  dataPtr->eventMgr = &_eventMgr;

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
  std::unordered_map<std::string, std::string> modelPaths;
  std::unordered_set<std::string> removals;
  {
    std::lock_guard<std::mutex> lock(dataPtr->pedsMutex);
    snapshot = dataPtr->pedState;
    seqSnapshot = dataPtr->pedSeq;
    modelPaths = dataPtr->pedModelPath;
    removals.swap(dataPtr->pendingRemovals);  // take and clear, one-shot
  }
  if (snapshot.empty() && removals.empty())
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();
  const double simTimeSec = std::chrono::duration<double>(_info.simTime).count();

  std::vector<gz::sim::Entity> toRemove;
  std::unordered_set<std::string> liveActors;
  _ecm.Each<gz::sim::components::Actor, gz::sim::components::Name>(
    [&](const gz::sim::Entity & entity,
        const gz::sim::components::Actor *,
        const gz::sim::components::Name * name) -> bool
    {
      const std::string & pedName = name->Data();
      liveActors.insert(pedName);
      auto it = snapshot.find(pedName);
      if (it == snapshot.end())
      {
        if (removals.count(pedName))
          toRemove.push_back(entity);
        return true;
      }
      const auto & ped = it->second;

      const auto pathIt = modelPaths.find(pedName);
      // Ped names repeat across episodes while their models change, so an
      // actor spawned from a different SDF than the ped's current one carries
      // a stale skin: remove it and let the spawn pass recreate it next tick.
      if (pathIt != modelPaths.end() && !pathIt->second.empty())
      {
        const auto spawnedIt = dataPtr->actorModelPath.find(pedName);
        if (spawnedIt == dataPtr->actorModelPath.end() ||
            spawnedIt->second != pathIt->second)
        {
          dataPtr->actorModelPath.erase(pedName);
          toRemove.push_back(entity);
          return true;
        }
      }

      const uint64_t seq = seqSnapshot[pedName];
      if (dataPtr->anchorSeq[pedName] != seq)
      {
        dataPtr->anchorSeq[pedName] = seq;
        dataPtr->anchorTime[pedName] = simTimeSec;
      }
      const double elapsed =
        std::clamp(simTimeSec - dataPtr->anchorTime[pedName], 0.0, kMaxExtrap);

      const CachedActor * tmpl = (pathIt != modelPaths.end())
        ? dataPtr->ActorTemplate(pathIt->second) : nullptr;
      const double groundZ = (tmpl != nullptr) ? tmpl->groundZ : kZOffset;

      gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x + ped.twist.linear.x * elapsed,
          ped.pose.position.y + ped.twist.linear.y * elapsed,
          ped.pose.position.z + groundZ),
        gz::math::Quaterniond(
          ped.pose.orientation.w, ped.pose.orientation.x,
          ped.pose.orientation.y, ped.pose.orientation.z));
      SetComponent<gz::sim::components::Pose>(_ecm, entity, gz::math::Pose3d::Zero);
      SetComponent<gz::sim::components::TrajectoryPose>(_ecm, entity, worldPose);

      // Idle peds stand (real-time stand clip); moving peds walk with the clip
      // phased to ground speed so the feet stay planted.
      const double speed = std::hypot(ped.twist.linear.x, ped.twist.linear.y);
      auto & cursor = dataPtr->animTime[pedName];
      if (speed > kIdleSpeed)
      {
        SetComponent<gz::sim::components::AnimationName>(
          _ecm, entity, std::string(kWalkAnimName));
        if (ped.gait_phase > 0.0f && tmpl != nullptr && tmpl->walkClipSeconds > 0.0)
        {
          // Absolute phase lock to the ROS-side gait: this formula must match
          // GaitGenerator's cadence in gait.py (_gait_walk/_gait_run).
          const double phaseRate = kTwoPi * std::clamp(0.4 + 0.55 * speed, 0.4, 2.2);
          const double phaseSeconds =
            (ped.gait_phase + phaseRate * elapsed) / kTwoPi * tmpl->walkClipSeconds;
          cursor = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(phaseSeconds));
        }
        else
        {
          const double advance = dt * (speed / kWalkRefSpeed);
          cursor += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(advance));
        }
      }
      else
      {
        SetComponent<gz::sim::components::AnimationName>(
          _ecm, entity, std::string(kIdleAnimName));
        cursor += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(dt));
      }
      SetComponent<gz::sim::components::AnimationTime>(_ecm, entity, cursor);
      return true;
    });

  // Remove ejected pedestrians' actors.
  for (const auto & entity : toRemove)
    _ecm.RequestRemoveEntity(entity);

  // Spawn an actor for any ped with no live actor yet (idempotent on the ECM).
  if (dataPtr->eventMgr != nullptr)
  {
    gz::sim::SdfEntityCreator creator(_ecm, *dataPtr->eventMgr);
    for (const auto & [pedName, ped] : snapshot)
    {
      if (liveActors.count(pedName))
        continue;
      auto pathIt = modelPaths.find(pedName);
      if (pathIt == modelPaths.end() || pathIt->second.empty())
        continue;
      const CachedActor * tmpl = dataPtr->ActorTemplate(pathIt->second);
      if (tmpl == nullptr)
        continue;

      sdf::Actor actor = tmpl->actor;
      actor.SetName(pedName);
      const gz::sim::Entity entity = creator.CreateEntities(&actor);
      creator.SetParent(entity, dataPtr->worldEntity);
      dataPtr->actorModelPath[pedName] = pathIt->second;

      // Place immediately so the actor does not flash at the origin before the
      // drive pass picks it up next tick.
      const gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x,
          ped.pose.position.y,
          ped.pose.position.z + tmpl->groundZ),
        gz::math::Quaterniond(
          ped.pose.orientation.w, ped.pose.orientation.x,
          ped.pose.orientation.y, ped.pose.orientation.z));
      SetComponent<gz::sim::components::Pose>(_ecm, entity, gz::math::Pose3d::Zero);
      SetComponent<gz::sim::components::TrajectoryPose>(_ecm, entity, worldPose);
      gzmsg << "[PedSkeletonPlugin] spawned actor '" << pedName << "'\n";
    }
  }

  // Drop per-ped state for peds that despawned (pruned from the snapshot).
  const auto dropStale = [&](auto & map)
  {
    for (auto it = map.begin(); it != map.end(); )
      it = (snapshot.count(it->first) == 0) ? map.erase(it) : std::next(it);
  };
  dropStale(dataPtr->animTime);
  dropStale(dataPtr->anchorSeq);
  dropStale(dataPtr->anchorTime);
  dropStale(dataPtr->actorModelPath);
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
