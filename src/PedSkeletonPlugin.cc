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
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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

#include <sdf/Element.hh>
#include <sdf/Actor.hh>
#include <sdf/Root.hh>

#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <arena_people_msgs/msg/pedestrians.hpp>

#include "arena_gz_plugins/PedSkeletonPlugin.hh"

namespace arena_gz_plugins
{

using namespace std::chrono_literals;

// Fallback lift when no actor template is known yet (per-model lift otherwise
// comes from actorZOffsets, read off the template's <pose> z).
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
  // name -> model path the live actor was spawned with, so a later model change
  // on the same ped name forces a despawn/respawn instead of being ignored.
  std::unordered_map<std::string, std::string> spawnedModelPath;
  // SDF path -> parsed actor template, cached to avoid re-parsing on the sim thread.
  std::unordered_map<std::string, sdf::Actor> actorTemplates;
  // SDF path -> actor origin lift (the template's <pose> z), ground-to-origin
  // distance for this model. arenian needs 1.01, feet-at-origin bundles need 0.
  std::unordered_map<std::string, double> actorZOffsets;
  // SDF path -> {SDF clip name -> the name gz assigns the Ogre skeleton animation}.
  // gz names DAE clips by their SDF <animation name>, but BVH clips by their
  // resolved file path, so AnimationName must carry the latter for BVH actors.
  std::unordered_map<std::string, std::unordered_map<std::string, std::string>>
    actorAnimNames;
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
  const sdf::Actor * ActorTemplate(const std::string & path)
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

    // Absolutize skin/clip paths against the SDF's directory.
    const std::filesystem::path baseDir = std::filesystem::path(file).parent_path();
    const auto absolutize = [&baseDir](const std::string & uri)
    {
      if (uri.empty() || uri.find("://") != std::string::npos ||
          std::filesystem::path(uri).is_absolute())
        return uri;
      return (baseDir / uri).lexically_normal().string();
    };
    const sdf::ElementPtr elem = root.Actor()->Element();
    if (elem->HasElement("skin"))
    {
      const sdf::ElementPtr skin = elem->GetElement("skin");
      skin->GetElement("filename")->Set(absolutize(skin->Get<std::string>("filename")));
    }
    sdf::ElementPtr animation =
      elem->HasElement("animation") ? elem->GetElement("animation") : nullptr;
    for (; animation; animation = animation->GetNextElement("animation"))
      animation->GetElement("filename")->Set(absolutize(animation->Get<std::string>("filename")));

    sdf::Actor actor;
    actor.Load(elem);

    auto & nameMap = actorAnimNames[path];
    for (uint64_t i = 0; i < actor.AnimationCount(); ++i)
    {
      const sdf::Animation * anim = actor.AnimationByIndex(i);
      std::string ext = std::filesystem::path(anim->Filename()).extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      nameMap[anim->Name()] = (ext == ".bvh") ? anim->Filename() : anim->Name();
    }

    actorZOffsets[path] = actor.RawPose().Pos().Z();
    return &actorTemplates.emplace(path, std::move(actor)).first->second;
  }

  /// Map a logical clip name (kWalkAnimName/kIdleAnimName) to the name gz gives
  /// its Ogre skeleton animation for this model, falling back to the logical name
  /// for DAE actors (e.g. arenian) whose names already match.
  std::string ResolveAnimName(
    const std::string & modelPath, const std::string & logical)
  {
    auto it = actorAnimNames.find(modelPath);
    if (it != actorAnimNames.end())
    {
      auto jt = it->second.find(logical);
      if (jt != it->second.end())
        return jt->second;
    }
    return logical;
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

      // A ped whose model changed under the same name: tear the stale actor down
      // so the spawn pass recreates it with the new model next tick.
      const auto mpIt = modelPaths.find(pedName);
      const auto smpIt = dataPtr->spawnedModelPath.find(pedName);
      if (mpIt != modelPaths.end() && !mpIt->second.empty() &&
          smpIt != dataPtr->spawnedModelPath.end() && smpIt->second != mpIt->second)
      {
        gzmsg << "[PedSkeletonPlugin] model changed for '" << pedName
              << "', respawning\n";
        toRemove.push_back(entity);
        dataPtr->spawnedModelPath.erase(smpIt);
        return true;
      }

      const uint64_t seq = seqSnapshot[pedName];
      if (dataPtr->anchorSeq[pedName] != seq)
      {
        dataPtr->anchorSeq[pedName] = seq;
        dataPtr->anchorTime[pedName] = simTimeSec;
      }
      const double elapsed =
        std::clamp(simTimeSec - dataPtr->anchorTime[pedName], 0.0, kMaxExtrap);

      double lift = kZOffset;
      if (mpIt != modelPaths.end() && !mpIt->second.empty())
      {
        const auto zIt = dataPtr->actorZOffsets.find(mpIt->second);
        if (zIt != dataPtr->actorZOffsets.end())
          lift = zIt->second;
      }

      gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x + ped.twist.linear.x * elapsed,
          ped.pose.position.y + ped.twist.linear.y * elapsed,
          ped.pose.position.z + lift),
        gz::math::Quaterniond(
          ped.pose.orientation.w, ped.pose.orientation.x,
          ped.pose.orientation.y, ped.pose.orientation.z));
      SetComponent<gz::sim::components::Pose>(_ecm, entity, gz::math::Pose3d::Zero);
      SetComponent<gz::sim::components::TrajectoryPose>(_ecm, entity, worldPose);

      // Idle peds stand (real-time stand clip); moving peds walk with the clip
      // phased to ground speed so the feet stay planted.
      const double speed = std::hypot(ped.twist.linear.x, ped.twist.linear.y);
      auto & cursor = dataPtr->animTime[pedName];
      const std::string modelPath =
        (mpIt != modelPaths.end()) ? mpIt->second : std::string();
      double advance;
      if (speed > kIdleSpeed)
      {
        SetComponent<gz::sim::components::AnimationName>(
          _ecm, entity, dataPtr->ResolveAnimName(modelPath, kWalkAnimName));
        advance = dt * (speed / kWalkRefSpeed);
      }
      else
      {
        SetComponent<gz::sim::components::AnimationName>(
          _ecm, entity, dataPtr->ResolveAnimName(modelPath, kIdleAnimName));
        advance = dt;
      }
      cursor += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(advance));
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
      const sdf::Actor * tmpl = dataPtr->ActorTemplate(pathIt->second);
      if (tmpl == nullptr)
        continue;

      sdf::Actor actor = *tmpl;
      actor.SetName(pedName);
      const gz::sim::Entity entity = creator.CreateEntities(&actor);
      creator.SetParent(entity, dataPtr->worldEntity);
      dataPtr->spawnedModelPath[pedName] = pathIt->second;

      // Place immediately so the actor does not flash at the origin before the
      // drive pass picks it up next tick.
      const gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x,
          ped.pose.position.y,
          ped.pose.position.z + dataPtr->actorZOffsets[pathIt->second]),
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
  dropStale(dataPtr->spawnedModelPath);
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
