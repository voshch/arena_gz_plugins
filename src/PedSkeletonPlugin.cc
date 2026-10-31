/// PedSkeletonPlugin drives Gazebo actor pedestrians from arena_peds. Each tick,
/// in PreUpdate (sim thread), it sets every matching actor's world pose via
/// components::TrajectoryPose and poses its skeleton per bone via
/// components::BoneTransforms: the wire body pose is mapped onto the skin bones
/// through the rig/<name>.rig beside the actor SDF, so every simulator renders the same
/// articulated body. Actors whose template cannot be bone-driven (no .rig
/// file, no idle DAE, no skin skeleton) and every actor of a build
/// without the gz-sim overlay keep scrubbing the walk clip via
/// components::AnimationTime.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <filesystem>
#include <system_error>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Skeleton.hh>
#include <gz/common/SkeletonAnimation.hh>
#include <gz/common/SkeletonNode.hh>
#include <gz/common/SubMesh.hh>

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
#include <builtin_interfaces/msg/time.hpp>
#include <arena_people_msgs/msg/pedestrians.hpp>

#include "arena_gz_plugins/BodyRig.hh"
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
// Its first frame is also the rest pose shown without a fresh wire.
static constexpr const char * kIdleAnimName = "idle";
// Optional clip whose first frame gives the bones the rig does not map their
// neutral rotation. Without it the idle clip's first frame is used.
static constexpr const char * kNeutralAnimName = "neutral";
// Meters above a foot's lowest bind vertex that still count as sole.
static constexpr double kSoleBand = 0.01;
// walk.dae advances its root 1.384 m over its 5.79 s clip, a 0.239 m/s natural
// stride, so the cursor advances speed/0.239 per second to keep the feet planted.
static constexpr double kWalkRefSpeed = 0.239;
// Below this speed the ped is idle and the clip is held still.
static constexpr double kIdleSpeed = 0.05;
// Max seconds to dead-reckon a stalled ped.
static constexpr double kMaxExtrap = 0.5;
// 2*pi, converts a gait phase in radians to a fraction of the walk clip.
static constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
// Wire frames buffered per ped.
static constexpr std::size_t kAngleBufferLen = 32;
// Assumed wire period with fewer than 2 buffered frames.
static constexpr double kDefaultPeriod = 0.05;
// Interpolation delay, in wire periods.
static constexpr double kInterpDelayPeriods = 1.5;
// Wire age beyond which the pose blends back to the rest pose.
static constexpr double kStalenessS = 0.5;
// Blend ramp between the rest pose and the wire pose.
static constexpr double kBlendS = 0.3;

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

/// Remove a component if the entity carries it.
template <typename ComponentT>
static void DropComponent(
  gz::sim::EntityComponentManager & _ecm,
  const gz::sim::Entity & _entity)
{
  if (_ecm.Component<ComponentT>(_entity) != nullptr)
    _ecm.RemoveComponent<ComponentT>(_entity);
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
  // Skin skeleton bones (depth-first), their neutral local transforms, their
  // rest rotations and the bundle rig. Empty when the template is
  // clip-scrubbed.
  std::vector<std::string> boneNames;
  std::vector<int> boneParents;
  std::vector<gz::math::Pose3d> neutral;
  std::vector<gz::math::Quaterniond> rest;
  Rig rig;
  // Heel and toe sole point of each foot, as bone index and point in that
  // bone's frame. Empty for a rig on wheels.
  std::vector<std::pair<std::size_t, gz::math::Vector3d>> soles;

  bool BoneDriven() const { return !boneNames.empty(); }
};

/// One received wire frame.
struct WireFrame
{
  double stamp{0.0};
  JointRotations rotations;
  gz::math::Vector3d rootOffset;
  std::unordered_map<std::string, double> drivers;
};

/// Per-ped wire buffer and blend weight.
struct BoneDriver
{
  std::deque<WireFrame> frames;
  double weight{0.0};
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

/// The actor's animation with the given name, nullptr if absent.
static const sdf::Animation * FindAnimation(const sdf::Actor & actor, const std::string & name)
{
  for (uint64_t i = 0; i < actor.AnimationCount(); ++i)
  {
    const sdf::Animation * anim = actor.AnimationByIndex(i);
    if (anim != nullptr && anim->Name() == name)
      return anim;
  }
  return nullptr;
}

/// Resolve an actor-relative filename against the actor SDF's directory.
static std::filesystem::path ResolveActorFile(
  const std::string & filename, const std::string & actorFile)
{
  std::filesystem::path path(filename);
  if (!path.is_absolute())
    path = std::filesystem::path(actorFile).parent_path() / path;
  return path;
}

/// Resolve the "walk" animation's clip relative to the actor SDF's directory
/// and, if it is a BVH, return its duration. 0.0 for non-BVH clips (walk.dae)
/// or a missing walk animation.
static double WalkClipDuration(const sdf::Actor & actor, const std::string & actorFile)
{
  const sdf::Animation * walkAnim = FindAnimation(actor, kWalkAnimName);
  if (walkAnim == nullptr)
    return 0.0;

  const auto clipPath = ResolveActorFile(walkAnim->Filename(), actorFile);
  if (clipPath.extension() != ".bvh")
    return 0.0;

  return ParseBvhDuration(clipPath.string());
}

/// Load the rig/<name>.rig beside the actor SDF. false (warned) unless it is
/// present, readable and not newer than this reader.
static bool LoadRig(const std::filesystem::path & sdfPath, Rig & rig)
{
  const auto path = sdfPath.parent_path() / (sdfPath.stem().string() + ".rig");
  std::string detail;
  switch (LoadRigFile(path, rig, detail))
  {
    case RigStatus::Loaded:
      return true;
    case RigStatus::Missing:
      gzwarn << "[PedSkeletonPlugin] no rig at '" << path.string()
             << "', run rig_human on the bundle, clip-scrubbing its actors\n";
      return false;
    case RigStatus::TooNew:
      gzwarn << "[PedSkeletonPlugin] rig '" << path.string() << "' is version " << detail
             << ", this plugin reads up to " << kRigReader
             << ", update arena_gz_plugins, clip-scrubbing its actors\n";
      return false;
    case RigStatus::Malformed:
      gzwarn << "[PedSkeletonPlugin] rig '" << path.string() << "' is malformed ("
             << detail << "), clip-scrubbing its actors\n";
      return false;
  }
  return false;
}

/// First frame of an actor clip by bone name, false (warned) unless it is a
/// DAE with an animation.
static bool ClipFirstFrame(
  const sdf::Animation & anim, const std::string & actorFile,
  std::map<std::string, gz::math::Matrix4d> & frame)
{
  const auto clipPath = ResolveActorFile(anim.Filename(), actorFile);
  if (clipPath.extension() != ".dae")
  {
    gzwarn << "[PedSkeletonPlugin] " << anim.Name() << " clip '" << clipPath.string()
           << "' is not a DAE, clip-scrubbing '" << actorFile << "'\n";
    return false;
  }
  const gz::common::Mesh * clip = gz::common::MeshManager::Instance()->Load(clipPath.string());
  const gz::common::SkeletonPtr clipSkel =
    (clip != nullptr && clip->HasSkeleton()) ? clip->MeshSkeleton() : nullptr;
  if (clipSkel == nullptr || clipSkel->AnimationCount() == 0)
  {
    gzwarn << "[PedSkeletonPlugin] " << anim.Name() << " clip '" << clipPath.string()
           << "' has no animation, clip-scrubbing '" << actorFile << "'\n";
    return false;
  }
  frame = clipSkel->Animation(0)->PoseAt(0.0, false);
  return true;
}

/// Heel and toe sole point of each foot in the frame of the bone it rides on,
/// from the rig's soles joints. None for a rig without soles or on wheels.
static std::vector<std::pair<std::size_t, gz::math::Vector3d>> SolePoints(
  const gz::common::Mesh & skin, const gz::common::Skeleton & skel,
  const std::unordered_map<std::string, std::size_t> & boneIndex, const Rig & rig)
{
  std::vector<std::pair<std::size_t, gz::math::Vector3d>> soles;
  if (rig.soles.size() != 4 || rig.HasDistanceDriver())
    return soles;
  const gz::math::Matrix4d bindShape = skel.BindShapeTransform();
  for (const std::size_t first : {0, 2})
  {
    const std::string * footBone = rig.JointBone(rig.soles[first]);
    const std::string * toeBone = rig.JointBone(rig.soles[first + 1]);
    if (footBone == nullptr || toeBone == nullptr)
      continue;
    const gz::common::SkeletonNode * foot = skel.NodeByName(*footBone);
    const gz::common::SkeletonNode * toe = skel.NodeByName(*toeBone);
    if (foot == nullptr || toe == nullptr)
      continue;
    std::vector<std::pair<gz::math::Vector3d, const gz::common::SkeletonNode *>> cloud;
    for (unsigned int s = 0; s < skin.SubMeshCount(); ++s)
    {
      const auto sub = skin.SubMeshByIndex(s).lock();
      if (!sub)
        continue;
      std::vector<float> weight(sub->VertexCount(), 0.0f);
      std::vector<const gz::common::SkeletonNode *> owner(sub->VertexCount(), nullptr);
      for (unsigned int a = 0; a < sub->NodeAssignmentsCount(); ++a)
      {
        const gz::common::NodeAssignment assignment = sub->NodeAssignmentByIndex(a);
        if (assignment.vertexIndex < weight.size() && assignment.weight > weight[assignment.vertexIndex])
        {
          weight[assignment.vertexIndex] = assignment.weight;
          owner[assignment.vertexIndex] = skel.NodeByHandle(assignment.nodeIndex);
        }
      }
      for (unsigned int v = 0; v < sub->VertexCount(); ++v)
      {
        if (owner[v] == foot || owner[v] == toe)
          cloud.emplace_back(bindShape * sub->Vertex(v), owner[v]);
      }
    }
    if (cloud.empty())
      continue;
    gz::math::Vector3d forward = toe->InverseBindTransform().Inverse().Translation() -
      foot->InverseBindTransform().Inverse().Translation();
    forward.Z(0.0);
    double lowest = cloud.front().first.Z();
    for (const auto & vertex : cloud)
      lowest = std::min(lowest, vertex.first.Z());
    std::vector<std::size_t> sole;
    double front = -std::numeric_limits<double>::infinity();
    double back = std::numeric_limits<double>::infinity();
    for (std::size_t v = 0; v < cloud.size(); ++v)
    {
      if (cloud[v].first.Z() > lowest + kSoleBand)
        continue;
      sole.push_back(v);
      front = std::max(front, cloud[v].first.Dot(forward));
      back = std::min(back, cloud[v].first.Dot(forward));
    }
    const double middle = 0.5 * (front + back);
    for (const bool ahead : {false, true})
    {
      const std::pair<gz::math::Vector3d, const gz::common::SkeletonNode *> * pick = nullptr;
      for (const std::size_t v : sole)
      {
        if ((cloud[v].first.Dot(forward) > middle) == ahead &&
            (pick == nullptr || cloud[v].first.Z() < pick->first.Z()))
          pick = &cloud[v];
      }
      if (pick != nullptr)
        soles.emplace_back(boneIndex.at(pick->second->Name()), pick->second->InverseBindTransform() * pick->first);
    }
  }
  return soles;
}

/// Height of a template's lowest sole point above the skeleton origin for
/// one set of bone-local poses.
static double SoleHeight(
  const CachedActor & tmpl, const std::map<std::string, gz::math::Pose3d> & bones)
{
  std::vector<gz::math::Matrix4d> world(tmpl.boneNames.size());
  for (std::size_t b = 0; b < world.size(); ++b)
  {
    const gz::math::Matrix4d local(bones.at(tmpl.boneNames[b]));
    world[b] = tmpl.boneParents[b] < 0 ? local : world[tmpl.boneParents[b]] * local;
  }
  double height = std::numeric_limits<double>::infinity();
  for (const auto & [bone, point] : tmpl.soles)
    height = std::min(height, (world[bone] * point).Z());
  return height;
}

/// Fill the template's bones, rest pose (idle clip first frame), neutral
/// stance (the rig's mapped bones posed at its stance, the others at the
/// neutral clip first frame, the rest pose without one) and rig from its skin
/// skeleton. The root and unanimated bones take their skin transform. Leaves
/// the template clip-scrubbed (warned) when any input is unavailable.
static void LoadBoneDrive(CachedActor & entry, const std::string & actorFile)
{
  Rig rig;
  if (!LoadRig(actorFile, rig))
    return;
  const sdf::Animation * idleAnim = FindAnimation(entry.actor, kIdleAnimName);
  if (idleAnim == nullptr)
  {
    gzwarn << "[PedSkeletonPlugin] '" << actorFile << "' has no '" << kIdleAnimName
           << "' animation, clip-scrubbing it\n";
    return;
  }
  std::map<std::string, gz::math::Matrix4d> restFrame;
  if (!ClipFirstFrame(*idleAnim, actorFile, restFrame))
    return;
  std::map<std::string, gz::math::Matrix4d> neutralFrame = restFrame;
  const sdf::Animation * neutralAnim = FindAnimation(entry.actor, kNeutralAnimName);
  if (neutralAnim != nullptr && !ClipFirstFrame(*neutralAnim, actorFile, neutralFrame))
    return;

  const auto skinPath = ResolveActorFile(entry.actor.SkinFilename(), actorFile);
  const gz::common::Mesh * skin = gz::common::MeshManager::Instance()->Load(skinPath.string());
  const gz::common::SkeletonPtr skel =
    (skin != nullptr && skin->HasSkeleton()) ? skin->MeshSkeleton() : nullptr;
  if (skel == nullptr || skel->RootNode() == nullptr)
  {
    gzwarn << "[PedSkeletonPlugin] skin '" << skinPath.string()
           << "' has no skeleton, clip-scrubbing '" << actorFile << "'\n";
    return;
  }

  std::vector<std::pair<gz::common::SkeletonNode *, int>> stack{{skel->RootNode(), -1}};
  while (!stack.empty())
  {
    const auto [node, parent] = stack.back();
    stack.pop_back();
    const auto local = [node](const std::map<std::string, gz::math::Matrix4d> & frame)
      {
        const auto channel = frame.find(node->Name());
        return (node->IsRootNode() || channel == frame.end()) ? node->Transform() : channel->second;
      };
    const gz::math::Matrix4d neutral = local(neutralFrame);
    const int index = static_cast<int>(entry.boneNames.size());
    entry.boneNames.push_back(node->Name());
    entry.boneParents.push_back(parent);
    entry.neutral.emplace_back(neutral.Translation(), neutral.Rotation().Normalized());
    entry.rest.push_back(local(restFrame).Rotation().Normalized());
    for (unsigned int i = node->ChildCount(); i > 0; --i)
      stack.emplace_back(node->Child(i - 1), index);
  }
  entry.neutral = PoseBones(
    entry.boneNames, entry.boneParents, entry.neutral,
    BoneWorldRotations(rig, rig.stance), {}, {}, gz::math::Vector3d::Zero);

  std::unordered_map<std::string, std::size_t> boneIndex;
  for (std::size_t b = 0; b < entry.boneNames.size(); ++b)
    boneIndex.emplace(entry.boneNames[b], b);
  entry.soles = SolePoints(*skin, *skel, boneIndex, rig);
  entry.rig = std::move(rig);
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

  // ped topics end in this, <topic_suffix> in the plugin SDF
  std::string topicSuffix{"/arena_peds"};

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

  // name -> buffered wire frames and blend weight, under pedsMutex.
  std::unordered_map<std::string, BoneDriver> boneDrivers;

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

  static double StampSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
  }

  /// Buffer one ped's wire body pose and driven joint angles. Caller holds
  /// pedsMutex.
  void PushWireFrame(
    const arena_people_msgs::msg::Pedestrians & msg,
    const arena_people_msgs::msg::Pedestrian & ped)
  {
    const auto & joints = ped.joint_state;
    const bool ownStamp =
      joints.header.stamp.sec != 0 || joints.header.stamp.nanosec != 0;
    WireFrame frame;
    frame.stamp = StampSeconds(ownStamp ? joints.header.stamp : msg.header.stamp);
    const auto & body = ped.body;
    for (std::size_t i = 0; i < body.joints.size() && i < body.rotations.size(); ++i)
    {
      const auto & q = body.rotations[i];
      frame.rotations[body.joints[i]] = gz::math::Quaterniond(q.w, q.x, q.y, q.z).Normalized();
    }
    frame.rootOffset = gz::math::Vector3d(
      body.root_offset.x, body.root_offset.y, body.root_offset.z);
    for (std::size_t i = 0; i < joints.name.size() && i < joints.position.size(); ++i)
      frame.drivers[joints.name[i]] = joints.position[i];
    auto & driver = boneDrivers[ped.name];
    driver.frames.push_back(std::move(frame));
    if (driver.frames.size() > kAngleBufferLen)
      driver.frames.pop_front();
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
            if (!ped.body.joints.empty() || !ped.joint_state.name.empty())
              PushWireFrame(*msg, ped);
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
              boneDrivers.erase(name);
              pendingRemovals.insert(name);
            }
          }
          prev = std::move(names);
        });
  }

  /// Scan all advertised topics ending in the topic suffix and subscribe.
  void DiscoverTopics()
  {
    if (!rosNode)
      return;
    const std::string & suffix = topicSuffix;
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
#ifdef ARENA_GZ_BONE_TRANSFORMS
    LoadBoneDrive(entry, file);
#endif
    return &actorTemplates.emplace(path, std::move(entry)).first->second;
  }

  /// Mean of the last four positive stamp gaps, kDefaultPeriod with fewer
  /// than two frames or no positive gap.
  static double WirePeriod(const std::deque<WireFrame> & frames)
  {
    if (frames.size() < 2)
      return kDefaultPeriod;
    const std::size_t first = frames.size() > 5 ? frames.size() - 5 : 0;
    double sum = 0.0;
    int gaps = 0;
    for (std::size_t i = first + 1; i < frames.size(); ++i)
    {
      const double gap = frames[i].stamp - frames[i - 1].stamp;
      if (gap > 0.0)
      {
        sum += gap;
        ++gaps;
      }
    }
    return gaps > 0 ? sum / gaps : kDefaultPeriod;
  }

  /// Frame at time t, interpolated between the bracketing frames and clamped
  /// at both ends. frames must be non-empty.
  static WireFrame SampleWire(const std::deque<WireFrame> & frames, double t)
  {
    if (frames.size() == 1 || t <= frames.front().stamp)
      return frames.front();
    if (t >= frames.back().stamp)
      return frames.back();

    const auto upper = std::upper_bound(
      frames.begin(), frames.end(), t,
      [](double value, const WireFrame & frame) { return value < frame.stamp; });
    const auto idx = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(
      std::distance(frames.begin(), upper) - 1, 0,
      static_cast<std::ptrdiff_t>(frames.size()) - 2));
    const WireFrame & a = frames[idx];
    const WireFrame & b = frames[idx + 1];
    const double span = b.stamp - a.stamp;
    const double alpha = span <= 0.0 ? 0.0 : (t - a.stamp) / span;
    WireFrame frame;
    frame.stamp = t;
    for (const auto & [name, rotation] : a.rotations)
    {
      const auto other = b.rotations.find(name);
      frame.rotations[name] = gz::math::Quaterniond::Slerp(
        alpha, rotation,
        other == b.rotations.end() ? gz::math::Quaterniond::Identity : other->second, true);
    }
    for (const auto & [name, rotation] : b.rotations)
    {
      if (a.rotations.count(name) == 0)
        frame.rotations[name] = gz::math::Quaterniond::Slerp(
          alpha, gz::math::Quaterniond::Identity, rotation, true);
    }
    frame.rootOffset = a.rootOffset * (1.0 - alpha) + b.rootOffset * alpha;
    for (const auto & [name, angle] : a.drivers)
    {
      const auto other = b.drivers.find(name);
      frame.drivers[name] =
        (1.0 - alpha) * angle + alpha * (other == b.drivers.end() ? 0.0 : other->second);
    }
    for (const auto & [name, angle] : b.drivers)
    {
      if (a.drivers.count(name) == 0)
        frame.drivers[name] = alpha * angle;
    }
    return frame;
  }

  /// Local pose of every bone for one wire frame, through the rig.
  static std::vector<gz::math::Pose3d> WirePose(const CachedActor & tmpl, const WireFrame & frame)
  {
    JointRotations driven;
    for (const auto & driver : tmpl.rig.drivers)
    {
      const auto angle = frame.drivers.find(driver.name);
      driven[driver.bone] = gz::math::Quaterniond(
        driver.axis, angle == frame.drivers.end() ? 0.0 : angle->second);
    }
    return PoseBones(
      tmpl.boneNames, tmpl.boneParents, tmpl.neutral,
      BoneWorldRotations(tmpl.rig, WorldRotations(tmpl.rig, frame.rotations)), driven,
      tmpl.rig.TopBone(), frame.rootOffset);
  }

  /// Ramp the blend weight and blend from the rest pose to the
  /// delay-compensated wire pose. Caller holds pedsMutex.
  static void BlendWire(
    BoneDriver & driver, const CachedActor & tmpl, double simTime, double dt,
    std::vector<gz::math::Pose3d> & pose)
  {
    if (driver.frames.empty())
    {
      driver.weight = 0.0;
      return;
    }
    const double target = (simTime - driver.frames.back().stamp) > kStalenessS ? 0.0 : 1.0;
    const double step = dt / kBlendS;
    driver.weight = (target > driver.weight)
      ? std::min(target, driver.weight + step)
      : std::max(target, driver.weight - step);
    if (driver.weight <= 0.0)
      return;

    const std::vector<gz::math::Pose3d> wire = WirePose(
      tmpl, SampleWire(driver.frames, simTime - kInterpDelayPeriods * WirePeriod(driver.frames)));
    for (std::size_t b = 0; b < pose.size(); ++b)
    {
      pose[b] = gz::math::Pose3d(
        pose[b].Pos() + (wire[b].Pos() - pose[b].Pos()) * driver.weight,
        gz::math::Quaterniond::Slerp(driver.weight, pose[b].Rot(), wire[b].Rot(), true));
    }
  }

  /// Local transform of every bone of a bone-driven template for one ped:
  /// the wire pose where frames are buffered and fresh, the rest pose
  /// otherwise.
  std::map<std::string, gz::math::Pose3d> EvaluateBones(
    const std::string & pedName, const CachedActor & tmpl, double simTime, double dt)
  {
    std::vector<gz::math::Pose3d> pose(tmpl.neutral.size());
    for (std::size_t b = 0; b < pose.size(); ++b)
      pose[b] = gz::math::Pose3d(tmpl.neutral[b].Pos(), tmpl.rest[b]);
    {
      std::lock_guard<std::mutex> lock(pedsMutex);
      const auto driver = boneDrivers.find(pedName);
      if (driver != boneDrivers.end())
        BlendWire(driver->second, tmpl, simTime, dt, pose);
    }
    std::map<std::string, gz::math::Pose3d> bones;
    for (std::size_t b = 0; b < pose.size(); ++b)
      bones.emplace(tmpl.boneNames[b], pose[b]);
    return bones;
  }

  /// Drop a ped's buffered wire frames and blend weight.
  void ResetBoneDriver(const std::string & pedName)
  {
    std::lock_guard<std::mutex> lock(pedsMutex);
    boneDrivers[pedName] = BoneDriver{};
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
  if (_sdf->HasElement("topic_suffix"))
    dataPtr->topicSuffix = _sdf->Get<std::string>("topic_suffix");

  if (!dataPtr->enabled)
  {
    gzmsg << "[PedSkeletonPlugin] disabled via SDF.\n";
    return;
  }

  // The plugin attaches to the world, so its entity is the actors' parent.
  dataPtr->worldEntity = _entity;
  dataPtr->eventMgr = &_eventMgr;

#ifndef ARENA_GZ_BONE_TRANSFORMS
  gzwarn << "[PedSkeletonPlugin] built without BoneTransforms (no gz-sim overlay), "
         << "actors keep the clip-scrub path\n";
#endif

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  dataPtr->rosNode = std::make_shared<rclcpp::Node>("ped_actor_plugin");
  dataPtr->spinThread =
    std::thread(&PedSkeletonPluginPrivate::SpinLoop, dataPtr.get());

  gzmsg << "[PedSkeletonPlugin] configured, driving actor pose + skeleton.\n";
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
      double groundZ = (tmpl != nullptr) ? tmpl->groundZ : kZOffset;
#ifdef ARENA_GZ_BONE_TRANSFORMS
      std::map<std::string, gz::math::Pose3d> bones;
      if (tmpl != nullptr && tmpl->BoneDriven())
      {
        bones = dataPtr->EvaluateBones(pedName, *tmpl, simTimeSec, dt);
        if (!tmpl->soles.empty())
          groundZ = -SoleHeight(*tmpl, bones);
      }
#endif

      const gz::math::Quaterniond orientation(
        ped.pose.orientation.w, ped.pose.orientation.x,
        ped.pose.orientation.y, ped.pose.orientation.z);
      gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x + ped.twist.linear.x * elapsed,
          ped.pose.position.y + ped.twist.linear.y * elapsed,
          ped.pose.position.z) +
        orientation.RotateVector(gz::math::Vector3d(0.0, 0.0, groundZ)),
        orientation);
      SetComponent<gz::sim::components::Pose>(_ecm, entity, gz::math::Pose3d::Zero);
      SetComponent<gz::sim::components::TrajectoryPose>(_ecm, entity, worldPose);

#ifdef ARENA_GZ_BONE_TRANSFORMS
      if (tmpl != nullptr && tmpl->BoneDriven())
      {
        DropComponent<gz::sim::components::AnimationName>(_ecm, entity);
        DropComponent<gz::sim::components::AnimationTime>(_ecm, entity);
        SetComponent<gz::sim::components::BoneTransforms>(_ecm, entity, bones);
        return true;
      }
#endif

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
      double groundZ = tmpl->groundZ;
#ifdef ARENA_GZ_BONE_TRANSFORMS
      std::map<std::string, gz::math::Pose3d> bones;
      if (tmpl->BoneDriven())
      {
        dataPtr->ResetBoneDriver(pedName);
        bones = dataPtr->EvaluateBones(pedName, *tmpl, simTimeSec, dt);
        if (!tmpl->soles.empty())
          groundZ = -SoleHeight(*tmpl, bones);
      }
#endif
      const gz::math::Quaterniond orientation(
        ped.pose.orientation.w, ped.pose.orientation.x,
        ped.pose.orientation.y, ped.pose.orientation.z);
      const gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x,
          ped.pose.position.y,
          ped.pose.position.z) +
        orientation.RotateVector(gz::math::Vector3d(0.0, 0.0, groundZ)),
        orientation);
      SetComponent<gz::sim::components::Pose>(_ecm, entity, gz::math::Pose3d::Zero);
      SetComponent<gz::sim::components::TrajectoryPose>(_ecm, entity, worldPose);
#ifdef ARENA_GZ_BONE_TRANSFORMS
      if (tmpl->BoneDriven())
        SetComponent<gz::sim::components::BoneTransforms>(_ecm, entity, bones);
#endif
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
