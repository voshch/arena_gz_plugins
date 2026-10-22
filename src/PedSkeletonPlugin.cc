/// PedSkeletonPlugin drives Gazebo actor pedestrians from arena_peds. Each tick,
/// in PreUpdate (sim thread), it sets every matching actor's world pose via
/// components::TrajectoryPose and poses its skeleton per bone via
/// components::BoneTransforms: the wire joint angles are composed onto the
/// neutral stance through the same bone map as the Isaac provider, so both
/// simulators render the same articulated gait. Actors whose template cannot
/// be bone-driven (no <bone_map>, no idle DAE, no skin skeleton) and every
/// actor of a build without the gz-sim overlay keep scrubbing the walk clip
/// via components::AnimationTime.

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

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
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
// Its first frame is also the rest pose shown without a fresh wire.
static constexpr const char * kIdleAnimName = "idle";
// Optional clip whose first frame the wire angles compose onto. Without it the
// idle clip's first frame is used.
static constexpr const char * kNeutralAnimName = "neutral";
// Bones whose neutral stance is their skin transform.
static const std::unordered_set<std::string> kBindStanceBones{
  "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase",
  "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"};
// Bones that carry a rig on wheels, which then keeps its SDF ground offset.
static const std::unordered_set<std::string> kWheelBones{"WheelL", "WheelR"};
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

/// One wire joint's contribution to a skeleton bone's local rotation.
struct BoneTarget
{
  std::string bone;
  gz::math::Vector3d axis;
  double gain{1.0};
};

/// Wire joint name -> targets, in bone map file order.
using BoneMap = std::vector<std::pair<std::string, std::vector<BoneTarget>>>;

/// A bone map target resolved against one skeleton.
struct SchemaEntry
{
  std::size_t wireIdx{0};
  std::size_t boneIdx{0};
  gz::math::Vector3d axis;
  double gain{0.0};
};

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
  // rest rotations and the resolved wire schema. Empty when the template is
  // clip-scrubbed.
  std::vector<std::string> boneNames;
  std::vector<int> boneParents;
  std::vector<gz::math::Pose3d> neutral;
  std::vector<gz::math::Quaterniond> rest;
  std::vector<SchemaEntry> schema;
  // Heel and toe sole point of each foot, as bone index and point in that
  // bone's frame. Empty for a rig on wheels.
  std::vector<std::pair<std::size_t, gz::math::Vector3d>> soles;

  bool BoneDriven() const { return !boneNames.empty(); }
};

/// One received wire frame, angles in bone map order.
struct WireFrame
{
  double stamp{0.0};
  std::vector<double> angles;
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

/// Read the bone map JSON ({wire name: [{bone, axis, sign, scale}, ...]})
/// preserving key order. false (warned) if the file is missing or malformed.
static bool LoadBoneMap(const std::string & path, BoneMap & boneMap)
{
  std::ifstream file(path);
  if (!file.is_open())
  {
    gzwarn << "[PedSkeletonPlugin] cannot open bone map '" << path
           << "', actors keep the clip-scrub path\n";
    return false;
  }
  try
  {
    const auto doc = nlohmann::ordered_json::parse(file);
    if (!doc.is_object())
    {
      gzwarn << "[PedSkeletonPlugin] bone map '" << path
             << "' root is not an object, actors keep the clip-scrub path\n";
      return false;
    }
    for (const auto & [name, targets] : doc.items())
    {
      std::vector<BoneTarget> parsed;
      for (const auto & target : targets)
      {
        const auto & axis = target.at("axis");
        parsed.push_back({
          target.at("bone").get<std::string>(),
          gz::math::Vector3d(
            axis.at(0).get<double>(), axis.at(1).get<double>(), axis.at(2).get<double>()),
          target.at("sign").get<double>() * target.value("scale", 1.0)});
      }
      boneMap.emplace_back(name, std::move(parsed));
    }
  }
  catch (const nlohmann::json::exception & e)
  {
    gzwarn << "[PedSkeletonPlugin] bone map '" << path << "' is malformed ("
           << e.what() << "), actors keep the clip-scrub path\n";
    boneMap.clear();
    return false;
  }
  return true;
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
/// none for a rig on wheels.
static std::vector<std::pair<std::size_t, gz::math::Vector3d>> SolePoints(
  const gz::common::Mesh & skin, const gz::common::Skeleton & skel,
  const std::unordered_map<std::string, std::size_t> & boneIndex)
{
  std::vector<std::pair<std::size_t, gz::math::Vector3d>> soles;
  for (const auto & wheel : kWheelBones)
  {
    if (boneIndex.count(wheel) > 0)
      return soles;
  }
  const gz::math::Matrix4d bindShape = skel.BindShapeTransform();
  for (const std::string side : {"Left", "Right"})
  {
    const gz::common::SkeletonNode * foot = skel.NodeByName(side + "Foot");
    const gz::common::SkeletonNode * toe = skel.NodeByName(side + "ToeBase");
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
/// stance (neutral clip first frame, the rest pose without one) and wire schema
/// from its skin skeleton. The root and unanimated bones take their skin
/// transform, and so do the kBindStanceBones in the neutral stance. Leaves the
/// template clip-scrubbed (warned) when any input is unavailable.
static void LoadBoneDrive(
  CachedActor & entry, const std::string & actorFile, const BoneMap & boneMap)
{
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
    const gz::math::Matrix4d neutral =
      kBindStanceBones.count(node->Name()) > 0 ? node->Transform() : local(neutralFrame);
    const int index = static_cast<int>(entry.boneNames.size());
    entry.boneNames.push_back(node->Name());
    entry.boneParents.push_back(parent);
    entry.neutral.emplace_back(neutral.Translation(), neutral.Rotation().Normalized());
    entry.rest.push_back(local(restFrame).Rotation().Normalized());
    for (unsigned int i = node->ChildCount(); i > 0; --i)
      stack.emplace_back(node->Child(i - 1), index);
  }

  std::unordered_map<std::string, std::size_t> boneIndex;
  for (std::size_t b = 0; b < entry.boneNames.size(); ++b)
    boneIndex.emplace(entry.boneNames[b], b);
  entry.soles = SolePoints(*skin, *skel, boneIndex);
  for (std::size_t w = 0; w < boneMap.size(); ++w)
  {
    for (const auto & target : boneMap[w].second)
    {
      const auto bone = boneIndex.find(target.bone);
      if (bone == boneIndex.end())
        continue;
      const double norm = target.axis.Length();
      entry.schema.push_back({
        w, bone->second,
        norm > 0.0 ? target.axis / norm : gz::math::Vector3d::Zero,
        norm > 0.0 ? target.gain : 0.0});
    }
  }
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

  // Wire joint -> bone targets from <bone_map>, empty when bone-driving is off.
  BoneMap boneMap;
  std::unordered_map<std::string, std::size_t> wireIndex;
  // name -> buffered wire frames and blend weight, under pedsMutex.
  std::unordered_map<std::string, BoneDriver> boneDrivers;
  std::unordered_set<std::string> warnedJoints;

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

  /// Buffer one ped's wire joint angles in bone map order. Caller holds pedsMutex.
  void PushWireFrame(
    const arena_people_msgs::msg::Pedestrians & msg,
    const arena_people_msgs::msg::Pedestrian & ped)
  {
    const auto & joints = ped.joint_state;
    const bool ownStamp =
      joints.header.stamp.sec != 0 || joints.header.stamp.nanosec != 0;
    WireFrame frame{
      StampSeconds(ownStamp ? joints.header.stamp : msg.header.stamp),
      std::vector<double>(boneMap.size(), 0.0)};
    for (std::size_t i = 0; i < joints.name.size() && i < joints.position.size(); ++i)
    {
      const auto wire = wireIndex.find(joints.name[i]);
      if (wire == wireIndex.end())
      {
        if (warnedJoints.insert(joints.name[i]).second)
          gzwarn << "[PedSkeletonPlugin] unrecognized joint name '" << joints.name[i]
                 << "', ignoring\n";
        continue;
      }
      frame.angles[wire->second] = joints.position[i];
    }
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
            if (!boneMap.empty() && !ped.joint_state.name.empty())
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
    if (!boneMap.empty())
      LoadBoneDrive(entry, file, boneMap);
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

  /// Angles at time t, linearly interpolated between the bracketing frames and
  /// clamped at both ends. frames must be non-empty.
  static std::vector<double> SampleWire(const std::deque<WireFrame> & frames, double t)
  {
    if (frames.size() == 1 || t <= frames.front().stamp)
      return frames.front().angles;
    if (t >= frames.back().stamp)
      return frames.back().angles;

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
    std::vector<double> angles(a.angles.size());
    for (std::size_t k = 0; k < angles.size(); ++k)
      angles[k] = (1.0 - alpha) * a.angles[k] + alpha * b.angles[k];
    return angles;
  }

  /// Ramp the blend weight and slerp from the rest pose to the
  /// delay-compensated wire angles composed onto the neutral stance. Caller
  /// holds pedsMutex.
  static void BlendWire(
    BoneDriver & driver, const CachedActor & tmpl, double simTime, double dt,
    std::vector<gz::math::Quaterniond> & rotation)
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

    const std::vector<double> angles =
      SampleWire(driver.frames, simTime - kInterpDelayPeriods * WirePeriod(driver.frames));
    std::vector<gz::math::Quaterniond> wire(tmpl.neutral.size());
    for (std::size_t b = 0; b < wire.size(); ++b)
      wire[b] = tmpl.neutral[b].Rot();
    for (const auto & entry : tmpl.schema)
    {
      const gz::math::Quaterniond delta(entry.axis, entry.gain * angles[entry.wireIdx]);
      wire[entry.boneIdx] = (wire[entry.boneIdx] * delta).Normalized();
    }
    for (std::size_t b = 0; b < rotation.size(); ++b)
      rotation[b] = gz::math::Quaterniond::Slerp(driver.weight, tmpl.rest[b], wire[b], true);
  }

  /// Local transform of every bone of a bone-driven template for one ped:
  /// the wire pose where frames are buffered and fresh, the rest pose
  /// otherwise.
  std::map<std::string, gz::math::Pose3d> EvaluateBones(
    const std::string & pedName, const CachedActor & tmpl, double simTime, double dt)
  {
    std::vector<gz::math::Quaterniond> rotation = tmpl.rest;
    {
      std::lock_guard<std::mutex> lock(pedsMutex);
      const auto driver = boneDrivers.find(pedName);
      if (driver != boneDrivers.end())
        BlendWire(driver->second, tmpl, simTime, dt, rotation);
    }
    std::map<std::string, gz::math::Pose3d> bones;
    for (std::size_t b = 0; b < rotation.size(); ++b)
      bones.emplace(tmpl.boneNames[b], gz::math::Pose3d(tmpl.neutral[b].Pos(), rotation[b]));
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
#else
  if (!_sdf->HasElement("bone_map"))
  {
    gzwarn << "[PedSkeletonPlugin] no <bone_map>, actors keep the clip-scrub path\n";
  }
  else
  {
    const auto bonePath = _sdf->Get<std::string>("bone_map");
    if (LoadBoneMap(bonePath, dataPtr->boneMap))
    {
      for (std::size_t w = 0; w < dataPtr->boneMap.size(); ++w)
        dataPtr->wireIndex.emplace(dataPtr->boneMap[w].first, w);
      gzmsg << "[PedSkeletonPlugin] bone map '" << bonePath << "' maps "
            << dataPtr->boneMap.size() << " wire joints\n";
    }
  }
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
      const gz::math::Pose3d worldPose(
        gz::math::Vector3d(
          ped.pose.position.x,
          ped.pose.position.y,
          ped.pose.position.z + groundZ),
        gz::math::Quaterniond(
          ped.pose.orientation.w, ped.pose.orientation.x,
          ped.pose.orientation.y, ped.pose.orientation.z));
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
