#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "arena_gz_plugins/Npz.hh"

namespace arena_gz_plugins
{

/// One skin bone a rig joint turns, with its share of the joint angle.
struct RigBone
{
  std::string bone;
  double weight{1.0};
  gz::math::Quaterniond offset;
};

/// A rig joint, parent empty for a root.
struct RigJoint
{
  std::string name;
  std::string parent;
  std::vector<RigBone> bones;
};

/// A driven joint of a rig, turning its bone about a bone-local axis.
struct RigDriver
{
  std::string name;
  std::string bone;
  gz::math::Vector3d axis;
  bool distance{false};
};

/// The body rig of a bundle, from its .rig file.
struct Rig
{
  std::vector<RigJoint> joints;
  std::unordered_map<std::string, gz::math::Quaterniond> stance;
  std::vector<std::string> soles;
  std::vector<RigDriver> drivers;

  /// First bone of the first root joint, empty if none.
  std::string TopBone() const
  {
    for (const auto & joint : joints)
    {
      if (joint.parent.empty() && !joint.bones.empty())
        return joint.bones.front().bone;
    }
    return {};
  }

  /// Last bone of the named joint, nullptr if the joint is unknown.
  const std::string * JointBone(const std::string & name) const
  {
    for (const auto & joint : joints)
    {
      if (joint.name == name && !joint.bones.empty())
        return &joint.bones.back().bone;
    }
    return nullptr;
  }

  bool HasDistanceDriver() const
  {
    for (const auto & driver : drivers)
    {
      if (driver.distance)
        return true;
    }
    return false;
  }
};

/// Highest rig/version the reader understands.
inline constexpr std::int64_t kRigReader = 1;

enum class RigStatus
{
  Loaded,
  Missing,
  TooNew,
  Malformed,
};

/// Read a .rig file into rig. detail carries the file's version for TooNew and
/// the error for Malformed. The rig is left empty unless Loaded.
inline RigStatus LoadRigFile(const std::filesystem::path & path, Rig & rig, std::string & detail)
{
  rig = Rig{};
  std::error_code ec;
  if (!std::filesystem::exists(path, ec))
    return RigStatus::Missing;
  try
  {
    const auto arrays = ReadNpz(path.string());
    const auto get = [&arrays](const std::string & name) -> const NpyArray &
      {
        const auto found = arrays.find(name);
        if (found == arrays.end())
          throw std::runtime_error("missing member '" + name + "'");
        return found->second;
      };
    const std::int64_t version = get("rig/version").Ints().at(0);
    if (version > kRigReader)
    {
      detail = std::to_string(version);
      return RigStatus::TooNew;
    }

    const auto names = get("joints/names").Strings();
    const auto parents = get("joints/parents").Ints();
    const auto stance = get("joints/stance").Doubles();
    const auto boneNames = get("bones/names").Strings();
    const auto boneJoints = get("bones/joints").Ints();
    const auto weights = get("bones/weights").Doubles();
    const auto offsets = get("bones/offsets").Doubles();
    const auto quat = [](const std::vector<double> & v, std::size_t i)
      {
        return gz::math::Quaterniond(v.at(4 * i + 3), v.at(4 * i), v.at(4 * i + 1), v.at(4 * i + 2))
               .Normalized();
      };
    if (parents.size() != names.size() || stance.size() != 4 * names.size())
      throw std::runtime_error("joints arrays differ in length");
    if (boneJoints.size() != boneNames.size() || weights.size() != boneNames.size() ||
      offsets.size() != 4 * boneNames.size())
      throw std::runtime_error("bones arrays differ in length");

    for (std::size_t j = 0; j < names.size(); ++j)
    {
      if (parents[j] < -1 || parents[j] >= static_cast<std::int64_t>(names.size()))
        throw std::runtime_error("joint '" + names[j] + "' has a parent out of range");
      RigJoint joint;
      joint.name = names[j];
      if (parents[j] >= 0)
        joint.parent = names[parents[j]];
      rig.joints.push_back(std::move(joint));
      rig.stance.emplace(names[j], quat(stance, j));
    }
    for (std::size_t b = 0; b < boneNames.size(); ++b)
    {
      if (boneJoints[b] < 0 || boneJoints[b] >= static_cast<std::int64_t>(names.size()))
        throw std::runtime_error("bone '" + boneNames[b] + "' has a joint out of range");
      rig.joints[boneJoints[b]].bones.push_back({boneNames[b], weights[b], quat(offsets, b)});
    }

    if (arrays.count("soles/joints") > 0)
    {
      for (const auto joint : arrays.at("soles/joints").Ints())
        rig.soles.push_back(names.at(joint));
    }
    if (arrays.count("drivers/names") > 0)
    {
      const auto driverNames = get("drivers/names").Strings();
      const auto driverBones = get("drivers/bones").Strings();
      const auto axes = get("drivers/axes").Doubles();
      const auto settings = get("drivers/settings").Strings();
      if (driverBones.size() != driverNames.size() || axes.size() != 3 * driverNames.size() ||
        settings.size() != driverNames.size())
        throw std::runtime_error("drivers arrays differ in length");
      for (std::size_t d = 0; d < driverNames.size(); ++d)
      {
        const gz::math::Vector3d axis(axes[3 * d], axes[3 * d + 1], axes[3 * d + 2]);
        if (axis.Length() <= 0.0)
          continue;
        const auto from = settings[d].find("\"from\"");
        const auto open = from == std::string::npos ? from : settings[d].find('"', from + 6);
        const auto close = open == std::string::npos ? open : settings[d].find('"', open + 1);
        const bool distance = close != std::string::npos &&
          settings[d].compare(open + 1, close - open - 1, "distance") == 0;
        rig.drivers.push_back({driverNames[d], driverBones[d], axis / axis.Length(), distance});
      }
    }
  }
  catch (const std::exception & e)
  {
    rig = Rig{};
    detail = e.what();
    return RigStatus::Malformed;
  }
  return RigStatus::Loaded;
}

using JointRotations = std::unordered_map<std::string, gz::math::Quaterniond>;

/// Rotation about the axis of q by c times its angle.
inline gz::math::Quaterniond QuatPower(const gz::math::Quaterniond & q, double c)
{
  const double sign = q.W() < 0.0 ? -1.0 : 1.0;
  const gz::math::Vector3d vec(sign * q.X(), sign * q.Y(), sign * q.Z());
  const double norm = vec.Length();
  if (norm < 1e-12)
    return gz::math::Quaterniond::Identity;
  return gz::math::Quaterniond(vec / norm, c * 2.0 * std::atan2(norm, sign * q.W()));
}

/// A joint's rotation relative to its parent in the neutral stance.
inline gz::math::Quaterniond StanceLocal(const Rig & rig, const RigJoint & joint)
{
  const auto held = rig.stance.find(joint.name);
  if (held == rig.stance.end())
    return gz::math::Quaterniond::Identity;
  const auto above = rig.stance.find(joint.parent);
  return above == rig.stance.end() ? held->second : above->second.Inverse() * held->second;
}

/// World rotation of every rig joint from parent-relative local rotations,
/// a joint missing from local is identity.
inline JointRotations WorldRotations(const Rig & rig, const JointRotations & local)
{
  std::unordered_set<std::string> known;
  for (const auto & joint : rig.joints)
    known.insert(joint.name);
  JointRotations world;
  for (std::size_t pass = 0; pass < rig.joints.size() && world.size() < rig.joints.size(); ++pass)
  {
    for (const auto & joint : rig.joints)
    {
      if (world.count(joint.name) > 0)
        continue;
      gz::math::Quaterniond parent = gz::math::Quaterniond::Identity;
      if (!joint.parent.empty() && known.count(joint.parent) > 0)
      {
        const auto found = world.find(joint.parent);
        if (found == world.end())
          continue;
        parent = found->second;
      }
      const auto own = local.find(joint.name);
      world.emplace(
        joint.name,
        (parent * (own == local.end() ? StanceLocal(rig, joint) : own->second)).Normalized());
    }
  }
  return world;
}

/// World rotation of every bone the rig maps, from joint world rotations.
inline JointRotations BoneWorldRotations(const Rig & rig, const JointRotations & world)
{
  JointRotations bones;
  for (const auto & joint : rig.joints)
  {
    const auto own = world.find(joint.name);
    if (own == world.end() || joint.bones.empty())
      continue;
    if (joint.bones.size() == 1)
    {
      bones[joint.bones.front().bone] = (own->second * joint.bones.front().offset).Normalized();
      continue;
    }
    gz::math::Quaterniond parent = gz::math::Quaterniond::Identity;
    const auto up = world.find(joint.parent);
    if (!joint.parent.empty() && up != world.end())
      parent = up->second;
    const gz::math::Quaterniond turn = (parent.Inverse() * own->second).Normalized();
    double cumulative = 0.0;
    for (const auto & bone : joint.bones)
    {
      cumulative += bone.weight;
      bones[bone.bone] = (parent * QuatPower(turn, cumulative) * bone.offset).Normalized();
    }
  }
  return bones;
}

/// Local pose of every skeleton bone, parents first. A driven bone takes its
/// neutral rotation composed with its delta, a mapped bone the rotation that
/// gives it the mapped world rotation, any other bone its neutral rotation.
/// topBone shifts by offset, given in the skeleton frame.
inline std::vector<gz::math::Pose3d> PoseBones(
  const std::vector<std::string> & names, const std::vector<int> & parents,
  const std::vector<gz::math::Pose3d> & neutral, const JointRotations & mapped,
  const JointRotations & driven, const std::string & topBone,
  const gz::math::Vector3d & offset)
{
  std::vector<gz::math::Quaterniond> world(names.size());
  std::vector<gz::math::Pose3d> pose(names.size());
  for (std::size_t b = 0; b < names.size(); ++b)
  {
    const gz::math::Quaterniond parent =
      parents[b] < 0 ? gz::math::Quaterniond::Identity : world[parents[b]];
    gz::math::Quaterniond local = neutral[b].Rot();
    const auto delta = driven.find(names[b]);
    const auto map = mapped.find(names[b]);
    if (delta != driven.end())
      local = (neutral[b].Rot() * delta->second).Normalized();
    else if (map != mapped.end())
      local = (parent.Inverse() * map->second).Normalized();
    gz::math::Vector3d position = neutral[b].Pos();
    if (!topBone.empty() && names[b] == topBone)
      position += parent.Inverse().RotateVector(offset);
    pose[b] = gz::math::Pose3d(position, local);
    world[b] = (parent * local).Normalized();
  }
  return pose;
}

}  // namespace arena_gz_plugins
