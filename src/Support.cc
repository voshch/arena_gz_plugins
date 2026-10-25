#include "arena_gz_plugins/Support.hh"

#include <algorithm>
#include <filesystem>

#include <gz/math/Helpers.hh>
#include <yaml-cpp/yaml.h>

namespace arena_gz_plugins
{

PelvisClearance ReadPelvisClearance(const std::string & sdfPath)
{
  PelvisClearance clearance;
  try
  {
    const YAML::Node annotation =
      YAML::LoadFile((std::filesystem::path(sdfPath).parent_path() / "annotation.yaml").string());
    const YAML::Node measured = annotation["pelvis_clearance"];
    if (measured)
    {
      clearance.sit = measured["sit"].as<double>();
      clearance.lie = measured["lie"].as<double>();
    }
  }
  catch (const YAML::Exception &)
  {
    return PelvisClearance{};
  }
  return clearance;
}

gz::math::Pose3d SupportedPose(
  std::uint8_t kind, double height, const PelvisClearance & clearance,
  const gz::math::Vector3d & spot, const gz::math::Quaterniond & orientation,
  const gz::math::Vector3d & pelvis, double groundZ)
{
  if (kind == kLieOn)
  {
    // On its back: local +x (facing) up, local +z (head) to -x, so the head
    // points away from the facing and the feet toward it, the pelvis over the spot.
    const gz::math::Quaterniond supine(gz::math::Vector3d::UnitY, -GZ_PI / 2.0);
    const gz::math::Quaterniond lying = orientation * supine;
    const gz::math::Vector3d target(spot.X(), spot.Y(), spot.Z() + height + clearance.lie);
    return gz::math::Pose3d(target - lying.RotateVector(pelvis), lying);
  }
  // Seated: the pelvis rests on the seat, but the feet never go below the
  // floor. While the clip still folds the legs the body stays up and lowers
  // onto the seat as they fold; on a seat higher than the folded legs reach,
  // the feet hang.
  const double z = std::max(height + clearance.sit - pelvis.Z(), groundZ);
  return gz::math::Pose3d(gz::math::Vector3d(spot.X(), spot.Y(), spot.Z() + z), orientation);
}

}  // namespace arena_gz_plugins
