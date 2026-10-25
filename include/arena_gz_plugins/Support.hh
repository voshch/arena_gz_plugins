#ifndef ARENA_GZ_PLUGINS__SUPPORT_HH_
#define ARENA_GZ_PLUGINS__SUPPORT_HH_

/// Actors resting on furniture: sitting on a seat or lying on a bed.
///
/// The wire carries a floor position, a yaw and joint angles. A sit clip folds
/// the legs but keeps the hips at their standing height, and no clip tips a
/// whole body onto its back. While a ped holds a sit or lie interaction, the
/// adapter reports how high the surface under it is
/// (Pedestrian.support_height); SupportedPose places the actor so its pelvis
/// rests that character's measured clearance above the surface (annotation
/// `pelvis_clearance`, measured by the arena_assets human build). The same
/// placement as arena_isaac's peds/support.py.

#include <cstdint>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace arena_gz_plugins
{

/// humansim InteractionType values, as Pedestrian.interaction_type carries them.
constexpr std::uint8_t kSitOn = 3;
constexpr std::uint8_t kLieOn = 4;

/// The skeleton's root joint (cmu_mb rig), the bone the body rests on.
constexpr const char * kPelvisBone = "Hips";

/// Height of a character's pelvis joint above the surface it rests on, in m:
/// seated, and lying on its back. Defaults to the median over the 83
/// characters the arena_assets human build measured.
struct PelvisClearance
{
  double sit{0.122};
  double lie{0.117};
};

/// The clearance in the annotation beside a character's SDF, the typical one
/// when the asset was built without it or the annotation cannot be read.
PelvisClearance ReadPelvisClearance(const std::string & sdfPath);

/// Actor pose for a ped commanded to `spot` (x, y and its floor height) and
/// `orientation`, resting on a `kind` (kSitOn or kLieOn) surface `height` m
/// above its floor. `pelvis` is the pelvis position in the skeleton frame for
/// the evaluated bones, `groundZ` the actor z offset that puts the soles on
/// the floor (the feet never go below it while seated).
gz::math::Pose3d SupportedPose(
  std::uint8_t kind, double height, const PelvisClearance & clearance,
  const gz::math::Vector3d & spot, const gz::math::Quaterniond & orientation,
  const gz::math::Vector3d & pelvis, double groundZ);

}  // namespace arena_gz_plugins

#endif  // ARENA_GZ_PLUGINS__SUPPORT_HH_
