#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include <gz/math/Helpers.hh>

#include "arena_gz_plugins/Support.hh"

using arena_gz_plugins::kLieOn;
using arena_gz_plugins::kSitOn;
using arena_gz_plugins::PelvisClearance;
using arena_gz_plugins::ReadPelvisClearance;
using arena_gz_plugins::SupportedPose;
using gz::math::Pose3d;
using gz::math::Quaterniond;
using gz::math::Vector3d;

namespace
{

const PelvisClearance kClearance{0.1, 0.12};

Quaterniond Yaw(double yaw)
{
  return Quaterniond(0.0, 0.0, yaw);
}

void ExpectNear(const Vector3d & actual, const Vector3d & expected)
{
  EXPECT_NEAR(actual.X(), expected.X(), 1e-9);
  EXPECT_NEAR(actual.Y(), expected.Y(), 1e-9);
  EXPECT_NEAR(actual.Z(), expected.Z(), 1e-9);
}

}  // namespace

TEST(Support, SeatedPelvisRestsItsClearanceAboveTheSeat)
{
  // legs folded: the pelvis 0.45 m above the skeleton origin, soles 0.02 m below it
  const Vector3d pelvis(0.05, 0.0, 0.45);
  const Pose3d pose = SupportedPose(kSitOn, 0.4, kClearance, Vector3d(2.0, 1.0, 0.3), Yaw(0.7), pelvis, 0.02);
  ExpectNear(pose.Pos(), Vector3d(2.0, 1.0, 0.3 + 0.4 + 0.1 - 0.45));  // over its spot, yawed in place
  EXPECT_NEAR((pose.Pos() + pose.Rot().RotateVector(pelvis)).Z(), 0.3 + 0.4 + 0.1, 1e-9);
  EXPECT_EQ(pose.Rot(), Yaw(0.7));
}

TEST(Support, SeatedBodyNeverPushesItsFeetBelowTheFloor)
{
  // legs not yet folded: the pelvis 0.9 m up, the soles put on the floor by groundZ
  const Pose3d pose = SupportedPose(kSitOn, 0.4, kClearance, Vector3d(0.0, 0.0, 0.3), Yaw(0.0), Vector3d(0.0, 0.0, 0.9), 0.02);
  ExpectNear(pose.Pos(), Vector3d(0.0, 0.0, 0.3 + 0.02));
}

TEST(Support, LyingBodyIsOnItsBackOverItsSpotHeadAwayFromItsFacing)
{
  for (const double yaw : {0.0, GZ_PI / 2.0, -2.5})
  {
    const Vector3d pelvis(0.0, 0.0, 0.9);
    const Vector3d head(0.0, 0.0, 1.7);
    const Vector3d toe(0.2, 0.0, 0.0);
    const Pose3d pose = SupportedPose(kLieOn, 0.6, kClearance, Vector3d(3.0, -1.0, 0.2), Yaw(yaw), pelvis, 0.0);
    const auto world = [&pose](const Vector3d & point) {return pose.Pos() + pose.Rot().RotateVector(point);};
    const Vector3d facing(std::cos(yaw), std::sin(yaw), 0.0);
    ExpectNear(world(pelvis), Vector3d(3.0, -1.0, 0.2 + 0.6 + 0.12));
    ExpectNear(world(head) - world(pelvis), -0.8 * facing);  // the head beyond, away from the facing
    EXPECT_NEAR(world(toe).Z() - world(pelvis).Z(), 0.2, 1e-9);  // the toes point up
  }
}

TEST(Support, ClearanceComesFromTheAnnotationBesideTheSdf)
{
  const auto dir = std::filesystem::temp_directory_path() / "arena_gz_support_test";
  std::filesystem::create_directories(dir / "measured");
  std::filesystem::create_directories(dir / "bare");
  std::ofstream(dir / "measured" / "annotation.yaml") << "pelvis_clearance:\n  sit: 0.11\n  lie: 0.13\n";
  std::ofstream(dir / "bare" / "annotation.yaml") << "name: bare\n";

  const PelvisClearance measured = ReadPelvisClearance((dir / "measured" / "person.sdf").string());
  EXPECT_DOUBLE_EQ(measured.sit, 0.11);
  EXPECT_DOUBLE_EQ(measured.lie, 0.13);
  for (const char * name : {"bare", "missing"})
  {
    const PelvisClearance typical = ReadPelvisClearance((dir / name / "person.sdf").string());
    EXPECT_DOUBLE_EQ(typical.sit, PelvisClearance{}.sit);
    EXPECT_DOUBLE_EQ(typical.lie, PelvisClearance{}.lie);
  }
  std::filesystem::remove_all(dir);
}
