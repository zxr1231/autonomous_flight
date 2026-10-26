#include <gtest/gtest.h>

#include <autonomous_flight/simulation/evidenceObligation.h>

namespace {
std::vector<Eigen::Vector3d> straightPath() {
  return {Eigen::Vector3d(0.0, 0.0, 1.0),
          Eigen::Vector3d(2.0, 0.0, 1.0)};
}
}  // namespace

TEST(EvidenceObligation, FindsFirstMissingBodyVolumePoint) {
  const auto result = AutoFlight::firstMissingBodyEvidence(
      straightPath(), Eigen::Vector3d(0.25, 0.25, 0.15), 1.5, 0.05,
      [](const Eigen::Vector3d& point) { return point.x() < 1.0; });
  ASSERT_TRUE(result.found);
  EXPECT_LT(result.pathPosition.x(), 1.3);
  EXPECT_GE(result.missingEvidencePosition.x(), 1.0);
  EXPECT_GT(result.checkedBodySamples, 0u);
}

TEST(EvidenceObligation, RespectsLookaheadAndKnownFreePath) {
  const auto safe = [](const Eigen::Vector3d& point) { return point.x() < 1.8; };
  const auto shortResult = AutoFlight::firstMissingBodyEvidence(
      straightPath(), Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, 0.05, safe);
  EXPECT_FALSE(shortResult.found);
  const auto longResult = AutoFlight::firstMissingBodyEvidence(
      straightPath(), Eigen::Vector3d(0.0, 0.0, 0.0), 2.0, 0.05, safe);
  EXPECT_TRUE(longResult.found);
}

TEST(EvidenceObligation, EmptyPathAndInvalidGeometry) {
  EXPECT_FALSE(AutoFlight::firstMissingBodyEvidence(
      {}, Eigen::Vector3d::Zero(), 1.0, 0.05,
      [](const Eigen::Vector3d&) { return false; }).found);
  EXPECT_THROW(AutoFlight::firstMissingBodyEvidence(
      straightPath(), Eigen::Vector3d::Zero(), 1.0, 0.0,
      [](const Eigen::Vector3d&) { return true; }), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
