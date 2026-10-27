#include <gtest/gtest.h>

#include <autonomous_flight/simulation/observationRecoverySelector.h>

TEST(ObservationRecoverySelector, AvoidsFaultyCameraBinUnderSameCandidateBudget) {
  AutoFlight::AngularDepthReliability reliability(16);
  std::vector<AutoFlight::AngularDepthFrameEvidence> frame(16);
  for (auto& item : frame) item = {10, 10, 10};
  frame[8] = {10, 0, 0};
  for (int iteration = 0; iteration < 40; ++iteration) reliability.observe(frame);

  mapManager::OccupancyMapSnapshot map;
  map.resolution = 0.1;
  map.mapMin = Eigen::Vector3d(-5.0, -5.0, 0.0);
  map.dimensions = Eigen::Vector3i(100, 100, 30);
  map.pMinLog = -2.0;
  map.pOccLog = 2.0;
  map.occupancy.assign(100 * 100 * 30, 0.0);
  Eigen::Matrix4d bodyToCamera = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d cameraRotation;
  cameraRotation << 0.0, 0.0, 1.0,
                   -1.0, 0.0, 0.0,
                    0.0,-1.0, 0.0;
  bodyToCamera.block<3, 3>(0, 0) = cameraRotation;
  AutoFlight::DepthIntrinsics camera;
  camera.fx = camera.fy = 554.254691191187;
  camera.cx = 320.5;
  camera.cy = 240.5;
  const Eigen::Vector3d current(0.0, 0.0, 1.0);
  const Eigen::Vector3d target(3.0, 0.0, 1.0);
  const auto safe = [](const Eigen::Vector3d&) { return true; };
  const auto geometry = AutoFlight::selectObservationRecovery(
      current, 0.0, bodyToCamera, target, camera, map, reliability, true, safe);
  const auto faultAware = AutoFlight::selectObservationRecovery(
      current, 0.0, bodyToCamera, target, camera, map, reliability, false, safe);
  ASSERT_TRUE(geometry.found);
  ASSERT_TRUE(faultAware.found);
  EXPECT_EQ(geometry.inspectedCandidates, 35u);
  EXPECT_EQ(faultAware.inspectedCandidates, 35u);
  EXPECT_NE(geometry.cameraBin, faultAware.cameraBin);
  EXPECT_NE(reliability.state(faultAware.cameraBin),
            AutoFlight::AngularReliabilityState::Faulty);
  const auto retry = AutoFlight::selectObservationRecovery(
      current, 0.0, bodyToCamera, target, camera, map, reliability, false, safe,
      [&](const Eigen::Vector3d& position, double yaw) {
        return (position - faultAware.bodyPosition).norm() < 0.05 &&
               std::abs(AutoFlight::shortestAngleDifference(
                   yaw, faultAware.bodyYaw)) < 0.05;
      });
  ASSERT_TRUE(retry.found);
  EXPECT_FALSE((retry.bodyPosition - faultAware.bodyPosition).norm() < 0.05 &&
               std::abs(AutoFlight::shortestAngleDifference(
                   retry.bodyYaw, faultAware.bodyYaw)) < 0.05);
}

TEST(ObservationRecoverySelector, RejectsUncertifiedConnection) {
  AutoFlight::AngularDepthReliability reliability(16);
  mapManager::OccupancyMapSnapshot map;
  map.resolution = 0.1;
  map.mapMin = Eigen::Vector3d(-5.0, -5.0, 0.0);
  map.dimensions = Eigen::Vector3i(100, 100, 30);
  map.pMinLog = -2.0;
  map.pOccLog = 2.0;
  map.occupancy.assign(100 * 100 * 30, 0.0);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  AutoFlight::DepthIntrinsics camera;
  camera.fx = camera.fy = 554.0;
  camera.cx = 320.5;
  camera.cy = 240.5;
  const auto result = AutoFlight::selectObservationRecovery(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.0, transform,
      Eigen::Vector3d(3.0, 0.0, 1.0), camera, map, reliability, true,
      [](const Eigen::Vector3d&) { return false; });
  EXPECT_FALSE(result.found);
}
