#include <gtest/gtest.h>

#include <autonomous_flight/simulation/depthAngularObserver.h>
#include <Eigen/Geometry>

TEST(DepthAngularObserver, RequiresStableMappedSurfaceBeforeCallingInvalidRayFault) {
  cv::Mat image(16, 16, CV_32FC1, cv::Scalar(3.0f));
  image.col(8).setTo(cv::Scalar(0.0f));
  mapManager::OccupancyMapSnapshot map;
  map.resolution = 1.0;
  map.mapMin = Eigen::Vector3d::Zero();
  map.dimensions = Eigen::Vector3i(10, 10, 10);
  map.pMinLog = -2.0;
  map.pOccLog = 2.0;
  map.occupancy.assign(1000, 0.0);
  map.validHitFrameCount.assign(1000, 0);
  for (int y = 0; y < 10; ++y) {
    for (int z = 0; z < 10; ++z) {
      const int address = 4 * 100 + y * 10 + z;
      map.occupancy[address] = 3.0;
      map.validHitFrameCount[address] = 3;
    }
  }
  Eigen::Matrix4d cameraToWorld = Eigen::Matrix4d::Identity();
  cameraToWorld.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  cameraToWorld.block<3, 1>(0, 3) = Eigen::Vector3d(1.0, 5.0, 5.0);
  AutoFlight::DepthIntrinsics camera;
  camera.fx = camera.fy = 100.0;
  camera.cx = camera.cy = 8.0;
  const auto evidence = AutoFlight::diagnoseAngularDepthFrame(
      image, cameraToWorld, camera, map, 16);
  EXPECT_GE(evidence[8].expectedStableSurfaceRays, 8);
  EXPECT_EQ(evidence[8].validExpectedSurfaceRays, 0);
  EXPECT_GE(evidence[7].validExpectedSurfaceRays, 8);

  std::fill(map.validHitFrameCount.begin(), map.validHitFrameCount.end(), 0);
  const auto withoutReference = AutoFlight::diagnoseAngularDepthFrame(
      image, cameraToWorld, camera, map, 16);
  EXPECT_EQ(withoutReference[8].expectedStableSurfaceRays, 0);
  EXPECT_EQ(withoutReference[8].validRays, 0);
}
