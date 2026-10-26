#ifndef AUTOFLIGHT_DEPTH_ANGULAR_OBSERVER_H
#define AUTOFLIGHT_DEPTH_ANGULAR_OBSERVER_H

#include <autonomous_flight/simulation/angularDepthReliability.h>
#include <map_manager/occupancyMap.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace AutoFlight {

struct DepthIntrinsics {
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  double minimum = 0.5, maximum = 5.0;
};

// Samples a bounded set of raw rays per angular bin.  An invalid ray counts
// as a fault trial only when a previously supported occupied surface is
// expected in this map snapshot.  Unmapped/open directions abstain.
inline std::vector<AngularDepthFrameEvidence> diagnoseAngularDepthFrame(
    const cv::Mat& rawDepth, const Eigen::Matrix4d& cameraToWorld,
    const DepthIntrinsics& camera,
    const mapManager::OccupancyMapSnapshot& map,
    std::size_t bins = 16,
    const std::function<bool(const Eigen::Vector3d&)>& dynamicReference = {}) {
  if (rawDepth.type() != CV_32FC1 || bins == 0 || camera.fx <= 0.0 ||
      camera.fy <= 0.0 || camera.minimum <= 0.0 ||
      camera.maximum <= camera.minimum || map.resolution <= 0.0)
    throw std::invalid_argument("invalid angular depth observer input");
  std::vector<AngularDepthFrameEvidence> result(bins);
  const Eigen::Vector3d origin = cameraToWorld.block<3, 1>(0, 3);
  const Eigen::Matrix3d rotation = cameraToWorld.block<3, 3>(0, 0);
  const Eigen::Vector3i dimensions = map.dimensions;
  const std::size_t volume = static_cast<std::size_t>(dimensions.x()) *
                             dimensions.y() * dimensions.z();
  if (map.occupancy.size() != volume || map.validHitFrameCount.size() != volume)
    throw std::invalid_argument("angular observer needs a valid-hit map snapshot");

  for (std::size_t bin = 0; bin < bins; ++bin) {
    const int left = static_cast<int>(bin * rawDepth.cols / bins);
    const int right = static_cast<int>((bin + 1) * rawDepth.cols / bins);
    if (left >= right) continue;
    auto& evidence = result[bin];
    for (int rowSample = 0; rowSample < 6; ++rowSample) {
      const int v = std::min(rawDepth.rows - 1,
          (2 * rowSample + 1) * rawDepth.rows / 12);
      for (int columnSample = 0; columnSample < 6; ++columnSample) {
        const int u = std::min(right - 1,
            left + (2 * columnSample + 1) * (right - left) / 12);
        const float depth = rawDepth.at<float>(v, u);
        const bool valid = mapManager::validDepthHit(
            depth, camera.minimum, camera.maximum);
        if (valid) ++evidence.validRays;

        Eigen::Vector3d ray((u - camera.cx) / camera.fx,
                            (v - camera.cy) / camera.fy, 1.0);
        const Eigen::Vector3d direction = rotation * ray.normalized();
        for (double distance = camera.minimum; distance < camera.maximum;
             distance += map.resolution) {
          const Eigen::Vector3d point = origin + distance * direction;
          const Eigen::Vector3i index = ((point - map.mapMin) /
                                           map.resolution).array().floor().cast<int>();
          if ((index.array() < 0).any() ||
              (index.array() >= dimensions.array()).any()) break;
          const std::size_t address = static_cast<std::size_t>(index.x()) *
              dimensions.y() * dimensions.z() +
              static_cast<std::size_t>(index.y()) * dimensions.z() + index.z();
          const double occupancy = map.occupancy[address];
          if (occupancy < map.pMinLog) break;
          if (occupancy >= map.pOccLog) {
            if (map.validHitFrameCount[address] >= 3 &&
                (!dynamicReference || !dynamicReference(point))) {
              ++evidence.expectedStableSurfaceRays;
              if (valid) ++evidence.validExpectedSurfaceRays;
            }
            break;
          }
        }
      }
    }
  }
  return result;
}

}  // namespace AutoFlight

#endif
