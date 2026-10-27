#ifndef AUTOFLIGHT_OBSERVATION_RECOVERY_SELECTOR_H
#define AUTOFLIGHT_OBSERVATION_RECOVERY_SELECTOR_H

#include <autonomous_flight/simulation/angularDepthReliability.h>
#include <autonomous_flight/simulation/depthAngularObserver.h>
#include <map_manager/occupancyMap.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace AutoFlight {

struct ObservationRecoveryAction {
  bool found = false;
  Eigen::Vector3d bodyPosition = Eigen::Vector3d::Zero();
  double bodyYaw = 0.0;
  std::size_t cameraBin = 0;
  double estimatedTime = std::numeric_limits<double>::infinity();
  std::size_t inspectedCandidates = 0;
};

inline double shortestAngleDifference(double from, double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

inline bool rayCanObserveObligation(
    const Eigen::Matrix4d& cameraToWorld,
    const Eigen::Vector3d& obligation,
    const DepthIntrinsics& camera,
    const mapManager::OccupancyMapSnapshot& map,
    std::size_t& cameraBin) {
  const Eigen::Vector3d origin = cameraToWorld.block<3, 1>(0, 3);
  const Eigen::Vector3d local =
      cameraToWorld.block<3, 3>(0, 0).transpose() * (obligation - origin);
  if (local.z() < camera.minimum || local.z() >= camera.maximum) return false;
  const double u = camera.fx * local.x() / local.z() + camera.cx;
  const double v = camera.fy * local.y() / local.z() + camera.cy;
  if (u < 0.0 || u >= 640.0 || v < 0.0 || v >= 480.0) return false;
  cameraBin = std::min<std::size_t>(15, static_cast<std::size_t>(u / 40.0));

  const Eigen::Vector3d direction = (obligation - origin).normalized();
  for (double distance = camera.minimum; distance + map.resolution <
       (obligation - origin).norm(); distance += map.resolution) {
    const Eigen::Vector3d point = origin + distance * direction;
    const Eigen::Vector3i idx = ((point - map.mapMin) / map.resolution)
                                    .array().floor().cast<int>();
    if ((idx.array() < 0).any() ||
        (idx.array() >= map.dimensions.array()).any()) return false;
    const std::size_t address = static_cast<std::size_t>(idx.x()) *
        map.dimensions.y() * map.dimensions.z() +
        static_cast<std::size_t>(idx.y()) * map.dimensions.z() + idx.z();
    if (address >= map.occupancy.size() ||
        map.occupancy[address] < map.pMinLog ||
        map.occupancy[address] >= map.pOccLog) return false;
  }
  return true;
}

// Fixed 5 positions x 7 relative yaws = at most 35 candidates.  E uses
// uniformReliability=true; F uses the measured angular model.  The caller
// must supply a known-free, body-valid connection predicate and must verify
// the final B-spline before executing any translational action.
inline ObservationRecoveryAction selectObservationRecovery(
    const Eigen::Vector3d& currentPosition, double currentYaw,
    const Eigen::Matrix4d& bodyToCamera,
    const Eigen::Vector3d& obligation,
    const DepthIntrinsics& camera,
    const mapManager::OccupancyMapSnapshot& map,
    const AngularDepthReliability& reliability,
    bool uniformReliability,
    const std::function<bool(const Eigen::Vector3d&)>& safeConnection,
    const std::function<bool(const Eigen::Vector3d&, double)>& alreadyTried = {}) {
  ObservationRecoveryAction best;
  const Eigen::Vector3d offsets[] = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.5, 0.0),
      Eigen::Vector3d(0.0, -0.5, 0.0)};
  const int degrees[] = {-90, -60, -30, 0, 30, 60, 90};
  for (const auto& offset : offsets) {
    const Eigen::Vector3d bodyPosition = currentPosition + offset;
    const bool connectionSafe = safeConnection(bodyPosition);
    for (int degree : degrees) {
      ++best.inspectedCandidates;
      if (!connectionSafe) continue;
      const double yaw = currentYaw + degree * M_PI / 180.0;
      if (alreadyTried && alreadyTried(bodyPosition, yaw)) continue;
      Eigen::Matrix4d bodyToWorld = Eigen::Matrix4d::Identity();
      bodyToWorld.block<3, 3>(0, 0) =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      bodyToWorld.block<3, 1>(0, 3) = bodyPosition;
      const Eigen::Matrix4d cameraToWorld = bodyToWorld * bodyToCamera;
      std::size_t bin = 0;
      if (!rayCanObserveObligation(cameraToWorld, obligation, camera, map, bin))
        continue;
      if (!uniformReliability &&
          reliability.state(bin) != AngularReliabilityState::Healthy)
        continue;
      const double cost = offset.norm() / 1.0 +
          std::abs(shortestAngleDifference(currentYaw, yaw)) / 0.5;
      if (cost < best.estimatedTime) {
        best.found = true;
        best.bodyPosition = bodyPosition;
        best.bodyYaw = yaw;
        best.cameraBin = bin;
        best.estimatedTime = cost;
      }
    }
  }
  return best;
}

}  // namespace AutoFlight

#endif
