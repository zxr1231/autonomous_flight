#ifndef AUTOFLIGHT_EVIDENCE_OBLIGATION_H
#define AUTOFLIGHT_EVIDENCE_OBLIGATION_H

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace AutoFlight {

struct EvidenceObligation {
  bool found = false;
  Eigen::Vector3d pathPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d missingEvidencePosition = Eigen::Vector3d::Zero();
  double distanceAlongPath = 0.0;
  std::size_t checkedBodySamples = 0;
};

// Searches the actual output path for its first sampled robot-body point
// without trustworthy free-space evidence.  The caller provides the map
// predicate, including the separately documented takeoff-safe prior.
inline EvidenceObligation firstMissingBodyEvidence(
    const std::vector<Eigen::Vector3d>& path,
    const Eigen::Vector3d& bodyHalfExtent,
    double maxTravel, double spatialStep,
    const std::function<bool(const Eigen::Vector3d&)>& hasSafeEvidence) {
  if (maxTravel <= 0.0 || spatialStep <= 0.0 ||
      (bodyHalfExtent.array() < 0.0).any()) {
    throw std::invalid_argument("invalid evidence obligation geometry");
  }
  EvidenceObligation result;
  if (path.empty()) return result;

  auto inspect = [&](const Eigen::Vector3d& center, double distance) {
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          Eigen::Vector3d point = center + Eigen::Vector3d(
              x * bodyHalfExtent.x(), y * bodyHalfExtent.y(),
              z * bodyHalfExtent.z());
          ++result.checkedBodySamples;
          if (!hasSafeEvidence(point)) {
            result.found = true;
            result.pathPosition = center;
            result.missingEvidencePosition = point;
            result.distanceAlongPath = distance;
            return true;
          }
        }
      }
    }
    return false;
  };

  if (inspect(path.front(), 0.0)) return result;
  double traveled = 0.0;
  for (std::size_t segment = 1; segment < path.size() && traveled < maxTravel;
       ++segment) {
    const Eigen::Vector3d displacement = path[segment] - path[segment - 1];
    const double length = displacement.norm();
    if (length <= 1e-9) continue;
    const double limit = std::min(length, maxTravel - traveled);
    const int steps = std::max(1, static_cast<int>(std::ceil(limit / spatialStep)));
    for (int step = 1; step <= steps; ++step) {
      const double local = limit * step / steps;
      const Eigen::Vector3d center = path[segment - 1] +
                                     displacement * (local / length);
      if (inspect(center, traveled + local)) return result;
    }
    traveled += limit;
  }
  return result;
}

}  // namespace AutoFlight

#endif
