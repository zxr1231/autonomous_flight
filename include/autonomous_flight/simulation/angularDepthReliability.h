#ifndef AUTOFLIGHT_ANGULAR_DEPTH_RELIABILITY_H
#define AUTOFLIGHT_ANGULAR_DEPTH_RELIABILITY_H

#include <boost/math/special_functions/beta.hpp>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

namespace AutoFlight {

struct AngularDepthFrameEvidence {
  int expectedStableSurfaceRays = 0;
  int validExpectedSurfaceRays = 0;
  int validRays = 0;
};

enum class AngularReliabilityState { Unknown, Healthy, Faulty };

// One frame contributes at most one Bernoulli trial per camera angular bin.
// Missing returns without a stable mapped surface are not evidence of failure.
class AngularDepthReliability {
 public:
  AngularDepthReliability(std::size_t bins = 16, std::size_t windowFrames = 60,
                          std::size_t minimumDiagnosticFrames = 8,
                          double healthyLowerBound = 0.5)
      : history_(bins), windowFrames_(windowFrames),
        minimumDiagnosticFrames_(minimumDiagnosticFrames),
        healthyLowerBound_(healthyLowerBound) {
    if (bins == 0 || windowFrames == 0 || minimumDiagnosticFrames == 0 ||
        healthyLowerBound <= 0.0 || healthyLowerBound >= 1.0)
      throw std::invalid_argument("invalid angular reliability parameters");
  }

  std::size_t binCount() const { return history_.size(); }

  void observe(const std::vector<AngularDepthFrameEvidence>& frame) {
    if (frame.size() != history_.size())
      throw std::invalid_argument("angular reliability bin count mismatch");
    for (std::size_t bin = 0; bin < frame.size(); ++bin) {
      const auto& item = frame[bin];
      if (item.expectedStableSurfaceRays < 0 || item.validExpectedSurfaceRays < 0 ||
          item.validRays < 0 ||
          item.validExpectedSurfaceRays > item.expectedStableSurfaceRays)
        throw std::invalid_argument("invalid angular frame evidence count");
      int trial = -1;  // Abstain if the frame cannot diagnose this bin.
      if (item.expectedStableSurfaceRays >= 8) {
        const double fraction = static_cast<double>(item.validExpectedSurfaceRays) /
                                item.expectedStableSurfaceRays;
        if (fraction >= 0.5) trial = 1;
        else if (fraction <= 0.1) trial = 0;
      } else if (item.validRays >= 8) {
        trial = 1;
      }
      auto& values = history_[bin];
      values.push_back(trial);
      if (values.size() > windowFrames_) values.pop_front();
    }
  }

  std::size_t diagnosticFrames(std::size_t bin) const {
    std::size_t count = 0;
    for (int trial : history_.at(bin)) if (trial >= 0) ++count;
    return count;
  }

  double lowerBound(std::size_t bin) const {
    const auto counts = successesAndFailures(bin);
    return boost::math::ibeta_inv(1.0 + counts.first, 1.0 + counts.second, 0.05);
  }

  AngularReliabilityState state(std::size_t bin) const {
    const auto counts = successesAndFailures(bin);
    if (counts.first + counts.second < minimumDiagnosticFrames_)
      return AngularReliabilityState::Unknown;
    if (lowerBound(bin) >= healthyLowerBound_)
      return AngularReliabilityState::Healthy;
    const double upper = boost::math::ibeta_inv(
        1.0 + counts.first, 1.0 + counts.second, 0.95);
    if (upper < healthyLowerBound_)
      return AngularReliabilityState::Faulty;
    return AngularReliabilityState::Unknown;
  }

 private:
  std::pair<std::size_t, std::size_t> successesAndFailures(std::size_t bin) const {
    std::size_t successes = 0, failures = 0;
    for (int trial : history_.at(bin)) {
      if (trial == 1) ++successes;
      else if (trial == 0) ++failures;
    }
    return {successes, failures};
  }

  std::vector<std::deque<int>> history_;
  std::size_t windowFrames_;
  std::size_t minimumDiagnosticFrames_;
  double healthyLowerBound_;
};

}  // namespace AutoFlight

#endif
