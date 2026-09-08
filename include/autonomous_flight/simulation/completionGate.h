#ifndef AUTONOMOUS_FLIGHT_COMPLETION_GATE_H
#define AUTONOMOUS_FLIGHT_COMPLETION_GATE_H
#include <algorithm>
#include <cstdint>

namespace AutoFlight {
// A planning failure or stale sensor cannot count as evidence of completion.
class CompletionGate {
 public:
  CompletionGate(int confirmations = 3, double duration = 5.0)
      : required_(std::max(2, confirmations)), duration_(std::max(0.0, duration)) {}
  bool observe(bool planValid, bool exhausted, bool sensorFresh,
               uint64_t sensorSequence, double now) {
    if (!planValid || !exhausted || !sensorFresh || (count_ && now < lastTime_)) {
      reset(); return false;
    }
    if (count_ && sensorSequence == sequence_) return false;
    if (!count_) firstTime_ = now;
    sequence_ = sensorSequence; lastTime_ = now; ++count_;
    return count_ >= required_ && now - firstTime_ >= duration_;
  }
  void reset() { count_ = 0; sequence_ = 0; }
 private:
  int required_, count_ = 0;
  double duration_, firstTime_ = 0, lastTime_ = 0;
  uint64_t sequence_ = 0;
};
}
#endif
