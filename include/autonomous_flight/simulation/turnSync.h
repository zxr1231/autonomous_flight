#ifndef AUTOFLIGHT_TURN_SYNC_H
#define AUTOFLIGHT_TURN_SYNC_H
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include <Eigen/Core>
namespace AutoFlight {
// Execution owns the accepted scale; candidate planner mutations must not reach it.
class CommittedTimeScale {
 double value_=1.;
 public:
 bool commit(double value){if(!std::isfinite(value)||value<=0)return false;value_=value;return true;}
 double value()const{return value_;}
 double time(double elapsed)const{return elapsed*value_;}
};

class FrozenTurnSafety {
  const std::vector<double>& occupancy_;
  const std::vector<bool>& inflated_;
  Eigen::Vector3d origin_; Eigen::Vector3i dims_;
  double resolution_, minimum_, occupied_;
 public:
  FrozenTurnSafety(const std::vector<double>& occupancy,const std::vector<bool>& inflated,
                   const Eigen::Vector3d& origin,const Eigen::Vector3i& dims,
                   double resolution,double minimum,double occupied):
    occupancy_(occupancy),inflated_(inflated),origin_(origin),dims_(dims),
    resolution_(resolution),minimum_(minimum),occupied_(occupied) {
    if(resolution<=0 || !std::isfinite(resolution) || (dims.array()<=0).any() ||
       occupancy.size()!=size_t(dims.x())*dims.y()*dims.z() || inflated.size()!=occupancy.size())
      throw std::invalid_argument("invalid_safety_map");
  }
  bool free(const Eigen::Vector3d& p)const {
    if(!p.allFinite())return false;
    const Eigen::Array3d q=((p-origin_)/resolution_).array().floor();
    if((q<0).any()||(q>=dims_.cast<double>().array()).any())return false;
    const Eigen::Vector3i i=q.cast<int>();
    const size_t a=size_t(i.x())*dims_.y()*dims_.z()+size_t(i.y())*dims_.z()+i.z();
    return !inflated_[a] && occupancy_[a]>=minimum_ && occupancy_[a]<occupied_;
  }
  bool safe(const Eigen::Vector3d& p,double xy,double z)const {
    for(double dx=-xy;dx<=xy+1e-8;dx+=resolution_)
      for(double dy=-xy;dy<=xy+1e-8;dy+=resolution_)
        for(double dz=-z;dz<=z+1e-8;dz+=resolution_)
          if(!free(p+Eigen::Vector3d(dx,dy,dz)))return false;
    return free(p);
  }
};
struct TurnSchedule {
  double startYaw=0,delta=0,duration=0,delay=0;
  static double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
  static TurnSchedule make(double from,double to,double maxRate){
    if(!std::isfinite(from)||!std::isfinite(to)||!std::isfinite(maxRate)||maxRate<=0)
      throw std::invalid_argument("invalid turn input");
    TurnSchedule s;s.startYaw=from;s.delta=wrap(to-from);
    s.duration=1.875*std::abs(s.delta)/maxRate;return s;
  }
  double yaw(double time) const {
    double u=duration>1e-9?std::max(0.,std::min(1.,time/duration)):1.;
    return wrap(startYaw+delta*(10*u*u*u-15*u*u*u*u+6*u*u*u*u*u));
  }
  double motionTime(double time)const{return std::max(0.,time-delay);}
};
inline bool delayBoundaryValid(double delay,double initialSpeed,double initialAcceleration) {
  if(!std::isfinite(delay)||!std::isfinite(initialSpeed)||!std::isfinite(initialAcceleration)||delay<0)return false;
  // Only a newly inserted hold requires rest on both sides of its boundary.
  return delay==0 || (initialSpeed<=.02 && initialAcceleration<=.05);
}
template<class Set> double retainedFraction(const Set& reference,const Set& observed){
  if(reference.empty())return 1.;
  size_t found=0;for(const auto& a:reference)found+=observed.count(a)!=0;
  return double(found)/reference.size();
}
}
#endif
