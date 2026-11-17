#include <autonomous_flight/simulation/dynamicExploration.h>
#include <global_planner/cameraGainEvaluator.h>
#include <chrono>
#include <iomanip>
#include <sstream>
namespace AutoFlight {
void dynamicExploration::turnEvent(const std::string& reason,double ms,size_t reference,double retained){
  std::ostringstream s;s<<std::setprecision(10)
    <<"{\"mode\":\""<<turnMode_<<"\",\"reason\":\""<<reason
    <<"\",\"sim_time\":"<<ros::Time::now().toSec()<<",\"global_sequence\":"<<activeGlobalSequence_
    <<",\"decision_ms\":"<<ms<<",\"reference_count\":"<<reference<<",\"retained\":"<<retained
    <<",\"turn_duration\":"<<turnSchedule_.duration<<",\"delay\":"<<turnSchedule_.delay
    <<",\"turn_origin\":["<<turnOrigin_.x()<<","<<turnOrigin_.y()<<","<<turnOrigin_.z()<<"]"
    <<",\"position\":["<<odom_.pose.pose.position.x<<","<<odom_.pose.pose.position.y<<","<<odom_.pose.pose.position.z<<"]}";
  ROS_INFO_STREAM("[TurnSync] "<<s.str());
}
void dynamicExploration::requestExplorationTurn(double yaw,bool hasNext){
  turnPending_=false;turnActive_=false;
  const auto& p=odom_.pose.pose.position;turnOrigin_=Eigen::Vector3d(p.x,p.y,p.z);
  turnTarget_=yaw;
  turnSchedule_=TurnSchedule::make(AutoFlight::rpy_from_quaternion(odom_.pose.pose.orientation),yaw,desiredAngularVel_);
  if(turnMode_=="serial" || returningHome_ || !hasNext || currVel_.norm()>.1 ||
     std::abs(turnSchedule_.delta)<.1 || depthRecoveryEnabled()){
    if(turnMode_!="serial")turnEvent("serial_ineligible");
    moveToOrientation(yaw,desiredAngularVel_);return;
  }
  // Keep the position target at the captured origin while the local curve is prepared.
  tracking_controller::Target hold;
  hold.position.x=turnOrigin_.x();hold.position.y=turnOrigin_.y();hold.position.z=turnOrigin_.z();hold.yaw=turnSchedule_.startYaw;
  updateTargetWithState(hold);
  turnPending_=true;
  turnEvent("requested");
}
bool dynamicExploration::configureTurnSync(trajPlanner::bspline curve){
  using Clock=std::chrono::steady_clock;
  const auto begin=Clock::now(),deadline=begin+std::chrono::milliseconds(500);
  auto elapsed=[&](){return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();};
  auto check=[&](){if(Clock::now()>=deadline)throw std::runtime_error("budget_exhausted");};
  size_t referenceCount=0;double retention=0;
  try {
    const auto map=map_->captureSnapshot();check();
    const double factor=bsplineTraj_->getLinearFactor();
    if(!std::isfinite(factor)||factor<=0)throw std::runtime_error("invalid_time_scale");
    const double flight=curve.getDuration()/factor;
    auto velocity=curve.getDerivative();auto acceleration=velocity.getDerivative();
    if(flight<=0 || !std::isfinite(flight))throw std::runtime_error("invalid_curve");
    const double gap=(curve.at(0)-turnOrigin_).norm();
    const auto& current=odom_.pose.pose.position;
    const Eigen::Vector3d currentPosition(current.x,current.y,current.z);
    ROS_INFO_STREAM("[TurnSyncDrift] requested_to_current="<<(currentPosition-turnOrigin_).norm()
        <<" curve_to_current="<<(curve.at(0)-currentPosition).norm());

    const double startSpeed=velocity.at(0).norm()*factor;
    const double startAcceleration=acceleration.at(0).norm()*factor*factor;
    ROS_INFO_STREAM("[TurnSyncGeometry] {\"sim_time\":"<<ros::Time::now().toSec()
       <<",\"gap\":"<<gap<<",\"initial_speed\":"<<startSpeed<<",\"initial_acceleration\":"<<startAcceleration<<"}");
    if(gap>.03)throw std::runtime_error("curve_start_position");
    double marginXY=.3,marginZ=0;
    nh_.param("DEP/safe_distance_xy",marginXY,.3);nh_.param("DEP/safe_distance_z",marginZ,0.);
    FrozenTurnSafety safety(map.occupancy,map.inflated,map.mapMin,map.dimensions,map.resolution,map.pMinLog,map.pOccLog);
    auto safe=[&](const Eigen::Vector3d& p){return safety.safe(p,marginXY,marginZ);};
    std::vector<Eigen::Vector3d> op,ov,os;
    if(useFakeDetector_)getDynamicObstacles(op,ov,os);else map_->getDynamicObstacles(op,ov,os);
    auto dynamicSafe=[&](const Eigen::Vector3d& p,double t){
      for(size_t i=0;i<op.size();++i){
        const Eigen::Vector3d half=os[i]*.5+Eigen::Vector3d(.25,.25,.15);
        if(((p-op[i]-t*ov[i]).cwiseAbs().array()<=half.array()).all())return false;
      }
      return true;
    };
    // Both arms use the same map/safety and finite candidate pool.
    std::vector<double> feasible;
    const double step=std::min(.02,map.resolution/(2*std::max(.1,desiredVel_)));
    for(double fraction:{0.,.5}){
      const double delay=fraction*turnSchedule_.duration;
      if(!delayBoundaryValid(delay,startSpeed,startAcceleration)){
        turnEvent("candidate_delayed_nonrest",elapsed());continue;
      }
      bool valid=true;Eigen::Vector3d previous=turnOrigin_;
      const double horizon=std::max(turnSchedule_.duration,delay+flight);
      const int n=int(std::ceil(horizon/step));
      for(int j=0;j<=n;++j){
        check();const double t=horizon*j/n;
        const double u=std::min(curve.getDuration(),std::max(0.,t-delay)*factor);
        const Eigen::Vector3d p=curve.at(u);
        if((p-previous).norm()>map.resolution*.5+1e-6 || !safe(p) || !dynamicSafe(p,t) ||
           velocity.at(u).norm()*factor>desiredVel_+1e-3 ||
           acceleration.at(u).norm()*factor*factor>desiredAcc_+1e-3){valid=false;break;}
        previous=p;
      }
      if(valid)feasible.push_back(delay);
    }
    if(feasible.empty())throw std::runtime_error("no_safe_curve");
    globalPlanner::CameraGainConfig config;
    std::unique_ptr<globalPlanner::CameraGainEvaluator> evaluator;
    globalPlanner::CameraSharedBudget budget;budget.deadline=deadline;budget.maxSamples=2048;
    using Set=globalPlanner::PathGainEvaluator::AddressSet;Set reference;
    const int samples=std::max(1,int(std::ceil(turnSchedule_.duration/.1)));
    auto poses=[&](bool stationary,double delay){
      std::vector<globalPlanner::PathGainWaypoint> out;
      for(int j=0;j<=samples;++j){
        const double t=turnSchedule_.duration*j/samples;
        globalPlanner::PathGainWaypoint pose;
        pose.position=stationary?turnOrigin_:Eigen::Vector3d(curve.at(std::min(curve.getDuration(),std::max(0.,t-delay)*factor)));
        pose.yaw=turnSchedule_.yaw(t);out.push_back(pose);
      }
      return out;
    };
    auto visible=[&](const std::vector<globalPlanner::PathGainWaypoint>& p){
      auto result=evaluator->evaluatePoseSets(p,map.version,budget,0);
      if(!result.valid)throw std::runtime_error(result.failureReason);
      Set total;for(const auto& part:result.poseSets)total.insert(part.begin(),part.end());
      check();return total;
    };
    if(turnMode_=="observation"){
      std::vector<double> intrinsics,matrix,pmin,pmax;
      if(!nh_.getParam("dynamic_map/depth_intrinsics",intrinsics)||intrinsics.size()!=4 ||
         !nh_.getParam("dynamic_map/body_to_camera",matrix)||matrix.size()!=16 ||
         !nh_.getParam("DEP/global_region_min",pmin)||pmin.size()!=3 ||
         !nh_.getParam("DEP/global_region_max",pmax)||pmax.size()!=3)throw std::runtime_error("camera_config_missing");
      config.fx=intrinsics[0];config.fy=intrinsics[1];config.cx=intrinsics[2];config.cy=intrinsics[3];
      nh_.getParam("dynamic_map/image_cols",config.cols);nh_.getParam("dynamic_map/image_rows",config.rows);
      nh_.getParam("dynamic_map/depth_filter_margin",config.margin);nh_.getParam("dynamic_map/depth_skip_pixel",config.pixelSkip);
      nh_.getParam("dynamic_map/raycast_max_length",config.range);
      for(int a=0;a<3;++a){config.planningMin[a]=pmin[a];config.planningMax[a]=pmax[a];}
      for(int a=0;a<4;++a)for(int b=0;b<4;++b)config.bodyToCamera(a,b)=matrix[4*a+b];
      evaluator.reset(new globalPlanner::CameraGainEvaluator(map,config));check();
      reference=visible(poses(true,0));referenceCount=reference.size();
    }
    for(double delay:feasible){
      retention=turnMode_=="observation"?retainedFraction(reference,visible(poses(false,delay))):1.;
      if(retention<.98)continue;
      check();turnSchedule_.delay=delay;turnPending_=false;turnActive_=true;
      turnEvent("accepted",elapsed(),referenceCount,retention);return true;
    }
    throw std::runtime_error("observation_loss");
  }catch(const std::exception& e){
    // Fixed internal error strings only (no untrusted string in emitted JSON).
    turnEvent(std::string("fallback_")+e.what(),elapsed(),referenceCount,retention);
    return false;
  }
}
}
