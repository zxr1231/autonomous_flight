/*
	FILE: dynamicExploration.cpp
	-----------------------------
	Implementation of dynamic exploration
*/

#include <autonomous_flight/simulation/dynamicExploration.h>
#include <autonomous_flight/simulation/depthAngularObserver.h>
#include <autonomous_flight/simulation/observationRecoverySelector.h>
#include <chrono>
#include <nav_msgs/GetPlan.h>
#include <iomanip>
#include <sstream>

namespace {
	double pathLength(const nav_msgs::Path& path){
		double length = 0.0;
		for (size_t i=1; i<path.poses.size(); ++i){
			const auto& a = path.poses[i-1].pose.position;
			const auto& b = path.poses[i].pose.position;
			const double dx = b.x-a.x, dy = b.y-a.y, dz = b.z-a.z;
			length += std::sqrt(dx*dx + dy*dy + dz*dz);
		}
		return length;
	}
	void appendPathPoints(std::ostringstream& json, const std::string& key,
						  const nav_msgs::Path& path){
		json << ",\"" << key << "\":[";
		for (size_t i=0; i<path.poses.size(); ++i){
			if (i) json << ",";
			const auto& p = path.poses[i].pose.position;
			json << "[" << p.x << "," << p.y << "," << p.z << "]";
		}
		json << "]";
	}
}

namespace AutoFlight{
	dynamicExploration::dynamicExploration(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::freeMapCB, this);
		}
	}

	void dynamicExploration::initParam(){
        nh_.param<std::string>("autonomous_flight/viewpoint_selector", viewpointSelector_, "original");
        if (viewpointSelector_ == "fuel")
            nextViewClient_ = nh_.serviceClient<nav_msgs::GetPlan>("/fuel_compare/next_view");
        else if (viewpointSelector_ != "original" && viewpointSelector_ != "cerlab")
            throw std::invalid_argument("unknown viewpoint selector");

        this->nh_.param<std::string>("autonomous_flight/turn_sync_mode", this->turnMode_, "serial");
        this->nh_.param("autonomous_flight/terminal_recovery", this->terminalRecoveryEnabled_, false);
        if (this->turnMode_ != "serial" && this->turnMode_ != "parallel" && this->turnMode_ != "observation")
            throw std::runtime_error("invalid turn_sync_mode");

		this->nh_.param<std::string>("autonomous_flight/depth_evidence/arm",
		                            this->depthEvidenceArm_, "");
		if (this->depthEvidenceArm_ != "" && this->depthEvidenceArm_ != "A" &&
		    this->depthEvidenceArm_ != "B" &&
		    this->depthEvidenceArm_ != "C" && this->depthEvidenceArm_ != "D" &&
		    this->depthEvidenceArm_ != "E" && this->depthEvidenceArm_ != "F" &&
		    this->depthEvidenceArm_ != "F_DIAGNOSTIC"){
			throw std::runtime_error("unknown V2 depth evidence arm");
		}
		if (this->depthRecoveryEnabled() || this->depthEvidenceArm_ == "F_DIAGNOSTIC"){
			std::vector<double> intrinsics;
			if (!this->nh_.getParam("/dynamic_map/depth_intrinsics", intrinsics) ||
			    intrinsics.size() != 4){
				throw std::runtime_error("V2 arm F requires /dynamic_map/depth_intrinsics");
			}
			this->depthFx_ = intrinsics[0]; this->depthFy_ = intrinsics[1];
			this->depthCx_ = intrinsics[2]; this->depthCy_ = intrinsics[3];
			if (this->depthEvidenceArm_ == "E" || this->depthEvidenceArm_ == "F"){
				std::vector<double> matrix;
				if (!this->nh_.getParam("/dynamic_map/body_to_camera", matrix) ||
				    matrix.size() != 16)
					throw std::runtime_error("V2 E/F require /dynamic_map/body_to_camera");
				for (int row=0; row<4; ++row)
					for (int col=0; col<4; ++col)
						this->depthBodyToCamera_(row,col) = matrix[4*row+col];
			}
		}
		this->nh_.param("autonomous_flight/return_home_on_complete", this->returnHomeEnabled_, true);
		this->nh_.param("autonomous_flight/completion_gain_threshold", this->completionGainThreshold_, 0);
		this->completionGainThreshold_ = std::max(0, this->completionGainThreshold_);
		this->nh_.param("autonomous_flight/return_home_tolerance", this->homeTolerance_, 0.2);
		this->homeTolerance_ = std::max(0.05, this->homeTolerance_);
		int confirmations; double duration;
		this->nh_.param("autonomous_flight/completion_confirmations", confirmations, 3);
		this->nh_.param("autonomous_flight/completion_confirm_duration", duration, 5.0);
		this->completionGate_ = CompletionGate(confirmations, duration);
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}
		
		// desired velocity
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[AutoFlight]: No desired velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}	

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 2.0;
			cout << "[AutoFlight]: No desired acceleration param. Use default 2.0m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 0.5;
			cout << "[AutoFlight]: No angular velocity param. Use default 0.5rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/waypoint_stablize_time", this->wpStablizeTime_)){
			this->wpStablizeTime_ = 1.0;
			cout << "[AutoFlight]: No waypoint stablize time param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Waypoint stablize time is set to: " << this->wpStablizeTime_ << "s." << endl;
		}	

		//  initial scan
		if (not this->nh_.getParam("autonomous_flight/initial_scan", this->initialScan_)){
			this->initialScan_ = false;
			cout << "[AutoFlight]: No initial scan param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Initial scan is set to: " << this->initialScan_ << endl;
		}	

    	// replan time for dynamic obstacle
		if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
			this->replanTimeForDynamicObstacle_ = 0.3;
			cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.3s." << endl;
		}
		else{
			cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		}	

    	// free range 
    	std::vector<double> freeRangeTemp;
		if (not this->nh_.getParam("autonomous_flight/free_range", freeRangeTemp)){
			this->freeRange_ = Eigen::Vector3d (2, 2, 1);
			cout << "[AutoFlight]: No free range param found. Use default: [2, 2, 1]m." << endl;
		}
		else{
			this->freeRange_(0) = freeRangeTemp[0];
			this->freeRange_(1) = freeRangeTemp[1];
			this->freeRange_(2) = freeRangeTemp[2];
			cout << "[AutoFlight]: Free range is set to: " << this->freeRange_.transpose() << "m." << endl;
		}	

    	// reach goal distance
		if (not this->nh_.getParam("autonomous_flight/reach_goal_distance", this->reachGoalDistance_)){
			this->reachGoalDistance_ = 0.1;
			cout << "[AutoFlight]: No reach goal distance param found. Use default: 0.1m." << endl;
		}
		else{
			cout << "[AutoFlight]: Reach goal distance is set to: " << this->reachGoalDistance_ << "m." << endl;
		}	
	}

	void dynamicExploration::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}

		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->nh_));
		this->expPlanner_->setMap(this->map_);
		this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);		
	}

	void dynamicExploration::registerCallback(){
		// initialize exploration planner replan in another thread
		this->exploreReplanWorker_ = std::thread(&dynamicExploration::exploreReplan, this);
		this->exploreReplanWorker_.detach();

		// exploration callback
		// this->explorationTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::explorationCB, this);

		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicExploration::plannerCB, this);

		// replan check timer
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::replanCheckCB, this);
		
		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::trajExeCB, this);
	
		// visualization execution callabck
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicExploration::visCB, this);
	}

	void dynamicExploration::registerPub(){
		this->missionStatePub_ = this->nh_.advertise<std_msgs::String>("dynamicExploration/mission_state", 1, true);
		this->homePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("dynamicExploration/home", 1, true);
		this->returnPathPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/return_path", 1, true);
		this->planningEventPub_ = this->nh_.advertise<std_msgs::String>("dynamicExploration/planning_event", 100);
		if (this->depthEvidenceArm_ == "F_DIAGNOSTIC" ||
		    this->depthRecoveryEnabled()){
			this->depthEvidenceStatusPub_ = this->nh_.advertise<std_msgs::String>(
				"dynamicExploration/depth_evidence_status", 10);
		}
		if (this->depthRecoveryEnabled()){
			this->depthRecoveryEventPub_ = this->nh_.advertise<std_msgs::String>(
				"dynamicExploration/depth_recovery_event", 20);
		}
		std::string depthTopic;
		this->nh_.param<std::string>("dynamic_map/depth_image_topic", depthTopic, "/camera/depth/image_raw");
		this->completionDepthSub_ = this->nh_.subscribe(depthTopic, 1, &dynamicExploration::completionDepthCB, this);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/poly_traj", 1000);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/pwl_trajectory", 1000);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/bspline_trajectory", 1000);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/input_trajectory", 1000);
	}

	void dynamicExploration::explorationCB(const ros::TimerEvent&){
		if (this->explorationReplan_){
			this->expPlanner_->setMap(this->map_);
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
				this->waypointIdx_ = 1;
				this->explorationReplan_ = false;
			}
			ros::Time endTime = ros::Time::now();
			cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).toSec() << "s." << endl;			
		}
	}

	void dynamicExploration::plannerCB(const ros::TimerEvent&){
		if (this->homeReached_ || this->handlingWaypoints_) return;
		// cout << "in planner callback" << endl;

		if (this->replan_){
			using Clock = std::chrono::steady_clock;
			const auto localStart = Clock::now();
			const uint64_t localSequence = ++this->localPlanningSequence_;
			const std::string replanReason = this->pendingLocalReplanReason_;
			double inputPathMs = 0.0, updatePathMs = 0.0, bsplineMs = 0.0;
			double depthEvidenceCheckMs = 0.0;
			bool planSuccess = false;
			bool depthEvidenceRejected = false;
			bool executionSnapshotWritten = false;
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}			
			nav_msgs::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions);
			// double initTs = this->bsplineTraj_->getInitTs();

			// generate new trajectory
			nav_msgs::Path simplePath;
			geometry_msgs::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			simplePath.poses = {pStart, pGoal};
			const auto inputStart = Clock::now();
			this->pwlTraj_->updatePath(simplePath, false);
			this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			inputPathMs = std::chrono::duration<double, std::milli>(Clock::now()-inputStart).count();
			// if (not this->trajectoryReady_){
			// 	// generate new trajectory
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart.pose = this->odom_.pose.pose;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// }
			// else{
			// 	Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
			// 	geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
			// 	Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
			// 	// if ((bsplineLastPos - goalPos).norm() >= 0.1){
			// 	nav_msgs::Path inputPWLTraj;
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart = lastPs;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputPWLTraj, this->bsplineTraj_->getControlPointDist());


			// 	nav_msgs::Path adjustedInputCombinedTraj;
			// 	bool satisfyDistanceCheck = false;
			// 	double dtTemp = initTs;
			// 	double finalTimeTemp;
			// 	ros::Time startTime = ros::Time::now();
			// 	ros::Time currTime;
			// 	while (ros::ok()){
			// 		currTime = ros::Time::now();
			// 		if ((currTime - startTime).toSec() >= 0.05){
			// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 			break;
			// 		}							
			// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 		nav_msgs::Path inputCombinedTraj;
			// 		inputCombinedTraj.poses = inputRestTraj.poses;
			// 		for (size_t i=1; i<inputPWLTraj.poses.size(); ++i){
			// 			inputCombinedTraj.poses.push_back(inputPWLTraj.poses[i]);
			// 		}
					
			// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
			// 		if (satisfyDistanceCheck) break;
					
			// 		dtTemp *= 0.8; // magic number 0.8
			// 	}
			// 	inputTraj = adjustedInputCombinedTraj;
			// 	// }
			// 	// else{
			// 	// 	nav_msgs::Path adjustedInputRestTraj;
			// 	// 	bool satisfyDistanceCheck = false;
			// 	// 	double dtTemp = initTs;
			// 	// 	double finalTimeTemp;
			// 	// 	ros::Time startTime = ros::Time::now();
			// 	// 	ros::Time currTime;
			// 	// 	while (ros::ok()){
			// 	// 		currTime = ros::Time::now();
			// 	// 		if ((currTime - startTime).toSec() >= 0.05){
			// 	// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 	// 			break;
			// 	// 		}
			// 	// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 	// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
			// 	// 		if (satisfyDistanceCheck) break;
						
			// 	// 		dtTemp *= 0.8;
			// 	// 	}
			// 	// 	inputTraj = adjustedInputRestTraj;					
			// 	// }

			// }

			

			this->inputTrajMsg_ = inputTraj;
			const auto updateStart = Clock::now();
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			updatePathMs = std::chrono::duration<double, std::milli>(Clock::now()-updateStart).count();
			if (obstaclesPos.size() != 0 and updateSuccess){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				const auto bsplineStart = Clock::now();
				planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
                if (planSuccess && this->terminalRecoveryEnabled_ && !this->returningHome_) {
                    auto curve = this->bsplineTraj_->getTrajectory();
                    const Eigen::Vector3d endpoint = curve.at(curve.getDuration());
                    if (!this->expPlanner_->isPosValid(endpoint)) {
                        const auto& p = pGoal.pose.position;
                        const Eigen::Vector3d goal(p.x, p.y, p.z);
                        const auto safety = this->expPlanner_->compareEndpointSafety(goal, endpoint);
                        ROS_INFO_STREAM("[TerminalRecovery] map=" << safety.mapVersion
                            << " goal_safe=" << safety.goalSafe << " endpoint_safe=" << safety.endpointSafe
                            << " goal=" << goal.transpose() << " endpoint=" << endpoint.transpose()
                            << " rejected_goal_sample=" << safety.goalFailure.transpose()
                            << " rejected_endpoint_sample=" << safety.endpointFailure.transpose());
                        if (!safety.endpointSafe) {
                            if (safety.goalSafe && this->bsplineTraj_->retryExactEnd(goal, bsplineTrajMsgTemp)) {
                                curve = this->bsplineTraj_->getTrajectory();
                                const Eigen::Vector3d corrected = curve.at(curve.getDuration());
                                const bool reachedGoal = (corrected - goal).norm() <= 1e-8;
                                ROS_INFO_STREAM("[TerminalRecovery] exact_end error=" << (corrected - goal).norm());
                                planSuccess = reachedGoal && this->expPlanner_->isPosValid(corrected);
                            } else {
                                planSuccess = false;
                            }
                            if (!planSuccess) {
                                this->expPlanner_->avoidRejectedWaypoint(goal);
                                ROS_WARN_STREAM("[TerminalRecovery] rejected waypoint " << goal.transpose());
                                this->turnPending_ = false;
                                this->turnActive_ = false;
                                this->trajectoryReady_ = false;
                                this->replan_ = false;
                                this->stop();
                                this->explorationReplan_ = true;
                                return;
                            }
                            ROS_INFO("[TerminalRecovery] accepted exact-end retry");
                        }
                    }
                }

				bsplineMs = std::chrono::duration<double, std::milli>(Clock::now()-bsplineStart).count();
				if (planSuccess && this->depthRecoveryEnabled() &&
				    this->depthFaultObserved_ && !this->returningHome_){
					const auto evidenceStart = Clock::now();
					const auto obligation = this->inspectDepthEvidencePath(bsplineTrajMsgTemp);
					depthEvidenceCheckMs = std::chrono::duration<double, std::milli>(
						Clock::now()-evidenceStart).count();
					if (obligation.found){
						depthEvidenceRejected = true;
						planSuccess = false;
						if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Idle){
							this->beginDepthRecovery(obligation);
						} else if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Moving){
							this->trajectoryReady_ = false;
							this->replan_ = false;
							this->depthRecoveryPhase_ = DepthRecoveryPhase::Select;
							this->stop();
							this->publishDepthRecoveryEvent("ACTION_REJECTED", "bspline_missing_evidence");
						}
					}
				}
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
                    if (this->turnPending_ && !this->configureTurnSync(this->bsplineTraj_->getTrajectory())) {
                        this->startSerialTurnFallback();
                        return;
                    }
					this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
                    if(!this->executionScale_.commit(this->bsplineTraj_->getLinearFactor())){
                        this->turnActive_=false;this->trajectoryReady_=false;
                        this->stop();this->replan_=true;return;
                    }
					this->trajectory_ = this->bsplineTraj_->getTrajectory();
					executionSnapshotWritten =
						this->expPlanner_->exportExecutionDiagnosticSnapshot(localSequence);

					// optimize time
					// ros::Time timeOptStartTime = ros::Time::now();
					// this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
					// ros::Time timeOptEndTime = ros::Time::now();
					// cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else if (depthEvidenceRejected){
					// A generated spline was deliberately withheld until its swept
					// volume has actual valid-depth evidence.
				}
				else if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Moving){
					this->trajectoryReady_ = false;
					this->replan_ = false;
					this->depthRecoveryPhase_ = DepthRecoveryPhase::Select;
					this->stop();
					this->publishDepthRecoveryEvent("ACTION_REJECTED", "bspline_plan_failed");
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
						this->explorationReplan_ = true;
					}
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
						this->pendingLocalReplanReason_ = "local_plan_retry_dynamic_collision";
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
                            ROS_INFO_STREAM("[ExecutionClock] retained_scale="<<this->executionScale_.value()<<" rejected_candidate_scale="<<this->bsplineTraj_->getLinearFactor());
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory." << endl;
							cout << "\033[1;32m[AutoFlight]: Wait for a new exploration path.\033[0m" << endl;
							this->replan_ = false;
							this->explorationReplan_ = true;
						}
					}
				}
			}
			else{
				this->trajectoryReady_ = false;
				this->stop();
				this->replan_ = false;
				if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Moving){
					this->depthRecoveryPhase_ = DepthRecoveryPhase::Select;
					this->publishDepthRecoveryEvent("ACTION_REJECTED", "path_update_failed");
				} else this->explorationReplan_ = true;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
			}

			const double totalMs = std::chrono::duration<double, std::milli>(Clock::now()-localStart).count();
			std_msgs::String event;
			std::ostringstream json;
			json << std::fixed << std::setprecision(3)
				 << "{\"schema_version\":3,\"kind\":\"local\",\"sequence\":" << localSequence
				 << ",\"global_sequence\":" << this->activeGlobalSequence_
				 << ",\"trajectory_id\":" << (planSuccess ? localSequence : 0)
				 << ",\"replan_reason\":\"" << replanReason << "\""
				 << ",\"map_version\":" << this->map_->getMapVersion()
				 << ",\"depth_sequence\":" << this->depthSequence_.load()
				 << ",\"trajectory_start_sim\":" << (planSuccess ? this->trajStartTime_.toSec() : 0.0)
				 << ",\"start_x\":" << this->odom_.pose.pose.position.x
				 << ",\"start_y\":" << this->odom_.pose.pose.position.y
				 << ",\"start_z\":" << this->odom_.pose.pose.position.z
				 << ",\"start_yaw\":" << AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation)
				 << ",\"execution_snapshot_written\":" << (executionSnapshotWritten ? "true" : "false")
				 << ",\"execution_snapshot_name\":\"execution_"
				 << std::setw(6) << std::setfill('0') << localSequence << "\""
				 << ",\"waypoint_index\":" << std::max(0, this->waypointIdx_-1)
				 << ",\"sim_time\":" << ros::Time::now().toSec()
				 << ",\"success\":" << (planSuccess ? "true" : "false")
					 << ",\"depth_evidence_rejected\":"
					 << (depthEvidenceRejected ? "true" : "false")
				 << ",\"update_success\":" << (updateSuccess ? "true" : "false")
				 << ",\"input_path_ms\":" << inputPathMs
				 << ",\"update_path_ms\":" << updatePathMs
				 << ",\"bspline_ms\":" << bsplineMs
					 << ",\"depth_evidence_check_ms\":" << depthEvidenceCheckMs
				 << ",\"input_path_length\":" << pathLength(inputTraj)
				 << ",\"input_path_poses\":" << inputTraj.poses.size()
				 << ",\"bspline_path_length\":" << (planSuccess ? pathLength(this->bsplineTrajMsg_) : 0.0)
				 << ",\"bspline_path_poses\":" << (planSuccess ? this->bsplineTrajMsg_.poses.size() : 0)
				 << ",\"total_ms\":" << totalMs
				 << ",\"dynamic_obstacles\":" << obstaclesPos.size();
			appendPathPoints(json, "input_path_points", inputTraj);
			if (planSuccess) appendPathPoints(json, "bspline_path_points", this->bsplineTrajMsg_);
			json << "}";
			event.data = json.str();
			this->planningEventPub_.publish(event);
			if (planSuccess) this->pendingLocalReplanReason_ = "unspecified";
		}
	}

	void dynamicExploration::replanCheckCB(const ros::TimerEvent&){
		if (this->handlingWaypoints_) return; // moveToOrientation spins callbacks recursively.
		struct Guard { bool& busy; Guard(bool& b): busy(b) {busy=true;} ~Guard(){busy=false;} } guard(this->handlingWaypoints_);
		if (this->homeReached_) return;
		if (this->depthRecoveryEnabled() && this->handleDepthRecovery()) return;
		{
			std::lock_guard<std::mutex> lock(this->resultMutex_);
			if (this->resultReady_) {
                if (this->serialTurnFallback_.active()) {
                    this->serialTurnFallback_.cancel();
                    this->turnEvent("serial_fallback_cancelled");
                }
				this->resultReady_ = false;
				this->replan_ = false;
				this->trajectoryReady_ = false;
				this->newWaypoints_ = false;
				this->stop();
				this->setMissionState(this->resultState_);
				if (this->resultPath_.poses.size() >= 2) {
					this->waypoints_ = this->resultPath_;
					this->activeGlobalSequence_ = this->resultGlobalSequence_;
					this->waypointIdx_ = 1;
					this->newWaypoints_ = true;
				} else {
					this->waypoints_.poses.clear();
					this->explorationReplan_ = true;
				}
			}
		}
		if (this->returningHome_) {
			const double distance = AutoFlight::getPoseDistance(this->odom_.pose.pose, this->homePose_.pose);
			const auto& v = this->odom_.twist.twist.linear;
			const double speed = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
			if (distance <= this->homeTolerance_) {
				this->settlingHome_ = true;
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->newWaypoints_ = false;
				this->explorationReplan_ = false;
				this->updateTarget(this->homePose_);
				const double now = ros::Time::now().toSec();
				if (speed < 0.15) {
					if (this->homeStillSince_ < 0) this->homeStillSince_ = now;
					if (now - this->homeStillSince_ >= this->homeHoldTime_) {
						this->homeReached_ = true;
						this->setMissionState("HOME_REACHED");
						ROS_INFO("[ReturnHome] Arrived at takeoff location; hovering, no landing.");
					}
				} else this->homeStillSince_ = -1;
				return;
			}
			if (this->settlingHome_) {
				this->settlingHome_ = false;
				this->homeStillSince_ = -1;
				this->explorationReplan_ = true;
			}
		}
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/

        if (this->serialTurnFallback_.active()) {
            if (this->returningHome_) {
                this->serialTurnFallback_.cancel();
                this->turnEvent("serial_fallback_cancelled");
                this->stop();
            } else {
                this->advanceSerialTurnFallback();
            }
            return;
        }
        if (this->turnActive_) {
            bool prefixSafe = true;
            if (this->trajectoryReady_) {
                for (double dt=0; dt<=.2; dt+=.02) {
                    double u=std::min(this->trajectory_.getDuration(), this->trajTime_+dt*this->executionScale_.value());
                    if (!this->expPlanner_->isPosValid(this->trajectory_.at(u))) prefixSafe=false;
                }
            }
            if (!prefixSafe || !this->trajectoryReady_ || this->hasCollision() || this->hasDynamicCollision() ||
                !this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))) {
                this->turnEvent("runtime_safety_abort");
                this->turnActive_ = false; this->turnPending_ = false;
                this->trajectoryReady_ = false; this->stop();
                this->replan_ = false; this->explorationReplan_ = true;
            }
            return;
        }
		if (this->newWaypoints_){
			if (this->waypoints_.poses.size() < 2){
				this->newWaypoints_ = false;
				this->replan_ = false;
				this->trajectoryReady_ = false;
				this->explorationReplan_ = true;
				cout << "[AutoFlight]: Exploration path has fewer than two waypoints. Requesting a new path." << endl;
				return;
			}
			this->replan_ = false;
			this->trajectoryReady_ = false;
			double yaw = atan2(this->waypoints_.poses[1].pose.position.y - this->odom_.pose.pose.position.y, this->waypoints_.poses[1].pose.position.x - this->odom_.pose.pose.position.x);
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			this->requestExplorationTurn(yaw, true);
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();		
			this->replan_ = true;
			this->pendingLocalReplanReason_ = "new_global_path";
			this->newWaypoints_ = false;
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			++this->waypointIdx_;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 

			return;
		}

		// if (this->isReach(this->goal_, 0.1, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
		// cout << "outside the if" << endl;
		// cout << "waypoints size: " << this->waypoints_.poses.size() << endl;
		// cout << "current waypoint idx: " << this->waypointIdx_ << endl;
		if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
			// cout << "1" << endl;
			// when reach current goal point, reset replan and trajectory ready
			this->replan_ = false;
			this->trajectoryReady_ = false;
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			cout << "[AutoFlight]: Rotate and replan..." << endl;
			geometry_msgs::Quaternion quat = this->goal_.pose.orientation;
			double yaw = AutoFlight::rpy_from_quaternion(quat);
			this->waitTime(this->wpStablizeTime_);
			this->requestExplorationTurn(yaw, this->waypointIdx_ < int(this->waypoints_.poses.size()));
			cout << "[AutoFlight]: Finish rotation." << endl;
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();	

			// change current goal
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			if (this->waypointIdx_ + 1 > int(this->waypoints_.poses.size())){
				cout << "\033[1;32m[AutoFlight]: Finishing entire path. Requesting a new exploration path.\033[0m" << endl;
				this->replan_ = false;
				this->explorationReplan_ = true;
			}
			else{
				cout << "[AutoFlight]: Start planning for next waypoint." << endl;
				this->replan_ = true;
				this->pendingLocalReplanReason_ = "next_waypoint";
			}
			++this->waypointIdx_;
			this->trajectoryReady_ = false;
	
			return;		
		}
		else if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, true) and (this->replan_ or this->trajectoryReady_)){
			cout << "\033[[AutoFlight]: Finishing entire path. Requesting a new exploration path.\033[0m" << endl;
			this->replan_ = false;
			this->trajectoryReady_ = false;
			this->explorationReplan_ = true;
			return;		
		}

		if (this->waypoints_.poses.size() != 0){
			if (not this->isGoalValid() and (this->replan_ or this->trajectoryReady_)){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				cout << "\033[1;32m[AutoFlight]: Current goal is invalid. Requesting a new exploration path.\033[0m" << endl;
				this->explorationReplan_ = true;
				return;
			}
		}

		// if (this->reachExplorationGoal()){
		// 	this->replan_ = false;
		// 	this->trajectoryReady_ = false;
		// 	geometry_msgs::Quaternion quat = this->waypoints_.poses.back().pose.orientation;
		// 	double yaw = AutoFlight::rpy_from_quaternion(quat);
		// 	cout << "[AutoFlight]: Reach exploration goal. Rotate and replan..." << endl;
		// 	this->moveToOrientation(yaw, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Finish rotation. Start to replan." << endl;
		// 	this->replan_ = true;
		// 	return;
		// }

		if (this->trajectoryReady_){
			if (not this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))){
                if (this->terminalRecoveryEnabled_ && !this->returningHome_) {
                    const auto& p = this->goal_.pose.position;
                    this->expPlanner_->avoidRejectedWaypoint(Eigen::Vector3d(p.x, p.y, p.z));
                }
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				cout << "\033[1;32m[AutoFlight]: The current local goal is unsafe. Requesting a new exploration path.\033[0m" << endl;
				this->explorationReplan_ = true;
				return;
			}

			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				this->pendingLocalReplanReason_ = "static_collision";
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
			// if (this->hasDynamicObstacle()){
				this->replan_ = true;
				this->pendingLocalReplanReason_ = "dynamic_collision";
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				this->pendingLocalReplanReason_ = "distance_progress";
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 0.3 and this->replanForDynamicObstacle()){
				this->replan_ = true;
				this->pendingLocalReplanReason_ = "dynamic_obstacle_periodic";
				cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
				return;
			}
		}	
	}

	void dynamicExploration::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
            const double turnElapsed = realTime;
            const bool turning = this->turnActive_;
            if (turning) realTime = this->turnSchedule_.motionTime(realTime);

			this->trajTime_ = this->executionScale_.time(realTime);
			double linearReparamFactor = this->executionScale_.value();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
			double endTime = this->trajectory_.getDuration()/linearReparamFactor;

            if (turning && turnElapsed < this->turnSchedule_.delay) { vel.setZero(); acc.setZero(); }
			double leftTime = endTime - realTime; 
			tracking_controller::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = turning ? this->turnSchedule_.yaw(turnElapsed) : AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				target.yaw = turning ? this->turnSchedule_.yaw(turnElapsed) : AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}
            if (turning && turnElapsed >= this->turnSchedule_.duration &&
                std::abs(TurnSchedule::wrap(AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation)-this->turnTarget_)) < 0.05) {
                this->turnEvent("completed");
                this->trajStartTime_ += ros::Duration(this->turnSchedule_.delay);
                this->turnActive_ = false;
            }
		}
	}

	void dynamicExploration::visCB(const ros::TimerEvent&){
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
	}

	void dynamicExploration::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		double fov = 1.57;
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z);
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}

	void dynamicExploration::run(){
		cout << "\033[1;32m[AutoFlight]: Please double check all parameters. Then PRESS ENTER to continue or PRESS CTRL+C to stop.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();
		// Keep actual takeoff XY and yaw, at the configured flight altitude.
		this->homePose_.header.frame_id = "map";
		this->homePose_.header.stamp = ros::Time::now();
		this->homePose_.pose = this->odom_.pose.pose;
		this->homePose_.pose.position.z = this->takeoffHgt_;
		this->takeoff();
		this->homePub_.publish(this->homePose_);
		this->setMissionState("EXPLORING");

		cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Then PRESS ENTER to continue or PRESS CTRL+C to land.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();

		bool recordRosbag = false;
		this->nh_.param("autonomous_flight/record_rosbag", recordRosbag, false);
		if (recordRosbag){
			int temp1 = system("mkdir -p ~/rosbag_exploration_info");
			int temp2 = system("mv ~/rosbag_exploration_info/exploration_info.bag ~/rosbag_exploration_info/previous.bag 2>/dev/null");
			int temp3 = system("rosbag record -O ~/rosbag_exploration_info/exploration_info.bag /camera/aligned_depth_to_color/image_raw_t /camera/color/image_raw_t /dynamic_map/inflated_voxel_map_t /onboard_detector/dynamic_bboxes /mavros/local_position/pose /dynamicExploration/bspline_trajectory /mavros/setpoint_position/local /tracking_controller/target_pose /dep/best_paths /dep/roadmap /dep/candidate_paths /dep/best_paths /dep/frontier_regions /dynamic_map/2D_occupancy_map __name:=exploration_bag_info &");
			if (temp1==-1 or temp2==-1 or temp3==-1){
				cout << "[AutoFlight]: Recording fails." << endl;
			}
		}
		else{
			cout << "[AutoFlight]: Exploration rosbag recording is disabled." << endl;
		}

		this->initExplore();

		cout << "\033[1;32m[AutoFlight]: PRESS ENTER to Start Planning.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();

		this->registerCallback();
	}

	void dynamicExploration::initExplore(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->initialClearCenter_ = startPos;
		Eigen::Vector3d c1 = startPos - this->freeRange_;
		Eigen::Vector3d c2 = startPos + this->freeRange_;
		this->map_->freeRegion(c1, c2);
		cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

		if (this->initialScan_){
			cout << "[AutoFlight]: Start initial scan..." << endl;
			this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
						
			this->moveToOrientation(-PI_const, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();

			this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
			
			this->moveToOrientation(0, this->desiredAngularVel_);
			cout << "[AutoFlight]: End initial scan." << endl; 
		}		
	}

	void dynamicExploration::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		Eigen::Vector3d currVel = this->currVel_;
		Eigen::Vector3d currAcc = this->currAcc_;
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		// if (not this->trajectoryReady_){
		// 	double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// 	Eigen::Vector3d direction (cos(yaw), sin(yaw), 0.0);
		// 	currVel = this->desiredVel_ * direction;
		// 	currAcc = this->desiredAcc_ * direction;
		// }
		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);
	}

	bool dynamicExploration::hasCollision(){
		if (this->trajectoryReady_){
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					return true;
				}
			}
		}
		return false;
	}

	bool dynamicExploration::hasDynamicCollision(){
		if (this->trajectoryReady_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}

			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				
				for (size_t i=0; i<obstaclesPos.size(); ++i){
					Eigen::Vector3d ob = obstaclesPos[i];
					Eigen::Vector3d size = obstaclesSize[i];
					Eigen::Vector3d lowerBound = ob - size/2;
					Eigen::Vector3d upperBound = ob + size/2;
					if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
						p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
						p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
						return true;
					}					
				}
			}
		}
		return false;
	}

	void dynamicExploration::exploreReplan(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		// Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		// Eigen::Vector3d c1 = startPos - range;
		// Eigen::Vector3d c2 = startPos + range;
		// this->map_->freeRegion(c1, c2);
		// cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << range.transpose() << endl;

		// if (this->initialScan_){
		// 	cout << "[AutoFlight]: Start initial scan..." << endl;
		// 	this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
						
		// 	this->moveToOrientation(-PI_const, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();

		// 	this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
			
		// 	this->moveToOrientation(0, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: End initial scan." << endl; 
		// }
		while (ros::ok()){
			if (not this->explorationReplan_.exchange(false)){
				ros::WallDuration(0.05).sleep();
				continue;
			}
			bool pending;
			{
				std::lock_guard<std::mutex> lock(this->resultMutex_);
				pending = this->resultReady_;
			}
			if (pending) { this->explorationReplan_ = true; ros::WallDuration(0.05).sleep(); continue; }
			nav_msgs::Path plannedWaypoints;
			uint64_t plannedGlobalSequence = 0;
			std::string state;
			const double now = ros::Time::now().toSec();
			const double depthTime = this->lastDepthTime_.load();
			const bool sensorFresh = depthTime >= 0 && now >= depthTime && now - depthTime < 2.0;
			const ros::WallTime startTime = ros::WallTime::now();
			if (!sensorFresh) {
				this->completionGate_.reset();
				state = this->returningHome_ ? "RETURN_BLOCKED_SENSOR" : "EXPLORATION_BLOCKED_SENSOR";
			} else if (this->returningHome_) {
				const auto& p = this->homePose_.pose.position;
				const auto returnStart = std::chrono::steady_clock::now();
				bool success = this->expPlanner_->planReturnPath(Eigen::Vector3d(p.x,p.y,p.z), plannedWaypoints);
				const uint64_t returnSequence = ++this->returnPlanningSequence_;
				const double returnMs = std::chrono::duration<double, std::milli>(
					std::chrono::steady_clock::now()-returnStart).count();
				std_msgs::String event;
				std::ostringstream json;
				json << std::fixed << std::setprecision(3)
					 << "{\"schema_version\":2,\"kind\":\"return\",\"sequence\":" << returnSequence
					 << ",\"sim_time\":" << now
					 << ",\"success\":" << (success ? "true" : "false")
					 << ",\"total_ms\":" << returnMs
					 << ",\"path_length\":" << pathLength(plannedWaypoints)
					 << ",\"path_poses\":" << plannedWaypoints.poses.size();
				appendPathPoints(json, "path_points", plannedWaypoints);
				json << "}";
				event.data = json.str();
				this->planningEventPub_.publish(event);
				state = success ? "RETURNING_HOME" : "RETURN_BLOCKED";
				if (success) {
					plannedWaypoints.poses.back().pose.orientation = this->homePose_.pose.orientation;
					this->returnPathPub_.publish(plannedWaypoints);
				}
			} else {
				const bool legacySuccess = this->expPlanner_->makePlan();
                bool success = legacySuccess;
				const globalPlanner::DEPPlanningMetrics metrics = this->expPlanner_->getLastPlanningMetrics();
				plannedGlobalSequence = metrics.sequence;
				nav_msgs::Path selectedPath = success ? this->expPlanner_->getBestPath() : nav_msgs::Path();
				nav_msgs::Path rawPath = success ? this->expPlanner_->getBestRawPath() : nav_msgs::Path();
                std::string decision = "original";
                double selectorMs = metrics.totalMs;
                double sharedRouteMs = 0.;
                geometry_msgs::PoseStamped view;

                size_t viewCandidates = 0;
                size_t viewRejected = 0;
                if (viewpointSelector_ != "original") {
                    nav_msgs::Path candidates;
                    if (viewpointSelector_ == "cerlab" && !selectedPath.poses.empty()) {
                        candidates.poses.push_back(selectedPath.poses.back());
                    } else if (viewpointSelector_ == "fuel") {
                        const auto viewStart = std::chrono::steady_clock::now();
                        nav_msgs::GetPlan query;
                        if (nextViewClient_.call(query) && query.response.plan.header.frame_id == "map")
                            candidates = std::move(query.response.plan);
                        selectorMs = std::chrono::duration<double,std::milli>(
                            std::chrono::steady_clock::now()-viewStart).count();
                    }
                    selectedPath = nav_msgs::Path();
                    rawPath = nav_msgs::Path();
                    const auto sharedRouteStart = std::chrono::steady_clock::now();
                    success = false;
                    viewCandidates = std::min<size_t>(48, candidates.poses.size());
                    if (viewCandidates > 0) view = candidates.poses.front();
                    for (size_t index = 0; index < viewCandidates; ++index) {
                        nav_msgs::Path candidatePath;
                        if (expPlanner_->planViewPath(candidates.poses[index], candidatePath)) {
                            selectedPath = std::move(candidatePath);
                            view = candidates.poses[index];
                            success = true;
                            break;
                        }
                        ++viewRejected;
                    }
                    sharedRouteMs = std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now()-sharedRouteStart).count();
                    decision = viewCandidates == 0 ? "no_view" : (success ? "routed" : "route_rejected");
                    ROS_INFO_STREAM("[ViewSelection] selector=" << viewpointSelector_
                      << " outcome=" << decision << " candidates=" << viewCandidates
                      << " rejected_before_selection=" << viewRejected
                      << " xyz=" << view.pose.position.x << "," << view.pose.position.y
                      << "," << view.pose.position.z);
                }
				std_msgs::String event;
				std::ostringstream json;
				json << std::fixed << std::setprecision(3)
					 << "{\"schema_version\":4,\"kind\":\"global\",\"sequence\":" << metrics.sequence
					 << ",\"sim_time\":" << now
					 << ",\"map_version\":" << this->map_->getMapVersion()
					 << ",\"depth_sequence\":" << this->depthSequence_.load()
					 << ",\"success\":" << (success ? "true" : "false")
                     << ",\"viewpoint_selector\":\"" << viewpointSelector_ << "\""
                     << ",\"viewpoint_result\":\"" << decision << "\""
                     << ",\"viewpoint_xyz\":[" << view.pose.position.x << "," << view.pose.position.y
                     << "," << view.pose.position.z << "]"
                     << ",\"viewpoint_candidates\":" << viewCandidates
                     << ",\"viewpoint_rejected\":" << viewRejected
                     << ",\"selector_ms\":" << selectorMs
                     << ",\"shared_route_ms\":" << sharedRouteMs
					 << ",\"recovery_used\":" << (metrics.recoveryUsed ? "true" : "false")
					 << ",\"roadmap_nodes\":" << metrics.roadmapNodes
					 << ",\"goal_candidates\":" << metrics.goalCandidates
					 << ",\"candidate_paths\":" << metrics.candidatePaths
					 << ",\"best_path_gain\":" << metrics.bestPathGain
					 << ",\"gain_schema_version\":" << metrics.gainSchemaVersion
					 << ",\"gain_mode\":\"" << metrics.configuredGainMode << "\""
					 << ",\"selection_gain_mode\":\"" << metrics.selectionGainMode << "\""
					 << ",\"gain_sample_spacing\":" << metrics.gainSampleSpacing
					 << ",\"unique_evaluator_available\":"
					 << (metrics.uniqueEvaluatorAvailable ? "true" : "false")
					 << ",\"unique_evaluation_status\":\"" << metrics.uniqueEvaluationStatus << "\""
					 << ",\"legacy_selected_candidate\":" << metrics.legacySelectedCandidate;
				if (metrics.uniqueSelectedCandidate >= 0) json << ",\"unique_selected_candidate\":" << metrics.uniqueSelectedCandidate;
				else json << ",\"unique_selected_candidate\":null";
				if (metrics.selectedRawGain >= 0) json << ",\"selected_raw_gain\":" << metrics.selectedRawGain;
				else json << ",\"selected_raw_gain\":null";
				if (metrics.selectedUniqueGain >= 0) json << ",\"selected_unique_gain\":" << metrics.selectedUniqueGain;
				else json << ",\"selected_unique_gain\":null";
				if (metrics.selectedDuplicateRatio >= 0.0) json << ",\"selected_duplicate_ratio\":" << metrics.selectedDuplicateRatio;
				else json << ",\"selected_duplicate_ratio\":null";
				if (metrics.uniqueEvaluationMs >= 0.0) json << ",\"unique_evaluation_ms\":" << metrics.uniqueEvaluationMs;
				else json << ",\"unique_evaluation_ms\":null";
				if (metrics.uniqueMapVersion > 0) json << ",\"unique_map_version\":" << metrics.uniqueMapVersion;
				else json << ",\"unique_map_version\":null";
				if (metrics.uniqueTop1Changed >= 0) json << ",\"unique_top1_changed\":" << (metrics.uniqueTop1Changed ? "true" : "false");
				else json << ",\"unique_top1_changed\":null";
				if (metrics.uniqueScoreMargin >= 0.0) json << ",\"unique_score_margin\":" << metrics.uniqueScoreMargin;
				else json << ",\"unique_score_margin\":null";
				if (!metrics.gainFallbackReason.empty()) json << ",\"gain_fallback_reason\":\"" << metrics.gainFallbackReason << "\"";
				else json << ",\"gain_fallback_reason\":null";
				json
					 << ",\"selected_path_length\":" << pathLength(selectedPath)
					 << ",\"selected_path_poses\":" << selectedPath.poses.size()
					 << ",\"frontier_ms\":" << metrics.frontierMs
					 << ",\"roadmap_ms\":" << metrics.roadmapMs
					 << ",\"prune_ms\":" << metrics.pruneMs
					 << ",\"gain_update_ms\":" << metrics.gainUpdateMs
					 << ",\"goal_selection_ms\":" << metrics.goalSelectionMs
					 << ",\"candidate_search_ms\":" << metrics.candidateSearchMs
					 << ",\"path_scoring_ms\":" << metrics.pathScoringMs
					 << ",\"total_ms\":" << (metrics.totalMs + sharedRouteMs +
                        (viewpointSelector_ == "fuel" ? selectorMs : 0.));
				if (!metrics.routeControlJson.empty()) json << ",\"route_controls\":" << metrics.routeControlJson;
				appendPathPoints(json, "selected_path_points", selectedPath);
				appendPathPoints(json, "raw_path_points", rawPath);
				json << "}";
				event.data = json.str();
				this->planningEventPub_.publish(event);
				int checkedNodes = 0;
				// A failed path search is never completion by itself. If no path was
				// produced, a fresh scan of every currently reachable roadmap node may
				// still provide valid exhaustion evidence.
				const bool shouldAssess = this->returnHomeEnabled_ &&
					(!legacySuccess || (this->expPlanner_->getBestPathGain() >= 0 &&
					 this->expPlanner_->getBestPathGain() <= this->completionGainThreshold_));
				const bool exhausted = shouldAssess &&
					this->expPlanner_->reachableGainExhausted(this->completionGainThreshold_, checkedNodes);
				const bool assessmentValid = legacySuccess || (shouldAssess && checkedNodes > 0);
				const bool complete = this->completionGate_.observe(assessmentValid, exhausted, sensorFresh,
					this->depthSequence_.load(), now);
				if (complete) {
					this->returningHome_ = true;
					state = "RETURNING_HOME";
					ROS_INFO("[ReturnHome] Reachable roadmap gain is at or below threshold %d (%d nodes checked). Returning home; this is not a ground-truth coverage certificate.", this->completionGainThreshold_, checkedNodes);
				} else if (exhausted) {
					state = "CONFIRMING_COMPLETE";
				} else if (success) {
					plannedWaypoints = selectedPath;
					state = "EXPLORING";
				} else state = "EXPLORATION_BLOCKED";
			}
			{
				std::lock_guard<std::mutex> lock(this->resultMutex_);
				this->resultPath_ = plannedWaypoints;
				this->resultState_ = state;
				this->resultGlobalSequence_ = plannedGlobalSequence;
				this->resultReady_ = true;
			}
			ROS_INFO("[AutoFlight] Planning wall time: %.3f s", (ros::WallTime::now()-startTime).toSec());
			ros::WallDuration(plannedWaypoints.poses.empty() ? 1.0 : 0.1).sleep();
		}
	}

	void dynamicExploration::setMissionState(const std::string& state) {
		if (this->missionState_ == state) return;
		this->missionState_ = state;
		std_msgs::String message; message.data = state;
		this->missionStatePub_.publish(message);
		ROS_INFO_STREAM("[ReturnHome] Mission state: " << state);
	}

	void dynamicExploration::completionDepthCB(const sensor_msgs::ImageConstPtr& image) {
		if (image->data.empty() || !image->width || !image->height) return;
		this->lastDepthTime_ = image->header.stamp.toSec();
		++this->depthSequence_;
		if ((!this->depthRecoveryEnabled() &&
		     this->depthEvidenceArm_ != "F_DIAGNOSTIC") || image->encoding != "32FC1" ||
		    (++this->angularDiagnosticFrameCount_ % 5) != 0) return;
		try {
			const cv_bridge::CvImageConstPtr raw = cv_bridge::toCvShare(image, image->encoding);
			const auto mapSnapshot = this->map_->captureSnapshot();
			if (mapSnapshot.validHitFrameCount.empty()) return;
			nav_msgs::OdometryConstPtr odom(new nav_msgs::Odometry(this->odom_));
			Eigen::Matrix4d cameraToWorld;
			this->map_->getCameraPose(odom, cameraToWorld);
			std::vector<Eigen::Vector3d> obstaclePositions, obstacleVelocities, obstacleSizes;
			this->getDynamicObstacles(obstaclePositions, obstacleVelocities, obstacleSizes);
			const auto isDynamicReference = [&](const Eigen::Vector3d& point) {
				for (std::size_t i=0; i<obstaclePositions.size(); ++i) {
					const Eigen::Vector3d half = obstacleSizes[i] / 2.0 +
					                             Eigen::Vector3d::Constant(0.3);
					if (((point - obstaclePositions[i]).array().abs() <= half.array()).all())
						return true;
				}
				return false;
			};
			DepthIntrinsics camera;
			camera.fx = this->depthFx_; camera.fy = this->depthFy_;
			camera.cx = this->depthCx_; camera.cy = this->depthCy_;
			const auto frame = diagnoseAngularDepthFrame(
				raw->image, cameraToWorld, camera, mapSnapshot, 16, isDynamicReference);
			std_msgs::String status;
			{
				std::lock_guard<std::mutex> lock(this->angularReliabilityMutex_);
				this->angularReliability_.observe(frame);
				bool anyFaulty = false;
				for (std::size_t bin=0; bin<this->angularReliability_.binCount(); ++bin)
					if (this->angularReliability_.state(bin) == AngularReliabilityState::Faulty)
						anyFaulty = true;
				this->depthFaultObserved_ = anyFaulty;
				if (this->angularDiagnosticFrameCount_ % 20 == 0){
					std::ostringstream json;
					json << std::fixed << std::setprecision(3)
					     << "{\"schema_version\":1,\"sim_time\":"
					     << image->header.stamp.toSec()
					     << ",\"map_version\":" << mapSnapshot.version
					     << ",\"bins\":[";
					for (std::size_t bin=0; bin<this->angularReliability_.binCount(); ++bin){
						if (bin) json << ",";
						const auto state = this->angularReliability_.state(bin);
						const char* label = state == AngularReliabilityState::Healthy ? "healthy" :
						                    state == AngularReliabilityState::Faulty ? "faulty" : "unknown";
						json << "{\"state\":\"" << label
						     << "\",\"lower\":" << this->angularReliability_.lowerBound(bin)
						     << ",\"diagnostic_frames\":"
						     << this->angularReliability_.diagnosticFrames(bin) << "}";
					}
					json << "]}";
					status.data = json.str();
				}
			}
			if (!status.data.empty()) this->depthEvidenceStatusPub_.publish(status);
		} catch (const std::exception& error) {
			ROS_WARN_THROTTLE(5.0, "V2 angular depth diagnosis failed: %s", error.what());
		}
	}

	bool dynamicExploration::depthRecoveryEnabled() const {
		return this->depthEvidenceArm_ == "C" || this->depthEvidenceArm_ == "D" ||
		       this->depthEvidenceArm_ == "E" || this->depthEvidenceArm_ == "F";
	}

	void dynamicExploration::publishDepthRecoveryEvent(const std::string& name,
	                                                  const std::string& reason,
	                                                  double decisionMs) {
		if (!this->depthRecoveryEnabled()) return;
		std_msgs::String message;
		std::ostringstream json;
		json << std::fixed << std::setprecision(3)
		     << "{\"schema_version\":1,\"event\":\"" << name
		     << "\",\"reason\":\"" << reason
		     << "\",\"arm\":\"" << this->depthEvidenceArm_
		     << "\",\"sim_time\":" << ros::Time::now().toSec()
		     << ",\"attempts\":" << this->depthRecoveryAttempts_
		     << ",\"decision_ms\":";
		if (decisionMs < 0.0) json << "null";
		else json << decisionMs;
		json
		     << ",\"obligation\":[" << this->depthObligation_.x() << ","
		     << this->depthObligation_.y() << "," << this->depthObligation_.z() << "]}";
		message.data = json.str();
		this->depthRecoveryEventPub_.publish(message);
	}

	EvidenceObligation dynamicExploration::inspectDepthEvidencePath(const nav_msgs::Path& path) {
		std::vector<Eigen::Vector3d> positions;
		positions.reserve(path.poses.size());
		for (const auto& pose : path.poses) {
			positions.emplace_back(pose.pose.position.x, pose.pose.position.y,
			                       pose.pose.position.z);
		}
		Eigen::Vector3d robotSize;
		this->map_->getRobotSize(robotSize);
		const auto hasEvidence = [&](const Eigen::Vector3d& point) {
			const bool startPrior =
				((point - this->initialClearCenter_).array().abs() <= this->freeRange_.array()).all();
			return this->map_->isFree(point) &&
			       (startPrior || this->map_->hasValidDepthEvidence(point));
		};
		return firstMissingBodyEvidence(positions, robotSize / 2.0,
		                                1.5, 0.05, hasEvidence);
	}

	void dynamicExploration::beginDepthRecovery(const EvidenceObligation& obligation) {
		this->depthObligation_ = obligation.missingEvidencePosition;
		this->depthSavedGoal_ = this->goal_;
		this->depthRecoveryBaseYaw_ = AutoFlight::rpy_from_quaternion(
			this->odom_.pose.pose.orientation);
		this->depthRecoveryStartSim_ = ros::Time::now().toSec();
		this->depthRecoveryPhaseStartSim_ = this->depthRecoveryStartSim_;
		this->depthRecoveryAttempts_ = 0;
		this->depthRecoveryLastActionInPlace_ = false;
		this->depthTriedActions_.clear();
		bool alreadyHeld = false;
		if (this->depthEvidenceArm_ == "C") {
			for (const auto& previous : this->depthHeldObligations_) {
				if ((previous - this->depthObligation_).norm() < this->map_->getRes()) {
					alreadyHeld = true;
					break;
				}
			}
		}
		this->depthRecoveryPhase_ = alreadyHeld ? DepthRecoveryPhase::Blocked :
			(this->depthEvidenceArm_ == "C" ? DepthRecoveryPhase::Hold :
			 DepthRecoveryPhase::Select);
		this->trajectoryReady_ = false;
		this->bsplineTrajMsg_.poses.clear();
		this->replan_ = false;
		this->explorationReplan_ = false;
		this->stop();
		this->publishDepthRecoveryEvent("OBLIGATION_STARTED");
		if (alreadyHeld) this->publishDepthRecoveryEvent(
			"RECOVERY_BLOCKED", "hold_budget_exhausted_same_obligation");
	}

	bool dynamicExploration::handleDepthRecovery() {
		if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Idle) return false;
		if (this->returningHome_) {
			this->depthRecoveryPhase_ = DepthRecoveryPhase::Idle;
			return false;
		}
		const double now = ros::Time::now().toSec();
		const auto finish = [&]() {
			this->goal_ = this->depthSavedGoal_;
			this->trajectoryReady_ = false;
			this->replan_ = false;
			this->depthRecoveryPhase_ = DepthRecoveryPhase::Idle;
			{
				std::lock_guard<std::mutex> lock(this->resultMutex_);
				this->resultReady_ = false;
			}
			this->explorationReplan_ = true;
		};
		if (this->map_->hasValidDepthEvidence(this->depthObligation_)) {
			this->publishDepthRecoveryEvent("OBLIGATION_OBSERVED");
			finish();
			return true;
		}
		if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Blocked) return true;
		if (now - this->depthRecoveryStartSim_ >= 10.0 &&
		    this->depthRecoveryPhase_ != DepthRecoveryPhase::Hold) {
			this->depthRecoveryPhase_ = DepthRecoveryPhase::Blocked;
			this->trajectoryReady_ = false;
			this->replan_ = false;
			this->stop();
			this->publishDepthRecoveryEvent("RECOVERY_BLOCKED", "time_budget");
			return true;
		}
		if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Hold) {
			if (now - this->depthRecoveryPhaseStartSim_ >= 10.0) {
				this->depthHeldObligations_.push_back(this->depthObligation_);
				this->publishDepthRecoveryEvent("HOLD_EXPIRED");
				finish();
			}
			return true;
		}
		if (this->depthRecoveryPhase_ == DepthRecoveryPhase::WaitEvidence) {
			if (now - this->depthRecoveryPhaseStartSim_ >= 1.0) {
				this->depthRecoveryPhase_ = this->depthRecoveryAttempts_ >= 2 ?
					DepthRecoveryPhase::Blocked : DepthRecoveryPhase::Select;
				this->publishDepthRecoveryEvent(
					this->depthRecoveryPhase_ == DepthRecoveryPhase::Blocked ?
					"RECOVERY_BLOCKED" : "RETRY", "no_new_evidence");
			}
			return true;
		}
		if (this->depthRecoveryPhase_ == DepthRecoveryPhase::Moving) {
			if (this->isReach(this->depthRecoveryGoal_, 0.15, false)) {
				this->trajectoryReady_ = false;
				this->stop();
				this->moveToOrientation(
					AutoFlight::rpy_from_quaternion(this->depthRecoveryGoal_.pose.orientation),
					this->desiredAngularVel_);
				this->depthRecoveryPhase_ = DepthRecoveryPhase::WaitEvidence;
				this->depthRecoveryPhaseStartSim_ = ros::Time::now().toSec();
				this->publishDepthRecoveryEvent("ACTION_REACHED");
			} else if (this->trajectoryReady_ &&
			           (this->hasCollision() || this->hasDynamicCollision())) {
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				this->depthRecoveryPhase_ = DepthRecoveryPhase::Select;
				this->publishDepthRecoveryEvent("ACTION_REJECTED", "collision_change");
			}
			return true;
		}

		if (this->depthRecoveryAttempts_ >= 2) {
			this->depthRecoveryPhase_ = DepthRecoveryPhase::Blocked;
			this->publishDepthRecoveryEvent("RECOVERY_BLOCKED", "attempt_budget");
			return true;
		}
		const auto decisionStart = std::chrono::steady_clock::now();
		const Eigen::Vector3d current(this->odom_.pose.pose.position.x,
		                             this->odom_.pose.pose.position.y,
		                             this->odom_.pose.pose.position.z);
		const double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d targetPosition = current;
		double targetYaw = yaw;
		if (this->depthEvidenceArm_ == "D") {
			targetYaw = this->depthRecoveryBaseYaw_ +
				(this->depthRecoveryAttempts_ == 0 ? M_PI/3.0 : -M_PI/3.0);
		} else {
			const auto snapshot = this->map_->captureSnapshot();
			AngularDepthReliability reliability;
			if (this->depthEvidenceArm_ == "F") {
				std::lock_guard<std::mutex> lock(this->angularReliabilityMutex_);
				reliability = this->angularReliability_;
			}
			DepthIntrinsics camera;
			camera.fx = this->depthFx_; camera.fy = this->depthFy_;
			camera.cx = this->depthCx_; camera.cy = this->depthCy_;
			Eigen::Vector3d robotSize;
			this->map_->getRobotSize(robotSize);
			std::size_t rejectedByInflatedMap = 0;
			std::size_t rejectedByBodyEvidence = 0;
			const auto safeConnection = [&](const Eigen::Vector3d& target) {
				if ((target - current).norm() < 1e-6) {
					if (this->map_->isInflatedFree(current)) return true;
					++rejectedByInflatedMap;
					return false;
				}
				if (!this->map_->isInflatedFreeLine(current, target)) {
					++rejectedByInflatedMap;
					return false;
				}
				const auto hasEvidence = [&](const Eigen::Vector3d& point) {
					const bool startPrior =
						((point - this->initialClearCenter_).array().abs() <=
						 this->freeRange_.array()).all();
					return this->map_->isFree(point) &&
					       (startPrior || this->map_->hasValidDepthEvidence(point));
				};
				const bool missing = firstMissingBodyEvidence(
					{current, target}, robotSize / 2.0, 0.5, 0.05, hasEvidence).found;
				if (missing) ++rejectedByBodyEvidence;
				return !missing;
			};
			const auto alreadyTried = [&](const Eigen::Vector3d& pos, double candidateYaw) {
				// A failed in-place scan should not consume the final action
				// budget on another scan when a safe translation exists.
				if (this->depthRecoveryLastActionInPlace_ &&
				    (pos - current).norm() < 0.05) return true;
				for (const auto& previous : this->depthTriedActions_)
					if ((previous.first-pos).norm() < 0.05 &&
					    std::abs(shortestAngleDifference(previous.second, candidateYaw)) < 0.05)
						return true;
				return false;
			};
			const auto action = selectObservationRecovery(
				current, yaw, this->depthBodyToCamera_, this->depthObligation_,
				camera, snapshot, reliability, this->depthEvidenceArm_ == "E",
				safeConnection, alreadyTried);
			std_msgs::String diagnostic;
			std::ostringstream diagnosticJson;
			diagnosticJson << std::fixed << std::setprecision(3)
			               << "{\"schema_version\":1,\"event\":\"CANDIDATE_DIAGNOSTIC\""
			               << ",\"arm\":\"" << this->depthEvidenceArm_ << "\""
			               << ",\"sim_time\":" << ros::Time::now().toSec()
			               << ",\"obligation\":[" << this->depthObligation_.x() << ","
			               << this->depthObligation_.y() << "," << this->depthObligation_.z() << "]"
			               << ",\"inspected\":" << action.inspectedCandidates
			               << ",\"safe_positions\":" << action.safePositions
			               << ",\"rejected_connection\":" << action.rejectedConnection
			               << ",\"rejected_inflated_map_positions\":" << rejectedByInflatedMap
			               << ",\"rejected_body_evidence_positions\":" << rejectedByBodyEvidence
			               << ",\"rejected_tried\":" << action.rejectedTried
			               << ",\"rejected_range\":" << action.rejectedRange
			               << ",\"rejected_fov\":" << action.rejectedFov
			               << ",\"rejected_map_bounds\":" << action.rejectedMapBounds
			               << ",\"rejected_unknown_ray\":" << action.rejectedUnknownRay
			               << ",\"rejected_occupied_ray\":" << action.rejectedOccupiedRay
			               << ",\"rejected_unhealthy_bin\":" << action.rejectedUnhealthyBin
			               << ",\"viable_candidates\":" << action.viableCandidates
			               << ",\"selected\":" << (action.found ? "true" : "false") << "}";
			diagnostic.data = diagnosticJson.str();
			this->depthRecoveryEventPub_.publish(diagnostic);
			if (!action.found) {
				const double decisionMs = std::chrono::duration<double, std::milli>(
					std::chrono::steady_clock::now()-decisionStart).count();
				this->depthRecoveryPhase_ = DepthRecoveryPhase::Blocked;
				this->stop();
				this->publishDepthRecoveryEvent("RECOVERY_BLOCKED", "no_safe_view", decisionMs);
				return true;
			}
			targetPosition = action.bodyPosition;
			targetYaw = action.bodyYaw;
		}
		const double decisionMs = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now()-decisionStart).count();
		++this->depthRecoveryAttempts_;
		this->depthTriedActions_.push_back({targetPosition, targetYaw});
		this->depthRecoveryLastActionInPlace_ = (targetPosition - current).norm() < 0.05;
		if (this->depthRecoveryLastActionInPlace_) {
			this->moveToOrientation(targetYaw, this->desiredAngularVel_);
			this->stop();
			this->depthRecoveryPhase_ = DepthRecoveryPhase::WaitEvidence;
			this->depthRecoveryPhaseStartSim_ = ros::Time::now().toSec();
			this->publishDepthRecoveryEvent("ROTATE_EXECUTED", "", decisionMs);
			return true;
		}
		this->depthRecoveryGoal_.header.frame_id = "map";
		this->depthRecoveryGoal_.pose = this->odom_.pose.pose;
		this->depthRecoveryGoal_.pose.position.x = targetPosition.x();
		this->depthRecoveryGoal_.pose.position.y = targetPosition.y();
		this->depthRecoveryGoal_.pose.position.z = targetPosition.z();
		this->depthRecoveryGoal_.pose.orientation =
			AutoFlight::quaternion_from_rpy(0, 0, targetYaw);
		this->goal_ = this->depthRecoveryGoal_;
		this->depthRecoveryPhase_ = DepthRecoveryPhase::Moving;
		this->replan_ = true;
		this->pendingLocalReplanReason_ = "depth_recovery";
		this->publishDepthRecoveryEvent("MOVE_PLANNING", "", decisionMs);
		return true;
	}

	double dynamicExploration::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			double remainDistance = 0.0;
			for (double t=0.0; t<=this->trajectory_.getDuration(); t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					if (t <= this->trajTime_){
						totalDistance += (currP - prevP).norm();
					}
					else{
						remainDistance += (currP - prevP).norm();
					}
				}
				prevP = currP;
			}
			if (remainDistance <= 1.0){ // no replan when less than 1m
				return -1.0;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicExploration::replanForDynamicObstacle(){
		ros::Time currTime = ros::Time::now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		if (this->useFakeDetector_){
			this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).toSec();
			if (timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
		}
		return replan;
	}

	bool dynamicExploration::reachExplorationGoal(){
		if (this->waypoints_.poses.size() == 0) return false;
		double distThresh = 0.1;
		double yawDiffThresh = 0.1;
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d targetPos (this->waypoints_.poses.back().pose.position.x, this->waypoints_.poses.back().pose.position.y, this->waypoints_.poses.back().pose.position.z);
		double yawTarget = AutoFlight::rpy_from_quaternion(this->waypoints_.poses.back().pose.orientation); 
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		if ((currPos - targetPos).norm() <= distThresh and std::abs(currYaw - yawTarget) >= yawDiffThresh){
			return true;
		}
		return false;
	}

	bool dynamicExploration::isGoalValid(){
		Eigen::Vector3d pGoal (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
		if (this->returningHome_) return this->map_->isInflatedFree(pGoal);
		if (this->map_->isInflatedOccupied(pGoal)){
			return false;
		}
		else{
			return true;
		}
	}

	nav_msgs::Path dynamicExploration::getCurrentTraj(double dt){
		nav_msgs::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = ros::Time::now();
	
		if (this->trajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos, double yaw){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, direction) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	void dynamicExploration::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
		std::vector<onboardDetector::box3D> obstacles;
		this->detector_->getObstaclesInSensorRange(PI_const, obstacles);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}


	void dynamicExploration::waitTime(double time){
		ros::Rate r (30);
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		double passtime = 0.0;
		while (ros::ok() and passtime < time){
			currTime = ros::Time::now();
			passtime = (currTime - startTime).toSec();
			ros::spinOnce();
			r.sleep();
		}
	}
}
