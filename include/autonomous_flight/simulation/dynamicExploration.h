/*
	FILE: dynamicExploration.h
	-----------------------------
	header of dynamic exploration
*/
#ifndef DYNAMIC_EXPLORATION
#define DYNAMIC_EXPLORATION
#include <autonomous_flight/simulation/flightBase.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <autonomous_flight/simulation/completionGate.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <atomic>
#include <mutex>


namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::DEP> expPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer explorationTimer_;
		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;

		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher inputTrajPub_;
		
		// parameters
		bool useFakeDetector_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double wpStablizeTime_;
		bool initialScan_;
		double replanTimeForDynamicObstacle_;
		Eigen::Vector3d freeRange_;
		double reachGoalDistance_;

		// exploration data
		std::atomic<bool> explorationReplan_{true};
		std::atomic<bool> returningHome_{false};
		bool homeReached_ = false;
		bool settlingHome_ = false;
		bool handlingWaypoints_ = false;
		bool returnHomeEnabled_ = true;
		int completionGainThreshold_ = 0;
		double homeTolerance_ = 0.2;
		double homeHoldTime_ = 2.0;
		double homeStillSince_ = -1.0;
		geometry_msgs::PoseStamped homePose_;
		CompletionGate completionGate_;
		ros::Publisher missionStatePub_, homePub_, returnPathPub_, planningEventPub_;
		ros::Subscriber completionDepthSub_;
		std::atomic<double> lastDepthTime_{-1.0};
		std::atomic<uint64_t> depthSequence_{0};
		std::mutex resultMutex_;
		bool resultReady_ = false;
		nav_msgs::Path resultPath_;
		std::string resultState_;
		uint64_t resultGlobalSequence_ = 0;
		std::string missionState_;
		uint64_t localPlanningSequence_ = 0;
		uint64_t returnPlanningSequence_ = 0;
		uint64_t activeGlobalSequence_ = 0;
		bool replan_ = false;
		bool newWaypoints_ = false;
		int waypointIdx_ = 1;
		nav_msgs::Path waypoints_; // latest waypoints from exploration planner
		nav_msgs::Path inputTrajMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		bool trajectoryReady_ = false;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		trajPlanner::bspline trajectory_;
		ros::Time lastDynamicObstacleTime_;
	
	public:
		std::thread exploreReplanWorker_;
		dynamicExploration();
		dynamicExploration(const ros::NodeHandle& nh);

		void initParam();
		void initModules();
		void registerCallback();
		void registerPub();

		void explorationCB(const ros::TimerEvent&);
		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector

		void run();
		void initExplore();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);
		bool hasCollision();
		bool hasDynamicCollision();
		void exploreReplan();
		void setMissionState(const std::string& state);
		void completionDepthCB(const sensor_msgs::ImageConstPtr& image);
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		bool reachExplorationGoal();
		bool isGoalValid();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		nav_msgs::Path getRestGlobalPath(const Eigen::Vector3d& pos);
		nav_msgs::Path getRestGlobalPath(const Eigen::Vector3d& pos, double yaw);
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
		void waitTime(double time);
	};
}
#endif
