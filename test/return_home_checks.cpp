#include <autonomous_flight/simulation/completionGate.h>
#include <global_planner/dep.h>
#include <stdexcept>

using Eigen::Vector3d;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
  std::cout << "PASS: " << message << std::endl;
}

class TestMap : public mapManager::occMap {
 public:
  TestMap() {
    mapRes_ = 0.1; mapSizeMin_ = Vector3d(-4,-4,0); mapSizeMax_ = Vector3d(4,4,2);
    mapVoxelMin_ = Eigen::Vector3i::Zero(); mapVoxelMax_ = Eigen::Vector3i(80,80,20);
    pMinLog_ = -2; pOccLog_ = 1; pMaxLog_ = 2;
    robotSize_ = Vector3d::Zero();
    occupancy_.assign(80*80*20, pMinLog_);
    occupancyInflated_.assign(80*80*20, false);
  }
  void wall(double halfWidthY) {
    for (int x=0; x<80; ++x) for (int y=0; y<80; ++y) for (int z=0; z<20; ++z) {
      Eigen::Vector3i idx(x,y,z); Vector3d p; indexToPos(idx,p);
      if (std::abs(p.x())<0.25 && std::abs(p.y())<halfWidthY) {
        occupancy_[indexToAddress(idx)] = 2; occupancyInflated_[indexToAddress(idx)] = true;
      }
    }
  }
  void unknown(const Vector3d& p) { occupancy_[posToAddress(p)] = pMinLog_ - UNKNOWN_FLAG_; }
};

namespace globalPlanner {
struct ReturnHomeTestAccess {
  static void graph(DEP& planner, const std::shared_ptr<TestMap>& map,
                    const Vector3d& position, const std::vector<Vector3d>& points) {
    planner.map_ = map; planner.position_ = position; planner.odomReceived_ = true;
    for (auto node : planner.prmNodeVec_) node->adjNodes.clear();
    planner.prmNodeVec_.clear(); planner.maxConnectDist_ = 0.6;
    planner.globalRegionMin_ = Vector3d(-3,-3,0.7);
    planner.globalRegionMax_ = Vector3d(3,3,1.2);
    std::shared_ptr<PRM::Node> previous;
    for (auto p : points) {
      auto node = std::make_shared<PRM::Node>(p); planner.prmNodeVec_.insert(node);
      if (previous) { previous->adjNodes.insert(node); node->adjNodes.insert(previous); }
      previous = node;
    }
  }
  static void insert(DEP& planner, const std::shared_ptr<PRM::Node>& node) {
    planner.prmNodeVec_.insert(node);
  }
};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "return_home_checks", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  try {
    AutoFlight::CompletionGate gate(3,5);
    require(!gate.observe(true,true,true,1,0), "one zero-gain result does not complete");
    require(!gate.observe(true,true,true,1,6), "repeated depth frame does not complete");
    require(!gate.observe(true,true,true,2,7), "two confirmations do not complete");
    require(gate.observe(true,true,true,3,8), "three fresh confirmations across dwell complete");
    gate.reset();
    gate.observe(true,true,true,4,10); gate.observe(true,true,true,5,11);
    require(!gate.observe(false,true,true,6,20), "planning failure resets completion");
    require(!gate.observe(true,true,true,7,21), "post-failure observation starts new dwell");
    require(!gate.observe(true,true,false,8,30), "stale sensor cannot complete");
    gate.observe(true,true,true,9,31);
    require(!gate.observe(true,false,true,10,40), "remaining information resets completion");
    gate.observe(true,true,true,11,41);
    require(!gate.observe(true,true,true,12,1), "clock reset cannot complete");

    globalPlanner::DEP seededA(nh), seededB(nh), seededC(nh);
    seededA.setRandomSeed(42); seededB.setRandomSeed(42); seededC.setRandomSeed(43);
    std::vector<int> sequenceA, sequenceB, sequenceC;
    for (int i=0; i<32; ++i) {
      sequenceA.push_back(seededA.weightedSample({1,2,3,4}));
      sequenceB.push_back(seededB.weightedSample({1,2,3,4}));
      sequenceC.push_back(seededC.weightedSample({1,2,3,4}));
    }
    require(sequenceA == sequenceB, "same DEP seed reproduces weighted sampling");
    require(sequenceA != sequenceC, "different DEP seed changes weighted sampling");

    globalPlanner::DEP planner(nh);
    auto map = std::make_shared<TestMap>();
    const Vector3d start(-2.3,0,1), home(2.3,0,1);
    const std::vector<Vector3d> nodes{{-2,0,1},{-2,2,1},{0,2,1},{2,2,1},{2,0,1}};
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,start,nodes);
    nav_msgs::Path path;
    require(planner.planReturnPath(home,path) && path.poses.size()==2, "known-free direct return");
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,home,nodes);
    require(planner.planReturnPath(home,path) && path.poses.size()==1, "already home avoids zero-length line query");
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,start,nodes);
    map->wall(1.0);
    require(planner.planReturnPath(home,path) && path.poses.size()>2, "blocked straight line uses PRM detour");
    for (size_t i=1; i<path.poses.size(); ++i) {
      const auto& a=path.poses[i-1].pose.position; const auto& b=path.poses[i].pose.position;
      require(map->isInflatedFreeLine(Vector3d(a.x,a.y,a.z),Vector3d(b.x,b.y,b.z)), "returned segment is known free");
    }
    map->wall(4.0);
    require(!planner.planReturnPath(home,path) && path.poses.empty(), "fully blocked return fails safely");
    auto unknownMap = std::make_shared<TestMap>(); unknownMap->unknown(home);
    globalPlanner::ReturnHomeTestAccess::graph(planner,unknownMap,start,nodes);
    require(!planner.planReturnPath(home,path), "unknown home cannot be used");
    require(!planner.planReturnPath(Vector3d(10,0,1),path), "out-of-map home rejected");

    map = std::make_shared<TestMap>();
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,start,nodes);
    int checked = 0;
    require(planner.reachableGainExhausted(0,checked) && checked==5, "all reachable nodes freshly checked for exhaustion");
    // Off the roadmap edge: the informative node must remain reachable.
    map->unknown(Vector3d(2.55,0.55,1.05));
    require(!planner.reachableGainExhausted(0,checked), "information outside selected goals prevents completion");
    require(planner.reachableGainExhausted(500,checked), "configured low-gain threshold permits near-completion");
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,start,{});
    require(!planner.reachableGainExhausted(0,checked), "empty roadmap is not completion");

    // The closest roadmap node is isolated, while a second safe current-pose
    // connector reaches the requested goal. The old single-nearest start fails.
    map = std::make_shared<TestMap>();
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,Vector3d(0,0,1),{});
    auto isolated = std::make_shared<PRM::Node>(Vector3d(0.1,0,1));
    auto connector = std::make_shared<PRM::Node>(Vector3d(0.3,0.2,1));
    auto requested = std::make_shared<PRM::Node>(Vector3d(0.8,0.2,1));
    connector->adjNodes.insert(requested); requested->adjNodes.insert(connector);
    globalPlanner::ReturnHomeTestAccess::insert(planner,isolated);
    globalPlanner::ReturnHomeTestAccess::insert(planner,connector);
    globalPlanner::ReturnHomeTestAccess::insert(planner,requested);
    std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidates;
    require(planner.findCandidatePath({requested},candidates) && !candidates.empty(),
            "multiple safe current-pose connectors avoid nearest-node stranding");

    // All globally selected goals are disconnected. A reachable component must
    // still produce recovery candidates instead of remaining blocked forever.
    globalPlanner::ReturnHomeTestAccess::graph(planner,map,Vector3d(-2.3,0,1),
                                               {{-2,0,1},{-1.5,0,1}});
    auto unreachable = std::make_shared<PRM::Node>(Vector3d(2,0,1));
    globalPlanner::ReturnHomeTestAccess::insert(planner,unreachable);
    map->unknown(Vector3d(-1.45,0.25,1.05));
    candidates.clear();
    require(planner.findCandidatePath({unreachable},candidates) && !candidates.empty() &&
            candidates.front().back() != unreachable,
            "unreachable global goals fall back to the reachable component");
    std::cout << "ALL RETURN-HOME CHECKS PASSED" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << std::endl; return 1;
  }
  return 0;
}
