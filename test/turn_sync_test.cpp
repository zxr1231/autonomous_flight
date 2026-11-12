#include <gtest/gtest.h>
#include <autonomous_flight/simulation/turnSync.h>
using AutoFlight::TurnSchedule;
TEST(TurnSync, WrappedShortestAndRestEndpoints){
 auto s=TurnSchedule::make(3.1,-3.1,.5);
 EXPECT_LT(s.delta,.1);EXPECT_GT(s.delta,0);
 EXPECT_NEAR(TurnSchedule::wrap(s.yaw(s.duration)+3.1),0,1e-10);
 const double h=1e-6;
 EXPECT_NEAR(TurnSchedule::wrap(s.yaw(h)-s.yaw(0))/h,0,1e-5);
 EXPECT_NEAR(TurnSchedule::wrap(s.yaw(s.duration)-s.yaw(s.duration-h))/h,0,1e-5);
}
TEST(TurnSync, RateBoundAndDelay){
 auto s=TurnSchedule::make(0,2,.5);s.delay=.5*s.duration;
 for(double t=0;t<s.duration;t+=.001)
  EXPECT_LE(std::abs(TurnSchedule::wrap(s.yaw(t+.001)-s.yaw(t)))/.001,.50001);
 EXPECT_EQ(s.motionTime(.1),0);EXPECT_NEAR(s.motionTime(s.delay+1),1,1e-12);
 EXPECT_NEAR(s.yaw(s.duration+5),2,1e-12);
}
TEST(TurnSync, RetainsReferenceNotExtraCount){
 std::unordered_set<int> a{1,2},b{3,4,5,6},c{1,2,3},d;
 EXPECT_EQ(AutoFlight::retainedFraction(a,b),0);
 EXPECT_EQ(AutoFlight::retainedFraction(a,c),1);
 EXPECT_EQ(AutoFlight::retainedFraction(d,c),1);
 EXPECT_EQ(AutoFlight::retainedFraction(a,d),0);
}
TEST(TurnSync, InvalidAndZeroTurn){
 EXPECT_THROW(TurnSchedule::make(0,1,0),std::invalid_argument);
 EXPECT_THROW(TurnSchedule::make(NAN,1,.5),std::invalid_argument);
 auto s=TurnSchedule::make(1,1,.5);EXPECT_EQ(s.duration,0);EXPECT_DOUBLE_EQ(s.yaw(0),1);
}

TEST(TurnSync, UnknownOccupiedAndInflationFailClosed){
 std::vector<double> occ(125,-1.);std::vector<bool> inflated(125,false);
 AutoFlight::FrozenTurnSafety map(occ,inflated,Eigen::Vector3d::Zero(),Eigen::Vector3i(5,5,5),1.,-2.,0.);
 Eigen::Vector3d p(2.5,2.5,2.5);
 EXPECT_TRUE(map.safe(p,1.,0.));
 occ[2*25+2*5+2]=-3.;EXPECT_FALSE(map.free(p));
 occ[62]=0.;EXPECT_FALSE(map.free(p));
 occ[62]=-1.;inflated[62]=true;EXPECT_FALSE(map.free(p));
 inflated[62]=false;occ[3*25+2*5+2]=-3.;EXPECT_TRUE(map.free(p));EXPECT_FALSE(map.safe(p,1.,0.));
 EXPECT_FALSE(map.free(Eigen::Vector3d(-.01,1,1)));
 EXPECT_FALSE(map.free(Eigen::Vector3d(5,1,1)));
 EXPECT_FALSE(map.free(Eigen::Vector3d(NAN,1,1)));
}
TEST(TurnSync, RetentionThresholdIsReferenceBased){
 std::unordered_set<int> a,b;for(int i=0;i<100;++i)a.insert(i);
 for(int i=0;i<98;++i)b.insert(i);
 EXPECT_DOUBLE_EQ(AutoFlight::retainedFraction(a,b),.98);
 b.erase(97);b.insert(500);EXPECT_DOUBLE_EQ(AutoFlight::retainedFraction(a,b),.97);
}
TEST(TurnSync, RestConstraintOnlyForInsertedHold){
 EXPECT_TRUE(AutoFlight::delayBoundaryValid(0,.07,.2));
 EXPECT_FALSE(AutoFlight::delayBoundaryValid(1,.07,.2));
 EXPECT_TRUE(AutoFlight::delayBoundaryValid(1,.01,.01));
 EXPECT_FALSE(AutoFlight::delayBoundaryValid(0,NAN,0));
 EXPECT_FALSE(AutoFlight::delayBoundaryValid(-1,0,0));
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
