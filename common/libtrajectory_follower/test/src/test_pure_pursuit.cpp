#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <autoware_msgs/Lane.h>
#include "libtrajectory_follower/pure_pursuit.h"
#include "libtrajectory_follower/libtrajectory_follower.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
  }
  ~TestSuite()
  {
  }
};

TEST_F(TestSuite, PurePursuit_is_requirements_safisfied)
{
  PurePursuit pp;
  ASSERT_EQ(false, pp.isRequirementsSatisfied());

  geometry_msgs::PoseStampedConstPtr pose_ptr(new geometry_msgs::PoseStamped());
  autoware_msgs::LaneConstPtr wps_ptr(new autoware_msgs::Lane());
  pp.setCurrentPose(pose_ptr->pose);
  pp.setWaypoints(extractPoses(*wps_ptr));
  ASSERT_EQ(true, pp.isRequirementsSatisfied());
}

TEST_F(TestSuite, PurePursuit_run)
{
  PurePursuit pp;
  auto res = pp.run();
  ASSERT_EQ(false, res.first);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
