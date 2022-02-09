#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_lane_select.hpp"

namespace lane_planner {

class LaneSelectTestSuite : public ::testing::Test {
public:
  LaneSelectTestSuite() {}
  ~LaneSelectTestSuite() {}

  LaneSelectTestClass test_obj_;

protected:
  virtual void SetUp() { test_obj_.lsn = new LaneSelectNode(); };
  virtual void TearDown() { delete test_obj_.lsn; };
};

TEST_F(LaneSelectTestSuite, publishVehicleLocation) {
  ASSERT_EQ(test_obj_.vehicle_location_sub.getNumPublishers(), 1U)
      << "No publisher exist!";

  std::array<std::pair<std::string, int>, 2> driving_direction =
  {
    std::make_pair("Forward", 1),
    std::make_pair("Backward", -1)
  };
  for (const auto& dir : driving_direction)
  {
    test_obj_.publishTrafficWaypointsArray(dir.second);
    test_obj_.publishCurrentPose(-0.5 * dir.second, 0.0, 0.0);
    test_obj_.publishCurrentVelocity(0);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
    test_obj_.lsnSpinOnce();
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();

    ASSERT_EQ(0, test_obj_.cb_vehicle_location.waypoint_index)
        << dir.first << "Waypoint index does not match."
        << "It should be 0";
    ASSERT_EQ(test_obj_.lane_array_id_,
              test_obj_.cb_vehicle_location.lane_array_id)
        << dir.first << "LaneArray id does not match."
        << "It should be " << test_obj_.lane_array_id_;

    test_obj_.publishCurrentPose(0.5 * dir.second, 0.0, 0.0);
    test_obj_.publishCurrentVelocity(0);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
    test_obj_.lsnSpinOnce();
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();

    ASSERT_EQ(1, test_obj_.cb_vehicle_location.waypoint_index)
        << dir.first << "Waypoint index does not match."
        << "It should be 1";
  }
}

} // namespace lane_planner

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LaneSelectTestNode");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
