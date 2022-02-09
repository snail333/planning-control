// ROS includes
#include <ros/ros.h>

#include "lane_select_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_select");
  lane_planner::LaneSelectNode lsn;
  lsn.run();

  return 0;
}
