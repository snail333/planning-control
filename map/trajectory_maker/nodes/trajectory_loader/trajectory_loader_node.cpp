// ROS Includes
#include <ros/ros.h>

#include "trajectory_loader_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_loader");
  trajectory_maker::WaypointLoaderNode wln;
  wln.run();

  return 0;
}
