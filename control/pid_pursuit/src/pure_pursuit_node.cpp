// ROS Includes
#include <ros/ros.h>

// User defined includes
#include <pid_pursuit/pure_pursuit_core.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_pursuit");
  waypoint_follower::PurePursuitNode ppn;
  ppn.run();

  return 0;
}
