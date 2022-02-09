#ifndef VELOCITY_SET_PATH_H
#define VELOCITY_SET_PATH_H

#include <common_msgs/Trajectory.h>
#include <common_msgs/localization.h>
#include <common_msgs/sensorgps.h>
#include <libtrajectory_follower/libtrajectory_follower.h>
#include "velocity_set_info.h"

class VelocitySetPath
{
 private:
  common_msgs::Trajectory base_waypoints_;
  common_msgs::Trajectory prev_waypoints_;
  common_msgs::Trajectory new_waypoints_;
  common_msgs::Trajectory temporal_waypoints_;
  common_msgs::localization pose_global;
  bool set_path_;
  bool closest_waypoint_initialized_;
  double current_vel_;

  ros::WallTime waypoint_t1 = ros::WallTime::now();
  ros::WallTime waypoint_t2 = ros::WallTime::now();
  // ROS param
  double velocity_offset_; // m/s
  double decelerate_vel_min_; // m/s
  int closest_waypoint_index_;
  int closest_local_index_;
  int closest_search_size_; 
  int safety_waypoints_size_;

  bool checkWaypoint(int num, const char *name) const;

 public:
  VelocitySetPath();
  ~VelocitySetPath();
  double calcChangedVelocity(const double& current_vel, const double& accel, const std::array<int, 2>& range) const;
  void changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint, double deceleration);
  void avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint);
  void avoidSuddenAcceleration(double acceleration, int closest_waypoint);
  void changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint);
  void setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, common_msgs::localization control_pose);
  void initializeNewWaypoints();
  void resetFlag();
  int getLocalClosestWaypoint(const common_msgs::Trajectory& waypoints, const geometry_msgs::Pose& pose, const int& search_size);
  common_msgs::Trajectory localWaypoints(const common_msgs::Trajectory& current_waypoints);

  // ROS Callbacks
  void waypointsCallback(const common_msgs::TrajectoryConstPtr& msg);
  void closestWaypointCallback(const std_msgs::Int32& msg);
  void currentPoseCallback(const common_msgs::localizationConstPtr &msg);

  double calcInterval(const int begin, const int end) const;

  common_msgs::Trajectory getPrevWaypoints() const
  {
    return prev_waypoints_;
  }

  common_msgs::Trajectory getNewWaypoints() const
  {
    return new_waypoints_;
  }

  common_msgs::Trajectory getTemporalWaypoints() const
  {
    return temporal_waypoints_;
  }

  bool getSetPath() const
  {
    return set_path_;
  }

  double getCurrentVelocity() const
  {
    return current_vel_;
  }

  int getPrevWaypointsSize() const
  {
    return prev_waypoints_.points.size();
  }

  int getNewWaypointsSize() const
  {
    return new_waypoints_.points.size();
  }
};

#endif // VELOCITY_SET_PATH_H
