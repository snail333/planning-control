#ifndef PURE_PURSUIT_PURE_PURSUIT_H
#define PURE_PURSUIT_PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// C++ includes
#include <vector>

// User defined includes
#include <common_msgs/Trajectory.h>
#include <common_msgs/localization.h>
#include <libtrajectory_follower/libtrajectory_follower.h>

namespace waypoint_follower
{
class PurePursuit
{
public:
  PurePursuit();
  ~PurePursuit();

  // for setting data
  void setLookaheadDistance(const double& ld)
  {
    lookahead_distance_ = ld;
  }
  void setMinimumLookaheadDistance(const double& minld)
  {
    minimum_lookahead_distance_ = minld;
  }
  void setCurrentVelocity(const double& cur_vel)
  {
    current_linear_velocity_ = cur_vel;
  }
  void setCurrentWaypoints(const std::vector<common_msgs::TrajectoryPoint>& wps)
  {
    current_waypoints_ = wps;
  }
  void setCurrentPose(const common_msgs::localizationConstPtr& msg)
  {
    current_pose_.position.x = msg->pose.position.x;
    current_pose_.position.y = msg->pose.position.y;
    current_pose_.position.z = msg->pose.position.z;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, msg->pose.rotation.yaw);
    quaternionTFToMsg(quaternion, current_pose_.orientation);
  }
  void setLinearInterpolationParameter(const bool& param)
  {
    is_linear_interpolation_ = param;
  }

  // for debug on ROS
  geometry_msgs::Point getPoseOfNextWaypoint() const
  {
    geometry_msgs::Point nextwp;
    nextwp.x = current_waypoints_.at(next_waypoint_number_).x;
    nextwp.y = current_waypoints_.at(next_waypoint_number_).y;
    return nextwp;
  }
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return next_target_position_;
  }
  geometry_msgs::Pose getCurrentPose() const
  {
    return current_pose_;
  }
  std::vector<common_msgs::TrajectoryPoint> getCurrentWaypoints() const
  {
    return current_waypoints_;
  }
  double getLookaheadDistance() const
  {
    return lookahead_distance_;
  }
  double getMinimumLookaheadDistance() const
  {
    return minimum_lookahead_distance_;
  }
  // processing
  bool canGetCurvature(double* output_kappa);

private:
  // constant
  const double RADIUS_MAX_;
  const double KAPPA_MIN_;

  // variables
  bool is_linear_interpolation_;
  int next_waypoint_number_;
  geometry_msgs::Point next_target_position_;
  double lookahead_distance_;
  double minimum_lookahead_distance_;
  geometry_msgs::Pose current_pose_;
  double current_linear_velocity_;
  std::vector<common_msgs::TrajectoryPoint> current_waypoints_;

  // functions
  double calcCurvature(geometry_msgs::Point target) const;
  bool interpolateNextTarget(
    int next_waypoint, geometry_msgs::Point* next_target) const;
  void getNextWaypoint();
};
}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_H
