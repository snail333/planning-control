#ifndef LIBWAYPOINT_FOLLOWER_PURE_PURSUIT_H
#define LIBWAYPOINT_FOLLOWER_PURE_PURSUIT_H

#define EIGEN_MPL2_ONLY

// ROS includes
#include <geometry_msgs/Pose.h>

// C++ includes
#include <memory>
#include <utility>
#include <vector>

class PurePursuit
{
public:
  PurePursuit() : use_lerp_(false), lookahead_distance_(0.0), clst_thr_dist_(3.0), clst_thr_ang_(M_PI/4) {}

  // setter
  void setUseLerp(bool ul);
  void setCurrentPose(const geometry_msgs::Pose &msg);
  void setWaypoints(const std::vector<geometry_msgs::Pose> &msg);
  void setLookaheadDistance(double ld);
  void setClosestThreshold(double clst_thr_dist, double clst_thr_ang);

  // getter
  geometry_msgs::Point getLocationOfNextWaypoint();
  geometry_msgs::Point getLocationOfNextTarget();

  bool isRequirementsSatisfied();
  std::pair<bool, double> run();  // calculate curvature

private:
  // variables for debug
  geometry_msgs::Point loc_next_wp_;
  geometry_msgs::Point loc_next_tgt_;

  // variables got from outside
  bool use_lerp_;
  double lookahead_distance_, clst_thr_dist_, clst_thr_ang_;
  std::shared_ptr<std::vector<geometry_msgs::Pose>> curr_wps_ptr_;
  std::shared_ptr<geometry_msgs::Pose> curr_pose_ptr_;

  // functions
  int32_t findNextPointIdx(int32_t search_start_idx);
  std::pair<bool, geometry_msgs::Point> lerpNextTarget(int32_t next_wp_idx);
};

#endif  // LIBWAYPOINT_FOLLOWER_PURE_PURSUIT_H
