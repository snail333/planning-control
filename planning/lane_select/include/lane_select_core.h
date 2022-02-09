#ifndef LANE_SELECT_CORE_H
#define LANE_SELECT_CORE_H

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

// C++ includes
#include <iostream>
#include <numeric>
#include <tuple>

// User defined includes
#include "common_msgs/localization.h"
#include "common_msgs/Pose.h"
#include "common_msgs/TrajectoryArray.h"
// #include "hermite_curve.h"
#include "libtrajectory_follower/libtrajectory_follower.h"

namespace lane_planner
{
enum class ChangeFlag : int32_t
{
  straight,
  right,
  left,

  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

class LaneSelectNode
{
friend class LaneSelectTestClass;

public:
  LaneSelectNode();
  ~LaneSelectNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_, pub2_, pub3_, pub4_;
  ros::Publisher vis_pub1_;

  // subscriber
  ros::Subscriber sub1_, sub2_, sub3_;

  // variables
  int32_t lane_array_id_;
  int32_t current_lane_idx_;  // the index of the lane we are driving
  int32_t right_lane_idx_;
  int32_t left_lane_idx_;
  std::vector<std::tuple<common_msgs::Trajectory, int32_t, ChangeFlag>> tuple_vec_;  // lane, closest_waypoint,
                                                                                 // change_flag
  std::tuple<common_msgs::Trajectory, int32_t, ChangeFlag> lane_for_change_;
  bool is_lane_array_subscribed_, is_current_pose_subscribed_, 
      is_current_state_subscribed_, is_config_subscribed_;

  // parameter from runtime manager
  double distance_threshold_, distance_threshold_neighbor_lanes, lane_change_interval_, lane_change_target_ratio_, 
  lane_change_target_minimum_, vlength_hermite_curve_;
  int search_closest_waypoint_minimum_dt_;

  // topics
  common_msgs::localization current_pose_;
  double current_velocity_;
  std::string current_state_;

  common_msgs::Trajectory left_waypoints, right_waypoints, back_waypoints;

  // callbacks
  void callbackFromLaneArray(const common_msgs::TrajectoryArrayConstPtr& msg);
  void callbackFromPoseStamped(const common_msgs::localizationConstPtr& msg);
  //void callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr& msg);

  // initializer
  void initForROS();
  void initForLaneSelect();

  // functions
  void resetLaneIdx();
  void resetSubscriptionFlag();
  bool isAllTopicsSubscribed();
  void processing();
  void publishLane(const common_msgs::Trajectory& lane);
  void publishLaneID(const common_msgs::Trajectory& lane);
  void publishClosestWaypoint(const int32_t clst_wp);
  void publishChangeFlag(const ChangeFlag flag);
  void publishVehicleLocation(const int32_t clst_wp, const int32_t larray_id);
  bool getClosestWaypointNumberForEachLanes();
  int32_t findMostClosestLane(const std::vector<uint32_t> idx_vec, const common_msgs::Position p);
  void findCurrentLane();
  void findNeighborLanes();
  void changeLane();
  void updateChangeFlag();
  void createLaneForChange();
  int32_t getClosestLaneChangeWaypointNumber(const std::vector<common_msgs::TrajectoryPoint>& wps, int32_t cl_wp);

  // spinOnce for test
  void spinOnce()
  {
    ros::spinOnce();
  }
};

int32_t getClosestWaypointNumber(const common_msgs::Trajectory &current_lane, const common_msgs::Pose &current_pose,
                                 const double &current_velocity, const int32_t previous_number,
                                 const double distance_threshold, const int search_closest_waypoint_minimum_dt);

double getTwoDimensionalDistance(const common_msgs::Position &target1, const common_msgs::TrajectoryPoint &target2);

geometry_msgs::Point convertPointIntoRelativeCoordinate(const geometry_msgs::Point& input_point,
                                                        const geometry_msgs::Pose& pose);

geometry_msgs::Point convertPointIntoWorldCoordinate(const geometry_msgs::Point& input_point,
                                                     const geometry_msgs::Pose& pose);
double getRelativeAngle(const geometry_msgs::Pose& waypoint_pose, const geometry_msgs::Pose& current_pose);
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c);
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double sa, double b, double c);
}
#endif  // LANE_SELECT_CORE_H
