#ifndef PURE_PURSUIT_PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_PURE_PURSUIT_CORE_H

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

// User defined includes
// #include <common_msgs/ControlCommandStamped.h>
#include <common_msgs/Trajectory.h>
#include <common_msgs/SystemOverall.h>
#include <pid_pursuit/pure_pursuit.h>
#include <pid_pursuit/pure_pursuit_viz.h>
#include <pid_pursuit/pid_control.h>


#include <vector>
#include <math.h>
#include <memory>

namespace waypoint_follower
{
enum class Mode : int32_t
{
  waypoint,
  dialog,

  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

class PurePursuitNode
{
public:
  PurePursuitNode();
  ~PurePursuitNode();

  void run();
  friend class PurePursuitNodeTestSuite;

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;


  // class
  PurePursuit pp_;
  LongitudinalControl pid_;

  // publisher
  ros::Publisher pub1_,
    pub11_, pub12_, pub13_, pub14_, pub15_, pub16_, pub17_, pub18_, pub19_;

  // subscriber
  ros::Subscriber sub1_, sub2_, sub3_, sub4_;

  // constant
  int LOOP_RATE_;  // processing frequency

  // variables
  bool is_linear_interpolation_, publishes_for_steering_robot_,
    add_virtual_end_waypoints_;
  bool is_waypoint_set_, is_pose_set_, is_velocity_set_;
  double current_linear_velocity_, command_linear_velocity_;
  double wheel_base_, driver_ratio_;
  int expand_size_;
  LaneDirection direction_;
  int32_t velocity_source_;          // 0 = waypoint, 1 = Dialog
  double const_lookahead_distance_;  // meter
  double const_velocity_;            // km/h
  double lookahead_distance_ratio_a, lookahead_distance_ratio_b, lookahead_distance_ratio_c;
  // the next waypoint must be outside of this threshold.
  double minimum_lookahead_distance_;

  // callbacks
  void callbackFromCurrentPose(const common_msgs::localizationConstPtr& msg);
  // void callbackFromCurrentVelocity(
  //   const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackFromWayPoints(const common_msgs::TrajectoryConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  // void publishTwistStamped(
  //   const bool& can_get_curvature, const double& kappa) const;
  void publishControlCommandStamped(const bool& can_get_curvature, const double& kappa, 
  const double& lon_torque, const double& lon_accel) const;
  void publishDeviationCurrentPosition(
    const geometry_msgs::Point& point,
    const std::vector<common_msgs::TrajectoryPoint>& waypoints) const;
  void connectVirtualLastWaypoints(
    common_msgs::Trajectory* expand_lane, LaneDirection direction);

  int getSgn() const;
  double computeLookaheadDistance() const;
  double computeCommandVelocity() const;
  double computeCommandAccel() const;
  double computeAngularGravity(double velocity, double kappa) const;
};

double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa, const double& driver_ratio);

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_CORE_H
