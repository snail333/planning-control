#include <vector>
#include <pid_pursuit/pure_pursuit_core.h>

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_()
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , current_linear_velocity_(0)
  , command_linear_velocity_(0)
  , direction_(LaneDirection::Forward)
  , velocity_source_(-1)
  , const_lookahead_distance_(4.0)
  , const_velocity_(5.0)
  , lookahead_distance_ratio_a(2.0)
  , lookahead_distance_ratio_b(2.0)
  , lookahead_distance_ratio_c(2.0)
  , minimum_lookahead_distance_(6.0)
{
  initForROS();
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("velocity_source", velocity_source_, 0);
  private_nh_.param("loop_rate", LOOP_RATE_, 30);
  private_nh_.param("is_linear_interpolation", is_linear_interpolation_, true);
  private_nh_.param(
    "publishes_for_steering_robot", publishes_for_steering_robot_, false);
  private_nh_.param(
    "add_virtual_end_waypoints", add_virtual_end_waypoints_, false);
  private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
  private_nh_.param("const_velocity", const_velocity_, 5.0);
  private_nh_.param("lookahead_ratio_a", lookahead_distance_ratio_a, 2.0);
  private_nh_.param("lookahead_ratio_b", lookahead_distance_ratio_b, 2.0);
  private_nh_.param("lookahead_ratio_c", lookahead_distance_ratio_c, 2.0);
  private_nh_.param("minimum_lookahead_distance", minimum_lookahead_distance_, 6.0);
  private_nh_.param("vehicle_info/wheel_base", wheel_base_, 0.85);
  private_nh_.param("vehicle_info/driver_ratio", driver_ratio_, 15.4);

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 10,
    &PurePursuitNode::callbackFromWayPoints, this);
  sub2_ = nh_.subscribe("localization", 10,
    &PurePursuitNode::callbackFromCurrentPose, this);
  // sub4_ = nh_.subscribe("current_velocity", 10,
  //   &PurePursuitNode::callbackFromCurrentVelocity, this);

  // setup publisher
  //pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  pub1_ = nh_.advertise<common_msgs::SystemOverall>("control", 10);
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  // debug tool
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);
  pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
  pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
  pub18_ = nh_.advertise<visualization_msgs::Marker>("expanded_waypoints_mark", 0);
  pub19_ = nh_.advertise<std_msgs::Float32>("twist_mark", 0);
}

void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!is_pose_set_ || !is_waypoint_set_)
    {
      // ROS_WARN("Necessary topics are not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());
    pp_.setMinimumLookaheadDistance(minimum_lookahead_distance_);

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);

    double accelation = 0;
    accelation = pid_.Speed_PID_Acc();
    
    if((fabs(kappa) > 0.1 || fabs(current_linear_velocity_) < 1) && accelation > 0.8)
      accelation = 0.8;

    double torque = 0;
    torque = pid_.Speed_PID_AccAndTorque(accelation);

    // publishTwistStamped(can_get_curvature, kappa);
    publishControlCommandStamped(can_get_curvature, kappa, torque, accelation);

    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(
      pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(
          pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
    if (add_virtual_end_waypoints_)
    {
      pub18_.publish(
        displayExpandWaypoints(pp_.getCurrentWaypoints(), expand_size_));
    }
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data =
      computeAngularGravity(computeCommandVelocity(), kappa);
    pub16_.publish(angular_gravity_msg);

    publishDeviationCurrentPosition(
      pp_.getCurrentPose().position, pp_.getCurrentWaypoints());

    is_pose_set_ = false;
    is_waypoint_set_ = false;

    loop_rate.sleep();
  }
}

// void PurePursuitNode::publishTwistStamped(
//   const bool& can_get_curvature, const double& kappa) const
// {
//   geometry_msgs::TwistStamped ts;
//   ts.header.stamp = ros::Time::now();
//   ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
//   //ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
//   ts.twist.angular.z = can_get_curvature ? atan(kappa * wheel_base_) * 180 / M_PI: 0;
//   pub1_.publish(ts);
//   std_msgs::Float32 msg;
//   msg.data = computeCommandVelocity() * 3.6;
//   pub19_.publish(msg);
// }

void PurePursuitNode::publishControlCommandStamped(const bool& can_get_curvature, 
const double& kappa, const double& lon_torque, const double& lon_accel) const
{
  common_msgs::SystemOverall ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.systemstate.latcontrol.targetangle = 
      can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa, driver_ratio_): 0;
  ccs.systemstate.latcontrol.targettorque = 0;
  ccs.systemstate.loncontrol.targetspeed = can_get_curvature ? computeCommandVelocity() : 0;
  //ccs.systemstate.loncontrol.targetaccelation = can_get_curvature ? computeCommandAccel() : 0;
  ccs.systemstate.loncontrol.targetaccelation = can_get_curvature ? lon_accel : 0;
  ccs.systemstate.loncontrol.targettorque = can_get_curvature ? lon_torque : 0;
  pub1_.publish(ccs);

  std_msgs::Float32 msg;
  msg.data = computeCommandVelocity() * 3.6;
  pub19_.publish(msg);
}

double PurePursuitNode::computeLookaheadDistance() const
{
  if (velocity_source_ == enumToInteger(Mode::dialog))
  {
    return const_lookahead_distance_;
  }

  double maximum_lookahead_distance = current_linear_velocity_ * 10;
  double ld = lookahead_distance_ratio_a * pow(current_linear_velocity_, 2) + 
  lookahead_distance_ratio_b * current_linear_velocity_ + lookahead_distance_ratio_c ;
  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ :
    ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

int PurePursuitNode::getSgn() const
{
  int sgn = 0;
  if (direction_ == LaneDirection::Forward)
  {
    sgn = 1;
  }
  else if (direction_ == LaneDirection::Backward)
  {
    sgn = -1;
  }
  return sgn;
}

double PurePursuitNode::computeCommandVelocity() const
{
  if (velocity_source_ == enumToInteger(Mode::dialog))
  {
    return getSgn() * kmph2mps(const_velocity_);
  }

  return command_linear_velocity_;
}

double PurePursuitNode::computeCommandAccel() const
{
  const geometry_msgs::Pose current_pose = pp_.getCurrentPose();
  geometry_msgs::Pose target_pose;
  target_pose.position.x = pp_.getCurrentWaypoints().at(1).x;
  target_pose.position.y = pp_.getCurrentWaypoints().at(1).y;

  // v^2 - v0^2 = 2ax
  const double x =
      std::hypot(current_pose.position.x - target_pose.position.x,
        current_pose.position.y - target_pose.position.y);
  const double v0 = current_linear_velocity_;
  const double v = computeCommandVelocity();

  const double a = getSgn() * (v * v - v0 * v0) / (2 * x);
  return a;
}

double PurePursuitNode::computeAngularGravity(
  double velocity, double kappa) const
{
  const double gravity = 9.80665;
  return (velocity * velocity) / (1.0 / kappa * gravity);
}


void PurePursuitNode::publishDeviationCurrentPosition(
  const geometry_msgs::Point& point,
  const std::vector<common_msgs::TrajectoryPoint>& waypoints) const
{
  // Calculate the deviation of current position
  // from the waypoint approximate line

  if (waypoints.size() < 3)
  {
    return;
  }

  geometry_msgs::Point wayp1;
  geometry_msgs::Point wayp2;
  double a, b, c;

  wayp1.x = waypoints.at(1).x;
  wayp1.y = waypoints.at(1).y;
  wayp2.x = waypoints.at(2).x;
  wayp2.y = waypoints.at(2).y;
  getLinearEquation(wayp2, wayp1, &a, &b, &c);

  std_msgs::Float32 msg;
  msg.data = getDistanceBetweenLineAndPoint(point, a, b, c);

  pub17_.publish(msg);
}

void PurePursuitNode::callbackFromCurrentPose(
  const common_msgs::localizationConstPtr& msg)
{
  pp_.setCurrentPose(msg);
  current_linear_velocity_ = msg->speed;
  pp_.setCurrentVelocity(current_linear_velocity_);
  pid_.setCurrentSpeed(current_linear_velocity_);
  pid_.setCurrentPitch(msg->pose.rotation.pitch);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const common_msgs::TrajectoryConstPtr& msg)
{
  command_linear_velocity_ =
    (!msg->points.empty()) ? msg->points.at(0).velocity : 0;
  if (add_virtual_end_waypoints_)
  {
    const LaneDirection solved_dir = getLaneDirection(*msg);
    direction_ = (solved_dir != LaneDirection::Error) ? solved_dir : direction_;
    common_msgs::Trajectory expanded_lane(*msg);
    expand_size_ = -expanded_lane.points.size();
    connectVirtualLastWaypoints(&expanded_lane, direction_);
    expand_size_ += expanded_lane.points.size();

    pp_.setCurrentWaypoints(expanded_lane.points);
  }
  else
  {
    pp_.setCurrentWaypoints(msg->points);
  }

  pid_.setTargetSpeed(command_linear_velocity_);
  is_waypoint_set_ = true;
}

void PurePursuitNode::connectVirtualLastWaypoints(
  common_msgs::Trajectory* lane, LaneDirection direction)
{
  if (lane->points.empty())
  {
    return;
  }
  static double interval = 1.0;
  geometry_msgs::Pose pn;
  pn.position.x = lane->points.back().x;
  pn.position.y = lane->points.back().y;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, lane->points.back().angle);
  quaternionTFToMsg(quaternion, pn.orientation);

  common_msgs::TrajectoryPoint virtual_last_waypoint;
  virtual_last_waypoint.angle = lane->points.back().angle;
  virtual_last_waypoint.velocity = 0.0;
  geometry_msgs::Point virtual_last_point_rlt;
  const int sgn = getSgn();
  for (double dist = minimum_lookahead_distance_; dist > 0.0; dist -= interval)
  {
    virtual_last_point_rlt.x += interval * sgn;
    virtual_last_point_rlt = calcAbsoluteCoordinate(virtual_last_point_rlt, pn);
    virtual_last_waypoint.x = virtual_last_point_rlt.x;
    virtual_last_waypoint.y = virtual_last_point_rlt.y;
    lane->points.emplace_back(virtual_last_waypoint);
  }
}

double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa, const double& driver_ratio)
{
  double wheel_angle = 0;
  double eps_steer = 0;
  wheel_angle = atan(wheel_base * kappa) * 180 / M_PI;
  eps_steer = wheel_angle * driver_ratio;

  return eps_steer;
}

}  // namespace waypoint_follower
