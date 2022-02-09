#include <string>
#include <algorithm>
#include "simulator/simulator_core.h"

WFSimulatorCore::WFSimulatorCore() : nh_(""), pnh_("~"), vehicle_sim_model_(nh_, pnh_)
{
  /* simulator parameters */
  double loop_rate;
  pnh_.param<double>("loop_rate", loop_rate, 50.0);
  pnh_.param<double>("lidar_height", lidar_height_, 1.0);
  pnh_.param<std::string>("simulation_frame_id", simulation_frame_id_, "base_link");
  pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  pnh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "velodyne");
  const double dt = 1.0 / loop_rate;

  /* set pub sub topic name */
  std::string sim_pose_name, sim_lidar_pose_name, sim_velocity_name, sim_vehicle_status_name;
  pnh_.param<std::string>("sim_pose_name", sim_pose_name, "current_pose");
  pnh_.param<std::string>("sim_lidar_pose_name", sim_lidar_pose_name, "localizer_pose");
  pnh_.param<std::string>("sim_velocity_name", sim_velocity_name, "current_velocity");
  pnh_.param<std::string>("sim_vehicle_status_name", sim_vehicle_status_name, "vehicle_status");
  pub_pose_ = nh_.advertise<common_msgs::localization>(sim_pose_name, 1);
  pub_lidar_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(sim_lidar_pose_name, 1);
  //pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(sim_velocity_name, 1);
  pub_twist_ = nh_.advertise<std_msgs::Float32>("/estimated_vel_kmph", 1);
  pub_vehicle_status_ = nh_.advertise<common_msgs::CanRecieve>(sim_vehicle_status_name, 1);
  sub_vehicle_cmd_ = nh_.subscribe("control", 1, &WFSimulatorCore::callbackVehicleCmd, this);
  timer_simulation_ = nh_.createTimer(ros::Duration(dt), &WFSimulatorCore::timerCallbackSimulation, this);
  timer_tf_ = nh_.createTimer(ros::Duration(0.1), &WFSimulatorCore::timerCallbackPublishTF, this);

  bool use_waypoints_for_z_position_source;
  pnh_.param<bool>("use_waypoints_for_z_position_source", use_waypoints_for_z_position_source, false);
  if (use_waypoints_for_z_position_source)
  {
    sub_waypoints_ = nh_.subscribe("base_waypoints", 1, &WFSimulatorCore::callbackWaypoints, this);
    sub_closest_waypoint_ = nh_.subscribe("closest_waypoint", 1, &WFSimulatorCore::callbackClosestWaypoint, this);
  }

  /* set initialize source */
  std::string initialize_source;
  pnh_.param<std::string>("initialize_source", initialize_source, "ORIGIN");
  ROS_INFO_STREAM("initialize_source : " << initialize_source);
  if (initialize_source == "RVIZ")
  {
    sub_initialpose_ = nh_.subscribe("initialpose", 1, &WFSimulatorCore::callbackInitialPoseWithCov, this);
  }
  else if (initialize_source == "NDT")
  {
    sub_initialpose_ = nh_.subscribe("ndt_pose", 1, &WFSimulatorCore::callbackInitialPoseStamped, this);
  }
  else if (initialize_source == "GNSS")
  {
    sub_initialpose_ = nh_.subscribe("gnss_pose", 1, &WFSimulatorCore::callbackInitialPoseStamped, this);
  }
  else if (initialize_source == "ORIGIN")
  {
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;  // yaw = 0
    geometry_msgs::Twist t;
    vehicle_sim_model_.setInitialState(p, t);  // initialize with 0 for all variables
  }
  else
  {
    ROS_WARN("initialize_source is undesired, setting error!!");
  }
  current_pose_.orientation.w = 1.0;

  closest_pos_z_ = 0.0;
}

void WFSimulatorCore::callbackWaypoints(const common_msgs::TrajectoryConstPtr& msg)
{
  current_waypoints_ptr_ = std::make_shared<common_msgs::Trajectory>(*msg);
}

void WFSimulatorCore::callbackClosestWaypoint(const std_msgs::Int32ConstPtr& msg)
{
  if (current_waypoints_ptr_ != nullptr)
  {
    if (-1 < msg->data && msg->data < static_cast<int>(current_waypoints_ptr_->points.size()))
    {
      closest_pos_z_ = 0;
    }
  }
}

void WFSimulatorCore::callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulatorCore::callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulatorCore::timerCallbackPublishTF(const ros::TimerEvent& e)
{
  publishTF(current_pose_);
}

void WFSimulatorCore::timerCallbackSimulation(const ros::TimerEvent& e)
{
  if (!vehicle_sim_model_.isInitialized())
  {
    ROS_INFO_DELAYED_THROTTLE(3.0, "[simulator] waiting initial position...");
    return;
  }

  if (prev_update_time_ptr_ == nullptr)
  {
    prev_update_time_ptr_ = std::make_shared<ros::Time>(ros::Time::now());
  }

  /* calculate delta time */
  const double dt = (ros::Time::now() - *prev_update_time_ptr_).toSec();
  *prev_update_time_ptr_ = ros::Time::now();

  /* update vehicle dynamics */
  vehicle_sim_model_.updateStatus(dt, closest_pos_z_);
  current_pose_ = vehicle_sim_model_.getCurrentPose();
  const geometry_msgs::Twist& current_twist = vehicle_sim_model_.getCurrentTwist();
  const double steering_angle = vehicle_sim_model_.getCurrentSteeringAngle();

  /* publish pose & twist */
  publishPoseTwist(current_pose_, current_twist);

  /* publish vehicle_statue for steering vehicle */
  common_msgs::CanRecieve vs;
  vs.header.stamp = ros::Time::now();
  vs.header.frame_id = simulation_frame_id_;
  static const double MPS2KMPH = 3600.0 / 1000.0;
  vs.vehicleInfo.speed = current_twist.linear.x * MPS2KMPH;
  vs.vehicleInfo.steerangle = amathutils::rad2deg(steering_angle);
  pub_vehicle_status_.publish(vs);
}

void WFSimulatorCore::callbackVehicleCmd(const common_msgs::SystemOverallConstPtr& msg)
{
  vehicle_sim_model_.setVehicleCmd(msg);
}

void WFSimulatorCore::setInitialStateWithPoseTransform(const geometry_msgs::PoseStamped& pose_stamped,
                                                   const geometry_msgs::Twist& twist)
{
  tf::StampedTransform transform(getTransformFromTF(map_frame_id_, pose_stamped.header.frame_id));
  geometry_msgs::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.getOrigin().x();
  pose.position.y = pose_stamped.pose.position.y + transform.getOrigin().y();
  pose.position.z = pose_stamped.pose.position.z + transform.getOrigin().z();
  pose.orientation = pose_stamped.pose.orientation;
  vehicle_sim_model_.setInitialState(pose, twist);
}

void WFSimulatorCore::setInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                                   const geometry_msgs::Twist& twist)
{
  geometry_msgs::PoseStamped ps;
  ps.header = pose.header;
  ps.pose = pose.pose.pose;
  setInitialStateWithPoseTransform(ps, twist);
}

tf::StampedTransform WFSimulatorCore::getTransformFromTF(
  const std::string parent_frame, const std::string child_frame)
{
  tf::StampedTransform transform;
  while (1)
  {
    try
    {
      tf_listener_.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  return transform;
}

// void WFSimulatorCore::publishPoseTwist(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
// {
//   ros::Time current_time = ros::Time::now();

//   // simulatied pose
//   geometry_msgs::PoseStamped ps;
//   ps.header.frame_id = map_frame_id_;
//   ps.header.stamp = current_time;
//   ps.pose = pose;
//   pub_pose_.publish(ps);

//   // lidar pose
//   ps.pose.position.z += lidar_height_;
//   pub_lidar_pose_.publish(ps);

//   geometry_msgs::TwistStamped ts;
//   ts.header.frame_id = simulation_frame_id_;
//   ts.header.stamp = current_time;
//   ts.twist = twist;
//   pub_twist_.publish(ts);
// }

void WFSimulatorCore::publishPoseTwist(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
{
  ros::Time current_time = ros::Time::now();
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // simulatied pose
  common_msgs::localization localps;
  localps.header.frame_id = map_frame_id_;
  localps.header.stamp = current_time;
  localps.pose.position.x = pose.position.x;
  localps.pose.position.y = pose.position.y;
  localps.pose.position.z = pose.position.z;
  localps.pose.rotation.roll = roll;
  localps.pose.rotation.pitch = pitch;
  localps.pose.rotation.yaw = yaw;
  localps.twist.vel_x = twist.linear.x;
  localps.twist.vel_y = twist.linear.y;
  localps.twist.vel_z = twist.linear.z;
  localps.twist.gyro_x = twist.angular.x;
  localps.twist.gyro_y = twist.angular.y;
  localps.twist.gyro_z = twist.angular.z;
  localps.speed = twist.linear.x;
  pub_pose_.publish(localps);

  // lidar pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = map_frame_id_;
  ps.header.stamp = current_time;
  ps.pose = pose;
  ps.pose.position.z += lidar_height_;
  pub_lidar_pose_.publish(ps);

  std_msgs::Float32 vel_kmph;
  vel_kmph.data = twist.linear.x * 3.6;
  pub_twist_.publish(vel_kmph);
}

void WFSimulatorCore::publishTF(const geometry_msgs::Pose& pose)
{
  ros::Time current_time = ros::Time::now();

  // send odom transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = map_frame_id_;
  odom_trans.child_frame_id = simulation_frame_id_;
  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;
  tf_broadcaster_.sendTransform(odom_trans);

  // send lidar transform
  geometry_msgs::TransformStamped lidar_trans;
  lidar_trans.header.stamp = odom_trans.header.stamp;
  lidar_trans.header.frame_id = simulation_frame_id_;
  lidar_trans.child_frame_id = lidar_frame_id_;
  lidar_trans.transform.translation.z += lidar_height_;
  lidar_trans.transform.rotation.w = 1.0;
  tf_broadcaster_.sendTransform(lidar_trans);
}