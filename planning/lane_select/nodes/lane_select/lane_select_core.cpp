#include "lane_select_core.h"

#include <algorithm>

namespace lane_planner
{
// Constructor
LaneSelectNode::LaneSelectNode()
  : private_nh_("~")
  , lane_array_id_(-1)
  , current_lane_idx_(-1)
  , right_lane_idx_(-1)
  , left_lane_idx_(-1)
  , is_lane_array_subscribed_(false)
  , is_current_pose_subscribed_(false)
  , is_config_subscribed_(false)
  , distance_threshold_(30.0)
  , distance_threshold_neighbor_lanes(5.0)
  , lane_change_interval_(10.0)
  , lane_change_target_ratio_(2.0)
  , lane_change_target_minimum_(5.0)
  , vlength_hermite_curve_(10)
  , search_closest_waypoint_minimum_dt_(5)
  , current_state_("UNKNOWN")
{
  initForROS();
}

// Destructor
LaneSelectNode::~LaneSelectNode()
{
}

void LaneSelectNode::initForROS()
{
  // setup subscriber
  sub1_ = nh_.subscribe("/lane_waypoints_array", 10, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("localization", 10, &LaneSelectNode::callbackFromPoseStamped, this);
  //sub3_ = nh_.subscribe("current_velocity", 10, &LaneSelectNode::callbackFromTwistStamped, this);

  // setup publisher

  pub1_ = nh_.advertise<common_msgs::Trajectory>("base_waypoints", 1);
  pub2_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 1);

  // get from rosparam
  private_nh_.param<double>("lane_change_interval", lane_change_interval_, double(2));
  private_nh_.param<double>("distance_threshold", distance_threshold_, double(30.0));
  private_nh_.param<double>("distance_neighbor_lanes", distance_threshold_neighbor_lanes, double(5.0));
  private_nh_.param<int>("search_closest_waypoint_minimum_dt", search_closest_waypoint_minimum_dt_, int(5));
  private_nh_.param<double>("lane_change_target_ratio", lane_change_target_ratio_, double(2.0));
  private_nh_.param<double>("lane_change_target_minimum", lane_change_target_minimum_, double(5.0));
  private_nh_.param<double>("vector_length_hermite_curve", vlength_hermite_curve_, double(10.0));
}

bool LaneSelectNode::isAllTopicsSubscribed()
{
  if (!is_current_pose_subscribed_ || !is_lane_array_subscribed_)
  {
    // ROS_WARN("Necessary topics are not subscribed yet. Waiting...");
    return false;
  }
  return true;
}

void LaneSelectNode::initForLaneSelect()
{
  if (!isAllTopicsSubscribed())
    return;

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    publishClosestWaypoint(-1);
    resetLaneIdx();
    return;
  }

  findCurrentLane();
  publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
  publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));

  resetSubscriptionFlag();
  return;
}

void LaneSelectNode::resetLaneIdx()
{
  current_lane_idx_ = -1;
  right_lane_idx_ = -1;
  left_lane_idx_ = -1;
}

void LaneSelectNode::resetSubscriptionFlag()
{
  is_current_pose_subscribed_ = false;
}

void LaneSelectNode::processing()
{
  if (!isAllTopicsSubscribed())
    return;

  // search closest waypoint number for each lanes
  if (!getClosestWaypointNumberForEachLanes())
  {
    publishClosestWaypoint(-1);
    resetLaneIdx();
    return;
  }

  // if closest waypoint on current lane is -1,
  if (std::get<1>(tuple_vec_.at(current_lane_idx_)) == -1)
  {
    publishClosestWaypoint(-1);
    resetLaneIdx();
    return;
  }

  publishLane(std::get<0>(tuple_vec_.at(current_lane_idx_)));
  publishClosestWaypoint(std::get<1>(tuple_vec_.at(current_lane_idx_)));
  resetSubscriptionFlag();
}

bool LaneSelectNode::getClosestWaypointNumberForEachLanes()
{
  for (auto &el : tuple_vec_)
  {
    std::get<1>(el) = getClosestWaypointNumber(std::get<0>(el), current_pose_.pose, current_velocity_,
                                               std::get<1>(el), distance_threshold_, search_closest_waypoint_minimum_dt_);
    //ROS_INFO("closest: %d", std::get<1>(el));
  }

  // confirm if all closest waypoint numbers are -1. If so, output warning
  int32_t accum = 0;
  for (const auto &el : tuple_vec_)
  {
    accum += std::get<1>(el);
  }
  if (accum == (-1) * static_cast<int32_t>(tuple_vec_.size()))
  {
    //ROS_WARN("Cannot get closest waypoints. All closest waypoints are changed to -1...");
    return false;
  }

  return true;
}

void LaneSelectNode::findCurrentLane()
{
  std::vector<uint32_t> idx_vec;
  idx_vec.reserve(tuple_vec_.size());
  for (uint32_t i = 0; i < tuple_vec_.size(); i++)
  {
    if (std::get<1>(tuple_vec_.at(i)) == -1)
      continue;
    idx_vec.push_back(i);
  }
  current_lane_idx_ = findMostClosestLane(idx_vec, current_pose_.pose.position);
}

int32_t LaneSelectNode::findMostClosestLane(const std::vector<uint32_t> idx_vec, const common_msgs::Position p)
{
  std::vector<double> dist_vec;
  dist_vec.reserve(idx_vec.size());
  for (const auto &el : idx_vec)
  {
    int32_t closest_number = std::get<1>(tuple_vec_.at(el));
    dist_vec.push_back(
        getTwoDimensionalDistance(p, std::get<0>(tuple_vec_.at(el)).points.at(closest_number)));
  }
  std::vector<double>::iterator itr = std::min_element(dist_vec.begin(), dist_vec.end());
  return idx_vec.at(std::distance(dist_vec.begin(), itr));
}

void LaneSelectNode::publishLane(const common_msgs::Trajectory &lane)
{
  // publish global lane
  pub1_.publish(lane);
}

void LaneSelectNode::publishClosestWaypoint(const int32_t clst_wp)
{
  // publish closest waypoint
  std_msgs::Int32 closest_waypoint;
  closest_waypoint.data = clst_wp;
  pub2_.publish(closest_waypoint);
}

void LaneSelectNode::callbackFromLaneArray(const common_msgs::TrajectoryArrayConstPtr &msg)
{
  tuple_vec_.clear();
  tuple_vec_.shrink_to_fit();
  tuple_vec_.reserve(msg->trajectories.size());
  for (const auto &el : msg->trajectories)
  {
    auto t = std::make_tuple(el, -1, ChangeFlag::unknown);
    tuple_vec_.push_back(t);
  }

  lane_array_id_ = msg->id;
  current_lane_idx_ = -1;
  right_lane_idx_ = -1;
  left_lane_idx_ = -1;
  is_lane_array_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}

void LaneSelectNode::callbackFromPoseStamped(const common_msgs::localizationConstPtr &msg)
{
  current_pose_ = *msg;
  current_velocity_ = current_pose_.speed;
  is_current_pose_subscribed_ = true;

  if (current_lane_idx_ == -1)
    initForLaneSelect();
  else
    processing();
}

// void LaneSelectNode::callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr &msg)
// {
//   current_velocity_ = *msg;
//   is_current_velocity_subscribed_ = true;

//   if (current_lane_idx_ == -1)
//     initForLaneSelect();
//   else
//     processing();
// }

void LaneSelectNode::run()
{
  ros::spin();
}

// distance between target 1 and target2 in 2-D
double getTwoDimensionalDistance(const common_msgs::Position &target1, const common_msgs::TrajectoryPoint &target2)
{
  double distance = sqrt(pow(target1.x - target2.x, 2) + pow(target1.y - target2.y, 2));
  return distance;
}

geometry_msgs::Point convertPointIntoRelativeCoordinate(const geometry_msgs::Point &input_point,
                                                        const geometry_msgs::Pose &pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

geometry_msgs::Point convertPointIntoWorldCoordinate(const geometry_msgs::Point &input_point,
                                                     const geometry_msgs::Pose &pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = inverse * p;

  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

double getRelativeAngle(const geometry_msgs::Pose &waypoint_pose, const geometry_msgs::Pose &current_pose)
{
  tf::Vector3 x_axis(1, 0, 0);
  tf::Transform waypoint_tfpose;
  tf::poseMsgToTF(waypoint_pose, waypoint_tfpose);
  tf::Vector3 waypoint_v = waypoint_tfpose.getBasis() * x_axis;
  tf::Transform current_tfpose;
  tf::poseMsgToTF(current_pose, current_tfpose);
  tf::Vector3 current_v = current_tfpose.getBasis() * x_axis;

  return current_v.angle(waypoint_v) * 180 / M_PI;
}

//get closest waypoint from current pose
int32_t getClosestWaypointNumber(const common_msgs::Trajectory &current_lane, const common_msgs::Pose &current_pose,
                                 const double &current_velocity, const int32_t previous_number,
                                 const double distance_threshold, const int search_closest_waypoint_minimum_dt)
{
  if (current_lane.points.size() < 2)
    return -1;

  std::vector<uint32_t> idx_vec;
  // if previous number is -1, search closest waypoint from waypoints in front of current pose
  uint32_t range_min = 0;
  uint32_t range_max = current_lane.points.size();
  if (previous_number == -1)
  {
    idx_vec.reserve(current_lane.points.size());
  }
  else
  {
    if ( (0.2 * distance_threshold) <
        getTwoDimensionalDistance(current_pose.position, current_lane.points.at(previous_number)))
    {
      //ROS_WARN("Current_pose is far away from previous closest waypoint. Initilized...");
      return -1;
    }
    range_min = static_cast<uint32_t>(previous_number);
    double ratio = 3;
    double dt = std::max(current_velocity * ratio, static_cast<double>(search_closest_waypoint_minimum_dt));
    if (static_cast<uint32_t>(previous_number + dt) < current_lane.points.size())
    {
      range_max = static_cast<uint32_t>(previous_number + dt);
    }
  }
  const LaneDirection dir = getLaneDirection(current_lane);
  const int sgn = (dir == LaneDirection::Forward) ? 1 : (dir == LaneDirection::Backward) ? -1 : 0;

  geometry_msgs::Pose veh_pose;
  veh_pose.position.x = current_pose.position.x;
  veh_pose.position.y = current_pose.position.y;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, current_pose.rotation.yaw);
  quaternionTFToMsg(quaternion, veh_pose.orientation);

  for (uint32_t i = range_min; i < range_max; i++)
  {
    geometry_msgs::Pose wps_pose;
    wps_pose.position.x = current_lane.points.at(i).x;
    wps_pose.position.y = current_lane.points.at(i).y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, current_lane.points.at(i).angle);
    quaternionTFToMsg(quaternion, wps_pose.orientation);

    geometry_msgs::Point converted_p =
      convertPointIntoRelativeCoordinate(wps_pose.position, veh_pose);
    double angle = getRelativeAngle(wps_pose, veh_pose);
    if (converted_p.x * sgn > 0 && angle < 90)
    {
      idx_vec.push_back(i);
    }
  }

  if (idx_vec.empty())
    return -1;

  std::vector<double> dist_vec;
  dist_vec.reserve(idx_vec.size());
  for (const auto &el : idx_vec)
  {
    double dt = getTwoDimensionalDistance(current_pose.position, current_lane.points.at(el));
    dist_vec.push_back(dt);
  }
  std::vector<double>::iterator itr = std::min_element(dist_vec.begin(), dist_vec.end());
  int32_t found_number = idx_vec.at(static_cast<uint32_t>(std::distance(dist_vec.begin(), itr)));
  //check closest waypoint
  if (found_number > -1 && (uint32_t)found_number < current_lane.points.size())
  {
    if (distance_threshold <
        getTwoDimensionalDistance(current_pose.position, current_lane.points.at(found_number)))
    {
        //ROS_WARN("Current_pose is far away from closest waypoint. Initilized...");
        return -1;
    }
    return found_number;
  }
  else
  {
    return -1;
  }
  
}

// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c)
{
  //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = fabs(start.x - end.x);
  double sub_y = fabs(start.y - end.y);
  double error = pow(10, -5);  // 0.00001

  if (sub_x < error && sub_y < error)
  {
    //ROS_INFO("two points are the same point!!");
    return false;
  }

  *a = end.y - start.y;
  *b = (-1) * (end.x - start.x);
  *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
{
  double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

  return d;
}

}  // lane_planner
