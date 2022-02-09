#include <motion_planner/velocity_set/velocity_set_path.h>

VelocitySetPath::VelocitySetPath()
  : set_path_(false),
    current_vel_(0),
    closest_waypoint_index_(-1),
    closest_local_index_(-1)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("velocity_offset", velocity_offset_, 1.2);
  private_nh_.param<double>("decelerate_vel_min", decelerate_vel_min_, 1.3);
  private_nh_.param<int>("closest_search_size", closest_search_size_, 30);
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 30);
}

VelocitySetPath::~VelocitySetPath()
{
}

// check if waypoint number is valid
bool VelocitySetPath::checkWaypoint(int num, const char *name) const
{
  if (num < 0 || num >= getPrevWaypointsSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void VelocitySetPath::setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, common_msgs::localization control_pose)
{
  if (closest_waypoint < 0)
    return;

  temporal_waypoints_.points.clear();
  temporal_waypoints_.header = new_waypoints_.header;

  // push current pose
  common_msgs::TrajectoryPoint current_point;
  current_point.x = control_pose.pose.position.x;
  current_point.y = control_pose.pose.position.y;
  current_point.angle = control_pose.pose.rotation.yaw;
  current_point.velocity = new_waypoints_.points[closest_waypoint].velocity;
  temporal_waypoints_.points.push_back(current_point);

  for (int i = 0; i < temporal_waypoints_size; i++)
  {
    if (closest_waypoint + i >= getNewWaypointsSize())
      return;

    temporal_waypoints_.points.push_back(new_waypoints_.points[closest_waypoint + i]);
  }

  return;
}

double VelocitySetPath::calcChangedVelocity(const double& current_vel, const double& accel, const std::array<int, 2>& range) const
{
  static double current_velocity = current_vel;
  static double square_vel = current_vel * current_vel;
  if (current_velocity != current_vel)
  {
    current_velocity = current_vel;
    square_vel = current_vel * current_vel;
  }
  return std::sqrt(square_vel + 2.0 * accel * calcInterval(range.at(0), range.at(1)));
}

void VelocitySetPath::changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint)
{
  int extra = 4; // for safety

  // decelerate with constant deceleration
  for (int index = obstacle_waypoint + extra; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index, __FUNCTION__))
      continue;

    // v = sqrt( (v0)^2 + 2ax )
    std::array<int, 2> range = {index, obstacle_waypoint};
    double changed_vel = calcChangedVelocity(decelerate_vel_min_, deceleration, range);

    double prev_vel = prev_waypoints_.points[index].velocity;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    new_waypoints_.points[index].velocity = sgn * std::min(std::abs(prev_vel), changed_vel);
  }

}

void VelocitySetPath::avoidSuddenAcceleration(double acceleration, int closest_waypoint)
{
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, __FUNCTION__))
      return;

    // accelerate with constant acceleration
    // v = root((v0)^2 + 2ax)
    std::array<int, 2> range = {closest_waypoint, closest_waypoint + i};
    //double changed_vel = calcChangedVelocity(current_vel_, acceleration, range) + velocity_offset_;
    double changed_vel = calcChangedVelocity(std::abs(current_vel_) + velocity_offset_, acceleration, range);
    const double& target_vel = new_waypoints_.points[closest_waypoint + i].velocity;
    // Don't exceed original velocity
    if (changed_vel > std::abs(target_vel))
      changed_vel = std::abs(target_vel);

    const int sgn = (target_vel < 0) ? -1 : 1;
    new_waypoints_.points[closest_waypoint + i].velocity = sgn * changed_vel;
  }

  return;
}

void VelocitySetPath::avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint)
{
  if (closest_waypoint < 0)
    return;

  const double& closest_vel = new_waypoints_.points[closest_waypoint].velocity;

  // if accelerating, do not modify the speed profile.
  if ((current_vel_ >= 0.0 && current_vel_ <= closest_vel) || (current_vel_ < 0.0 && current_vel_ > closest_vel))
    return;

  // not avoid braking
  if (std::abs(current_vel_ - closest_vel) < velocity_change_limit)
    return;

  //std::cout << "avoid sudden braking!" << std::endl;
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, __FUNCTION__))
      return;

    // sqrt(v^2 - 2ax)
    std::array<int, 2> range = {closest_waypoint, closest_waypoint + i};
    double changed_vel = calcChangedVelocity(std::abs(current_vel_) - velocity_change_limit, -deceleration, range);
    const double& target_vel = new_waypoints_.points[closest_waypoint + i].velocity;

    if (std::isnan(changed_vel))
    {
      break;
    }
    const int sgn = (target_vel < 0) ? -1 : 1;
    new_waypoints_.points[closest_waypoint + i].velocity = sgn * changed_vel;
  }

}

void VelocitySetPath::changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint, double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = stop_waypoint; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index, __FUNCTION__))
      continue;

    // v = (v0)^2 + 2ax, and v0 = 0
    std::array<int, 2> range = {index, stop_waypoint};
    double changed_vel = calcChangedVelocity(0.0, deceleration, range);

    double prev_vel = prev_waypoints_.points[index].velocity;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    new_waypoints_.points[index].velocity = sgn * std::min(std::abs(prev_vel), changed_vel);
  }

  // fill velocity with 0 for stopping
  for (int i = stop_waypoint; i <= obstacle_waypoint; i++)
  {
    new_waypoints_.points[i].velocity = 0;
  }

}

void VelocitySetPath::initializeNewWaypoints()
{
  new_waypoints_ = prev_waypoints_;
}

double VelocitySetPath::calcInterval(const int begin, const int end) const
{
  // check index
  if (begin < 0 || begin >= getPrevWaypointsSize() || end < 0 || end >= getPrevWaypointsSize())
  {
    ROS_WARN("Invalid index");
    return 0;
  }

  // Calculate the inteval of waypoints
  double dist_sum = 0;
  for (int i = begin; i < end; i++)
  {
    tf::Vector3 v1(prev_waypoints_.points[i].x,
                   prev_waypoints_.points[i].y, 0);

    tf::Vector3 v2(prev_waypoints_.points[i + 1].x,
                   prev_waypoints_.points[i + 1].y, 0);

    dist_sum += tf::tfDistance(v1, v2);
  }

  return dist_sum;
}

void VelocitySetPath::resetFlag()
{
  set_path_ = false;
}

void VelocitySetPath::waypointsCallback(const common_msgs::TrajectoryConstPtr& msg)
{
  static common_msgs::Trajectory prev_base_waypoints;
  base_waypoints_ = *msg;

  if (set_path_)
  {
    // detect waypoint change by timestamp update
    waypoint_t2 = ros::WallTime::now();
    if ((waypoint_t2 - waypoint_t1).toSec() > 1e-1)
    {
      closest_local_index_ = -1;    
      prev_base_waypoints = base_waypoints_;
      waypoint_t1 = ros::WallTime::now();
    }
  }
  else
  {
    prev_base_waypoints = base_waypoints_;
  }

  new_waypoints_ = localWaypoints(base_waypoints_);
  prev_waypoints_ = new_waypoints_;
  set_path_ = true;
}

void VelocitySetPath::closestWaypointCallback(const std_msgs::Int32& msg)
{
  closest_waypoint_index_ = msg.data;

  if (closest_waypoint_index_ == -1)
  {
    closest_local_index_ = -1; // reset local closest waypoint
  }

  closest_waypoint_initialized_ = true;
}

void VelocitySetPath::currentPoseCallback(const common_msgs::localizationConstPtr &msg)
{
  pose_global = *msg;
  current_vel_ = msg->speed;
}

common_msgs::Trajectory VelocitySetPath::localWaypoints(const common_msgs::Trajectory& current_waypoints)
{
  common_msgs::Trajectory safety_waypoints;
  safety_waypoints.header = current_waypoints.header;
  safety_waypoints.points.clear();

  geometry_msgs::Pose current_pose;
  current_pose.position.x = pose_global.pose.position.x;
  current_pose.position.y = pose_global.pose.position.y;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, pose_global.pose.rotation.yaw);
  quaternionTFToMsg(quaternion, current_pose.orientation);

  int closest_waypoint = getLocalClosestWaypoint(current_waypoints, current_pose, closest_search_size_);
  // push waypoints from closest index
  for (int i = 0; i < safety_waypoints_size_; ++i)
  {
    int index = closest_waypoint + i;
    if (closest_waypoint < 0 || static_cast<int>(current_waypoints.points.size()) <= index)
    {
      break;
    }
    const common_msgs::TrajectoryPoint& wp = current_waypoints.points[index];
    safety_waypoints.points.push_back(wp);
  }
  return safety_waypoints;
}

int VelocitySetPath::getLocalClosestWaypoint(const common_msgs::Trajectory& waypoints, const geometry_msgs::Pose& pose, const int& search_size)
{
  static common_msgs::Trajectory local_waypoints;  // around self-vehicle
  const int prev_index = closest_local_index_;

  // search in all waypoints if lane_select judges you're not on waypoints
  if (closest_local_index_ == -1)
  {
    closest_local_index_ = getClosestWaypoint(waypoints, pose);
  }
  // search in limited area based on prev_index
  else
  {
    // get neighborhood waypoints around prev_index
    int start_index = std::max(0, prev_index - search_size / 2);
    int end_index = std::min(prev_index + search_size / 2, (int)waypoints.points.size());
    auto start_itr = waypoints.points.begin() + start_index;
    auto end_itr = waypoints.points.begin() + end_index;
    local_waypoints.points = std::vector<common_msgs::TrajectoryPoint>(start_itr, end_itr);

    // get closest waypoint in neighborhood waypoints
    closest_local_index_ = start_index + getClosestWaypoint(local_waypoints, pose);
  }

  if (closest_local_index_ != -1)
    closest_local_index_ += 1;
    
  return closest_local_index_;
}
