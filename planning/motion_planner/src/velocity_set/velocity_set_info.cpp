#include <motion_planner/velocity_set/velocity_set_info.h>

void joinPoints(const pcl::PointCloud<pcl::PointXYZ>& points1, pcl::PointCloud<pcl::PointXYZ>* points2)
{
  for (const auto& p : points1)
  {
    points2->push_back(p);
  }
}

VelocitySetInfo::VelocitySetInfo()
  : stop_range_(1.3),
    deceleration_range_(0),
    points_threshold_(10),
    detection_height_top_(0.2),
    detection_height_bottom_(-1.7),
    stop_distance_obstacle_(10),
    stop_distance_stopline_(5),
    deceleration_obstacle_(0.8),
    deceleration_stopline_(0.6),
    acceleration_(0.5),
    velocity_change_limit_(2.77),
    temporal_waypoints_size_(100),
    set_pose_(false)
{
  ros::NodeHandle private_nh_("~");
  ros::NodeHandle nh;

  double vel_change_limit_kph = 9.972;
  private_nh_.param<double>("remove_points_upto", remove_points_upto_, 2.3);
  private_nh_.param<double>("stop_distance_obstacle", stop_distance_obstacle_, 10.0);
  private_nh_.param<double>("stop_distance_stopline", stop_distance_stopline_, 5.0);
  private_nh_.param<double>("detection_range", stop_range_, 1.3);
  private_nh_.param<int>("points_threshold", points_threshold_, 10);
  private_nh_.param<double>("detection_height_top", detection_height_top_, 0.2);
  private_nh_.param<double>("detection_height_bottom", detection_height_bottom_, -1.7);
  private_nh_.param<double>("deceleration_obstacle", deceleration_obstacle_, 0.8);
  private_nh_.param<double>("deceleration_stopline", deceleration_stopline_, 0.6);
  private_nh_.param<double>("acceleration", acceleration_, 0.5);
  private_nh_.param<double>("velocity_change_limit", vel_change_limit_kph, 9.972);
  private_nh_.param<double>("deceleration_range", deceleration_range_, 0);
  private_nh_.param<double>("temporal_waypoints_size", temporal_waypoints_size_, 100.0);

  velocity_change_limit_ = vel_change_limit_kph / 3.6;  // kph -> mps

}

VelocitySetInfo::~VelocitySetInfo()
{
}

void VelocitySetInfo::clearPoints()
{
  points_.clear();
}

void VelocitySetInfo::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  points_.clear();
  for (const auto &v : sub_points)
  {
    if (v.x == 0 && v.y == 0)
      continue;

    if (v.z > detection_height_top_ || v.z < detection_height_bottom_)
      continue;

    // ignore points nearby the vehicle
    if (v.x * v.x + v.y * v.y < remove_points_upto_ * remove_points_upto_)
      continue;

    points_.push_back(v);
  }

}

void VelocitySetInfo::controlPoseCallback(const common_msgs::localizationConstPtr &msg)
{
  control_pose_ = *msg;
  if (!set_pose_)
    set_pose_ = true;
}

void VelocitySetInfo::localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  //health_checker_ptr_->NODE_ACTIVATE();
  //health_checker_ptr_->CHECK_RATE("topic_rate_localizer_pose_slow", 8, 5, 1, "topic localizer_pose subscribe rate slow.");
  localizer_pose_ = *msg;
}
