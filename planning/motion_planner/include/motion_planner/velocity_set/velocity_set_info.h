#ifndef VELOCITY_SET_INFO_H
#define VELOCITY_SET_INFO_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <memory>
#include <common_msgs/localization.h>

class VelocitySetInfo
{
 private:
  // parameters
  double stop_range_;               // if obstacle is in this range, stop
  double deceleration_range_;       // if obstacle is in this range, decelerate
  int points_threshold_;            // points threshold to find obstacles
  double detection_height_top_;     // from sensor
  double detection_height_bottom_;  // from sensor
  double stop_distance_obstacle_;   // (meter) stopping distance from obstacles
  double stop_distance_stopline_;   // (meter) stopping distance from stoplines
  double deceleration_obstacle_;    // (m/s^2) deceleration for obstacles
  double deceleration_stopline_;    // (m/s^2) deceleration for stopline
  double acceleration_;
  double velocity_change_limit_;    // (m/s)
  double temporal_waypoints_size_;  // (meter)

  // ROS param
  double remove_points_upto_;

  pcl::PointCloud<pcl::PointXYZ> points_;
  geometry_msgs::PoseStamped localizer_pose_;  // pose of sensor
  common_msgs::localization control_pose_;    // pose of base_link
  bool set_pose_;


 public:
  VelocitySetInfo();
  ~VelocitySetInfo();

  // ROS Callback
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void controlPoseCallback(const common_msgs::localizationConstPtr &msg);
  void localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

  void clearPoints();

  double getStopRange() const
  {
    return stop_range_;
  }

  double getDecelerationRange() const
  {
    return deceleration_range_;
  }

  int getPointsThreshold() const
  {
    return points_threshold_;
  }

  int getDetectionHeightTop() const
  {
    return detection_height_top_;
  }

  int getDetectionHeightBottom() const
  {
    return detection_height_bottom_;
  }

  int getStopDistanceObstacle() const
  {
    return stop_distance_obstacle_;
  }

  int getStopDistanceStopline() const
  {
    return stop_distance_stopline_;
  }

  double getDecelerationObstacle() const
  {
    return deceleration_obstacle_;
  }

  double getAcceleration() const
  {
    return acceleration_;
  }

  double getDecelerationStopline() const
  {
    return deceleration_stopline_;
  }

  double getVelocityChangeLimit() const
  {
    return velocity_change_limit_;
  }

  double getTemporalWaypointsSize() const
  {
    return temporal_waypoints_size_;
  }

  pcl::PointCloud<pcl::PointXYZ> getPoints() const
  {
    return points_;
  }

  common_msgs::localization getControlPose() const
  {
    return control_pose_;
  }

  geometry_msgs::PoseStamped getLocalizerPose() const
  {
    return localizer_pose_;
  }

  bool getSetPose() const
  {
    return set_pose_;
  }
};

#endif // VELOCITY_SET_INFO_H
