#ifndef _RTK_LOCAL_H
#define _RTK_LOCAL_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <llh2utm/llh2utm.hpp>

#include "common_msgs/sensorgps.h"
#include "common_msgs/sensorimu.h"
#include "common_msgs/wheelspeed.h"
#include "common_msgs/lidarmatching.h"
#include "common_msgs/localization.h"
using namespace std;


typedef struct POSE
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double lat;
  double lon;
  double height;
  double acce_x;
  double acce_y;
  double acce_z;
  double vel_x;
  double vel_y;
  double vel_z;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  uint8_t gps_status;
  uint8_t sate_num;
}POSE;

class rtk_localization
{
  public:
  rtk_localization();
  ~rtk_localization();
  void run();

  private:
  const double wrapToPmPi(double a_angle_rad);
  void gps_callback(const common_msgs::sensorgps::ConstPtr& msg);
  void imu_callback(const common_msgs::sensorimu::ConstPtr& input);
  void odom_callback(const common_msgs::wheelspeed::ConstPtr& input);
  void localizer_publish();

  POSE current_pose, current_pose_imu, current_gnss_pose;
  ros::NodeHandle nh, private_nh;
  ros::Publisher localization_pub, estimated_vel_kmph_pub;
  ros::Subscriber imu_sub, odom_sub, gps_sub;
  common_msgs::localization current_pose_msg;
  geometry_msgs::PoseStamped _prev_pose;
  ros::Time current_scan_time;
  std_msgs::Float32 estimated_vel_kmph;
  double gnss_yaw;
  double zone_lon_, zone_lat_;

};

#endif