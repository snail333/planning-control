#include "rtk_local.h"


rtk_localization::rtk_localization()
  : private_nh("~")
{
  private_nh.param<double>("zone_lat", zone_lat_, 39.9);
  private_nh.param<double>("zone_lon", zone_lon_, 116.4);

  // Publishers
  localization_pub = nh.advertise<common_msgs::localization>("localization", 1);
  estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 1);

  // Subscribers
  imu_sub = nh.subscribe("sensorimu", 10, &rtk_localization::imu_callback, this);
  odom_sub = nh.subscribe("sensor_odom", 10, &rtk_localization::odom_callback, this);
  gps_sub = nh.subscribe("sensorgps", 10, &rtk_localization::gps_callback, this);
}

rtk_localization::~rtk_localization(){}

void rtk_localization::run()
{
  ros::spin();
}

void rtk_localization::gps_callback(const common_msgs::sensorgps::ConstPtr& msg)
{
  gps_utm geo;

  geo.set_offset(true);
  geo.set_zone(zone_lat_, zone_lon_);
  geo.llh_to_xyz_geolib(msg->lat, msg->lon, msg->height);

  geometry_msgs::Quaternion _quat;
  geometry_msgs::PoseStamped pose;
  //pose.header = msg->header;
  pose.header.stamp = ros::Time::now();

  pose.header.frame_id = "map";
  pose.pose.position.x = geo.x();
  pose.pose.position.y = geo.y();
  pose.pose.position.z = 0;

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  //std::cout << "distance : " << distance << std::endl;

  if (distance > 0.2)
  {
    gnss_yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(gnss_yaw);
    _prev_pose = pose;
  }

  pose.pose.orientation = _quat;
  current_gnss_pose.x = pose.pose.position.x;
  current_gnss_pose.y = pose.pose.position.y;
  current_gnss_pose.z = pose.pose.position.z;
  current_gnss_pose.lat = msg->lat;
  current_gnss_pose.lon = msg->lon;
  current_gnss_pose.height = msg->height;
  current_gnss_pose.vel_x = msg->velocity;
  current_gnss_pose.gps_status = msg->status;
  current_gnss_pose.sate_num = msg->satenum;

  //localizer_publish();

  //座標変換
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, gnss_yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "gps"));
}

const double rtk_localization::wrapToPmPi(double a_angle_rad)
{
  if ( a_angle_rad > M_PI)
    a_angle_rad -= 2.0 * M_PI;
  else if ( a_angle_rad < -M_PI)
    a_angle_rad += 2.0 * M_PI; 

  return a_angle_rad;
}

void rtk_localization::odom_callback(const common_msgs::wheelspeed::ConstPtr& input)
{
  // double lr_speed, rr_speed;
  // lr_speed = input-> wheelspeed_lr_pluse;
  // rr_speed = input-> wheelspeed_rr_pluse;

  // localizer_publish();
}

void rtk_localization::imu_callback(const common_msgs::sensorimu::ConstPtr& input)
{
  double Heading, Roll, Pitch;
  Heading = M_PI * input-> heading / 180.0;
  Pitch = M_PI * input-> pitch / 180.0;
  Roll = M_PI * input-> roll / 180.0;
  
  current_pose_imu.roll = Roll;
  current_pose_imu.pitch = Pitch;
  current_pose_imu.yaw = M_PI_2 - Heading;
  current_pose_imu.yaw = wrapToPmPi(current_pose_imu.yaw);
  current_pose_imu.acce_x = input-> acce_x;
  current_pose_imu.acce_y = input-> acce_y;
  current_pose_imu.acce_z = input-> acce_z;
  current_pose_imu.gyro_x = input-> gyro_x;
  current_pose_imu.gyro_y = input-> gyro_y;
  current_pose_imu.gyro_z = input-> gyro_z;

  localizer_publish();
}

void rtk_localization::localizer_publish()
{

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion current_q, localizer_q;

    current_scan_time = ros::Time::now();
    current_pose.x = current_gnss_pose.x;
    current_pose.y = current_gnss_pose.y;
    current_pose.z = current_gnss_pose.z;
    current_pose.lat = current_gnss_pose.lat;
    current_pose.lon = current_gnss_pose.lon;
    current_pose.height = current_gnss_pose.height;
    current_pose.gps_status = current_gnss_pose.gps_status;
    current_pose.sate_num = current_gnss_pose.sate_num;
    current_pose.roll = current_pose_imu.roll;
    current_pose.pitch = current_pose_imu.pitch;
    current_pose.yaw = current_pose_imu.yaw;
    current_pose.acce_x = current_pose_imu.acce_x;
    current_pose.acce_y = current_pose_imu.acce_y;
    current_pose.acce_z = current_pose_imu.acce_z;
    current_pose.vel_x = current_gnss_pose.vel_x;
    current_pose.vel_y = 0;
    current_pose.vel_z = 0;
    current_pose.gyro_x = current_pose_imu.gyro_x;
    current_pose.gyro_y = current_pose_imu.gyro_y;
    current_pose.gyro_z = current_pose_imu.gyro_z;

    estimated_vel_kmph.data = current_pose.vel_x * 3.6;
    estimated_vel_kmph_pub.publish(estimated_vel_kmph);

    current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.header.frame_id = "/map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.llh.lat = current_pose.lat;
    current_pose_msg.llh.lon = current_pose.lon;
    current_pose_msg.llh.height = current_pose.height;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.rotation.roll = current_pose.roll;
    current_pose_msg.pose.rotation.pitch = current_pose.pitch;
    current_pose_msg.pose.rotation.yaw = current_pose.yaw;
    current_pose_msg.twist.vel_x = current_pose.vel_x;
    current_pose_msg.twist.vel_y = current_pose.vel_y;
    current_pose_msg.twist.vel_z = current_pose.vel_z;
    current_pose_msg.twist.gyro_x = current_pose.gyro_x;
    current_pose_msg.twist.gyro_y = current_pose.gyro_y;
    current_pose_msg.twist.gyro_z = current_pose.gyro_z;
    current_pose_msg.acceleration.acce_x = current_pose.acce_x;
    current_pose_msg.acceleration.acce_y = current_pose.acce_y;
    current_pose_msg.acceleration.acce_z = current_pose.acce_z;
    current_pose_msg.speed = current_pose.vel_x;
    current_pose_msg.nav_status.gps_status = current_pose.gps_status;
    current_pose_msg.nav_status.sate_num = current_pose.sate_num;
    localization_pub.publish(current_pose_msg);

    // Send TF "/base_link" to "/map"
    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    transform.setRotation(current_q);
    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtk_localization");
  rtk_localization rtk;
  rtk.run();

  return 0;
}
