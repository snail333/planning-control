#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include "common_msgs/CanRecieve.h"
#include "common_msgs/localization.h"
#include "libtrajectory_follower/libtrajectory_follower.h"

static const int SYNC_FRAMES = 50;

typedef message_filters::sync_policies::ApproximateTime<common_msgs::CanRecieve, common_msgs::localization>
    TwistPoseSync;

class WaypointSaver
{
public:
  WaypointSaver();
  ~WaypointSaver();

private:
  // functions

  void TwistPoseCallback(const common_msgs::CanRecieveConstPtr &twist_msg,
                         const common_msgs::localizationConstPtr &local_msg) const;
  void poseCallback(const common_msgs::localizationConstPtr &local_msg) const;
  void displayMarker(geometry_msgs::Pose pose, double velocity) const;
  void outputProcessing(geometry_msgs::Pose current_pose, double velocity) const;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher trajectory_saver_pub;

  // subscriber
  message_filters::Subscriber<common_msgs::CanRecieve> *twist_sub_;
  message_filters::Subscriber<common_msgs::localization> *pose_sub_;
  message_filters::Synchronizer<TwistPoseSync> *sync_tp_;

  // variables
  bool save_velocity_;
  double interval_;
  std::string filename_, pose_topic_, velocity_topic_, localization_topic_, vehicle_topic_;
};

WaypointSaver::WaypointSaver() : private_nh_("~")
{
  // parameter settings
  private_nh_.param<std::string>("save_filename", filename_, std::string("data.txt"));
  private_nh_.param<std::string>("localization_topic", localization_topic_, std::string("localization"));
  private_nh_.param<std::string>("vehicle_topic", vehicle_topic_, std::string("pcican"));
  private_nh_.param<double>("interval", interval_, 1.0);
  private_nh_.param<bool>("save_velocity", save_velocity_, false);

  // subscriber
  pose_sub_ = new message_filters::Subscriber<common_msgs::localization>(nh_, localization_topic_, 50);

  // if (save_velocity_)
  // {
  //   twist_sub_ = new message_filters::Subscriber<common_msgs::CanRecieve>(nh_, vehicle_topic_, 50);
  //   sync_tp_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *twist_sub_, *pose_sub_);
  //   sync_tp_->registerCallback(boost::bind(&WaypointSaver::TwistPoseCallback, this, _1, _2));
  // }
  // else
  // {
  //   pose_sub_->registerCallback(boost::bind(&WaypointSaver::poseCallback, this, _1));
  // }

  pose_sub_->registerCallback(boost::bind(&WaypointSaver::poseCallback, this, _1));

  trajectory_saver_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_saver_marker", 10, true);
}

WaypointSaver::~WaypointSaver()
{
  delete twist_sub_;
  delete pose_sub_;
  delete sync_tp_;
}

void WaypointSaver::poseCallback(const common_msgs::localizationConstPtr &local_msg) const
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = local_msg->pose.position.x;
  pose_msg.position.y = local_msg->pose.position.y;
  pose_msg.position.z = local_msg->pose.position.z;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(local_msg->pose.rotation.roll, 
                                local_msg->pose.rotation.pitch, local_msg->pose.rotation.yaw);
  quaternionTFToMsg(quaternion, pose_msg.orientation);

  if(save_velocity_)
  {
    outputProcessing(pose_msg, mps2kmph(local_msg->speed));
  }
  else
  {
    outputProcessing(pose_msg, 0);
  }

}

void WaypointSaver::TwistPoseCallback(const common_msgs::CanRecieveConstPtr &twist_msg,
                                      const common_msgs::localizationConstPtr &local_msg) const
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = local_msg->pose.position.x;
  pose_msg.position.y = local_msg->pose.position.y;
  pose_msg.position.z = local_msg->pose.position.z;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(local_msg->pose.rotation.roll, 
                                local_msg->pose.rotation.pitch, local_msg->pose.rotation.yaw);
  quaternionTFToMsg(quaternion, pose_msg.orientation);

  outputProcessing(pose_msg, mps2kmph(local_msg->speed));
  //outputProcessing(pose_msg, mps2kmph(twist_msg->twist.linear.x));
}

void WaypointSaver::outputProcessing(geometry_msgs::Pose current_pose, double velocity) const
{
  std::ofstream ofs(filename_.c_str(), std::ios::app);
  static geometry_msgs::Pose previous_pose;
  static bool receive_once = false;
  // first subscribe
  if (!receive_once)
  {
    ofs << "x,y,z,yaw,velocity,change_flag" << std::endl;
    ofs << std::fixed << std::setprecision(4) << current_pose.position.x << "," << current_pose.position.y << ","
        << current_pose.position.z << "," << tf::getYaw(current_pose.orientation) << ",0,0" << std::endl;
    receive_once = true;
    displayMarker(current_pose, 0);
    previous_pose = current_pose;
  }
  else
  {
    double distance = sqrt(pow((current_pose.position.x - previous_pose.position.x), 2) +
                           pow((current_pose.position.y - previous_pose.position.y), 2));

    // if car moves [interval] meter
    if (distance > interval_)
    {
      ofs << std::fixed << std::setprecision(4) << current_pose.position.x << "," << current_pose.position.y << ","
          << current_pose.position.z << "," << tf::getYaw(current_pose.orientation) << "," << velocity << ",0" << std::endl;
          
      displayMarker(current_pose, velocity);
      previous_pose = current_pose;
    }
  }
}

void WaypointSaver::displayMarker(geometry_msgs::Pose pose, double velocity) const
{
  static visualization_msgs::MarkerArray marray;
  static int id = 0;

  // initialize marker
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.frame_locked = true;

  // create saved trajectory marker
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.ns = "saved_trajectory_arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marray.markers.push_back(marker);

  // create saved trajectory velocity text
  marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.ns = "saved_trajectory_velocity";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << velocity << " km/h";
  marker.text = oss.str();
  marray.markers.push_back(marker);

  trajectory_saver_pub.publish(marray);
  id++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_saver");
  WaypointSaver ws;
  ros::spin();
  return 0;
}
