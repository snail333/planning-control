#include <ros/ros.h>
#include <common_msgs/TrajectoryArray.h>
#include "trajectory_replanner.h"

namespace trajectory_maker
{
class WaypointReplannerNode
{
public:
  WaypointReplannerNode();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher lane_pub_;
  ros::Subscriber lane_sub_;
  bool replanning_mode_;
  WaypointReplanner replanner_;
  common_msgs::TrajectoryArray lane_array_;
  void replan(common_msgs::TrajectoryArray &lane_array);
  void publishLaneArray();
  void laneCallback(const common_msgs::TrajectoryArray::ConstPtr& lane_array);

};

WaypointReplannerNode::WaypointReplannerNode() : pnh_("~")
{
  WaypointReplannerConfig temp_config;

  double velocity_max_kph, velocity_min_kph, curvel_max_kph;

  pnh_.param<bool>("replanning_mode", replanning_mode_, false);
  pnh_.param<double>("velocity_max", velocity_max_kph, 0.0);
  pnh_.param<double>("curvel_max", curvel_max_kph, 0.0);
  pnh_.param<double>("velocity_min", velocity_min_kph, 0.0);
  pnh_.param<double>("accel_limit", temp_config.accel_limit, 0.0);
  pnh_.param<double>("decel_limit", temp_config.decel_limit, 0.0);
  pnh_.param<double>("radius_thresh", temp_config.radius_thresh, 0.0);
  pnh_.param<double>("radius_min", temp_config.radius_min, 0.0);
  pnh_.param<bool>("resample_mode", temp_config.resample_mode, false);
  pnh_.param<double>("resample_interval", temp_config.resample_interval, 0.0);
  pnh_.param<bool>("replan_curve_mode", temp_config.replan_curve_mode, false);
  pnh_.param<bool>("replan_endpoint_mode", temp_config.replan_endpoint_mode, false);
  pnh_.param<bool>("overwrite_vmax_mode", temp_config.overwrite_vmax_mode, false);
  pnh_.param<double>("velocity_offset", temp_config.velocity_offset, 0.0);
  pnh_.param<double>("end_point_offset", temp_config.end_point_offset, 0.0);
  pnh_.param<double>("braking_distance", temp_config.braking_distance, 0.0);

  temp_config.velocity_max = kmph2mps(velocity_max_kph);
  temp_config.velocity_min = kmph2mps(velocity_min_kph);
  temp_config.curvel_max = kmph2mps(curvel_max_kph);
  temp_config.lookup_crv_width = 5;
  
  replanner_.updateConfig(temp_config);
  lane_pub_ = nh_.advertise<common_msgs::TrajectoryArray>("/lane_waypoints_array", 10, true);

  lane_sub_ = nh_.subscribe("/based/lane_waypoints_raw", 1, &WaypointReplannerNode::laneCallback, this);

}

void WaypointReplannerNode::replan(common_msgs::TrajectoryArray& lane_array)
{
  for (auto &el : lane_array.trajectories)
  {
    replanner_.replanLaneWaypointVel(el);
  }
}

void WaypointReplannerNode::publishLaneArray()
{
  common_msgs::TrajectoryArray array(lane_array_);

  if (replanning_mode_)
  {
    replan(array);
  }

  lane_pub_.publish(array);

}

void WaypointReplannerNode::laneCallback(const common_msgs::TrajectoryArray::ConstPtr& lane_array)
{
  lane_array_ = *lane_array;
  publishLaneArray();
}


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_replanner");
  trajectory_maker::WaypointReplannerNode wr;
  ros::spin();

  return 0;
}
