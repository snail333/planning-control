/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
//#include <autoware_msgs/Lane.h>
#include <common_msgs/Trajectory.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
class MPCWaypointsConverter
{
public:
  MPCWaypointsConverter()
  {
    pub_waypoints_ = nh_.advertise<common_msgs::Trajectory>("/mpc_waypoints", 1);
    sub_closest_waypoint_ = nh_.subscribe("/closest_waypoint", 1, &MPCWaypointsConverter::callbackClosestWaypoints, this);
    sub_base_waypoints_ = nh_.subscribe("/base_waypoints", 1, &MPCWaypointsConverter::callbackBaseWaypoints, this);
    sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &MPCWaypointsConverter::callbackFinalWaypoints, this);

    closest_idx_ = 0;
    back_waypoints_num_ = 10;
    front_waypoints_num_ = 50;
  };
  ~MPCWaypointsConverter(){};

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_waypoints_;
  ros::Subscriber sub_closest_waypoint_, sub_base_waypoints_, sub_final_waypoints_, sub_current_velocity_;

  common_msgs::Trajectory base_waypoints_;
  int closest_idx_;
  int back_waypoints_num_;
  int front_waypoints_num_;

  void callbackClosestWaypoints(const std_msgs::Int32 msg)
  {
    closest_idx_ = msg.data;
  }

  void callbackBaseWaypoints(const common_msgs::Trajectory &msg)
  {
    base_waypoints_ = msg;
  }

  void callbackFinalWaypoints(const common_msgs::Trajectory &final_waypoints)
  {
    if (base_waypoints_.points.size() == 0 || final_waypoints.points.size() == 0)
      return;

    if ((int)base_waypoints_.points.size() - 1 < closest_idx_)
    {
      ROS_WARN("base_waypoints_.waypoints.size() - 1 = %d, closest_idx_ = %d", (int)base_waypoints_.points.size(), closest_idx_);
      return;
    }

    auto sq_dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };

    common_msgs::Trajectory mpc_waypoints;
    mpc_waypoints.header = final_waypoints.header;
    mpc_waypoints.lane_ids = final_waypoints.lane_ids;  
    /*
    mpc_waypoints.increment = final_waypoints.increment;
    mpc_waypoints.lane_id = final_waypoints.lane_id;
    mpc_waypoints.lane_index = final_waypoints.lane_index;
    mpc_waypoints.cost = final_waypoints.cost;
    mpc_waypoints.closest_object_distance = final_waypoints.closest_object_distance;
    mpc_waypoints.closest_object_velocity = final_waypoints.closest_object_velocity;
    mpc_waypoints.is_blocked = final_waypoints.is_blocked;*/

    // find closest point index in base_waypoints (topic /closest_waypoints has no consistency with /final_waypoints due to delay)
    int closest_idx = -1;
    for (int i = 0; i < (int)base_waypoints_.points.size(); ++i) {
    geometry_msgs::Pose pos0;
    pos0.position.x = final_waypoints.points.at(1).x;
    pos0.position.y = final_waypoints.points.at(1).y;
    pos0.position.z = 0;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, final_waypoints.points.at(1).angle);
    quaternionTFToMsg(quaternion, pos0.orientation);

     geometry_msgs::Pose pos1;
    pos1.position.x = base_waypoints_.points.at(i).x;
    pos1.position.y = base_waypoints_.points.at(i).y;
    pos1.position.z = 0;
    tf::Quaternion quaternion1 = tf::createQuaternionFromRPY(0, 0, base_waypoints_.points.at(i).angle);
    quaternionTFToMsg(quaternion1, pos1.orientation);  
      const double d = sq_dist(pos0.position, pos1.position);
      if (d < 0.01) {
        closest_idx = i;
        break;
      }
    }
    if (closest_idx == -1) {
      ROS_ERROR("cannot find closest base_waypoints' waypoint to final_waypoints.waypoint[1] !!");
    }

    int base_start = std::max(closest_idx - back_waypoints_num_, 0);
    for (int i = base_start; i < closest_idx; ++i)
    {
      mpc_waypoints.points.push_back(base_waypoints_.points.at(i));
      mpc_waypoints.points.back().velocity = final_waypoints.points[1].velocity;
    }

    int final_end = std::min(front_waypoints_num_ + 1, (int)final_waypoints.points.size());
    for (int i = 1; i < final_end; ++i)
    {
      mpc_waypoints.points.push_back(final_waypoints.points.at(i));
    }
    
    pub_waypoints_.publish(mpc_waypoints);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_waypoints_converter");
  MPCWaypointsConverter obj;
  ros::spin();
  return 0;
};
