#ifndef LIBTRAJECTORY_FOLLOWER_TEST_LIBTRAJECTORY_FOLLOWER_H
#define LIBTRAJECTORY_FOLLOWER_TEST_LIBTRAJECTORY_FOLLOWER_H

#include <gtest/gtest.h>
#include "libtrajectory_follower/libtrajectory_follower.h"

enum class CoordinateResult
{
  Positive = 1,
  Negative = -1,
  Equal = 0
};

struct DirectionCheckDataSet
{
  int idx;
  double vel;
  DirectionCheckDataSet(int i, double v) :
    idx(i), vel(v)
  {}
  DirectionCheckDataSet() {}
};

struct ClosestCheckDataSet
{
  int dir;
  double vel;
  double offset;
  int num;
  geometry_msgs::PoseStamped pose;
  ClosestCheckDataSet(int d, double v, double o, int n, const geometry_msgs::PoseStamped& p)
    : dir(d), vel(v), offset(o), num(n), pose(p) {}
  ClosestCheckDataSet() {}
};

class LibWaypointFollowerTestClass
{
public:
  LibWaypointFollowerTestClass() {}
  common_msgs::Trajectory generateLane(int driving_direction, double velocity)
  {
    return std::move(generateOffsetLane(driving_direction, velocity, 0.0, 100));
  }

  common_msgs::Trajectory generateOffsetLane(int driving_direction, double velocity, double offset, int num)
  {
    common_msgs::Trajectory lane;
    for (int idx = 0; idx < num; idx++)
    {
      static common_msgs::TrajectoryPoint wp;
      // wp.gid = idx;
      // wp.lid = idx;
      // wp.pose.pose.position.x = driving_direction * (static_cast<double>(idx) + offset);
      // wp.pose.pose.position.y = 0.0;
      // wp.pose.pose.position.z = 0.0;
      // wp.twist.twist.linear.x = velocity;
      // wp.twist.twist.angular.z = 0.0;

      // tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      // quaternionTFToMsg(quaternion, wp.pose.pose.orientation);

      wp.x = driving_direction * (static_cast<double>(idx) + offset);
      wp.y = 0.0;
      wp.velocity = velocity;
      wp.angle = 0;
      
      lane.points.emplace_back(wp);
    }
    return std::move(lane);
  }

  geometry_msgs::PoseStamped generateCurrentPose(double x, double y, double yaw)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
    quaternionTFToMsg(quaternion, pose.pose.orientation);
    return std::move(pose);
  }
};

#endif  // LIBTRAJECTORY_FOLLOWER_TEST_LIBTRAJECTORY_FOLLOWER_H
