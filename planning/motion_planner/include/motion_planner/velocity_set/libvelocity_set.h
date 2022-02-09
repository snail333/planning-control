#ifndef _VELOCITY_SET_H
#define _VELOCITY_SET_H

#include <math.h>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <common_msgs/Trajectory.h>
#include <libtrajectory_follower/libtrajectory_follower.h>

enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4,
};

enum class EObstacleType
{
  NONE = -1,
  ON_WAYPOINTS = 1,
  ON_CROSSWALK = 2,
  STOPLINE = 3,
};

//////////////////////////////////////
// for visualization of obstacles
//////////////////////////////////////
class ObstaclePoints
{
private:
  std::vector<geometry_msgs::Point> stop_points_;
  std::vector<geometry_msgs::Point> decelerate_points_;
  geometry_msgs::Point previous_detection_;

public:
  void setStopPoint(const geometry_msgs::Point &p)
  {
    stop_points_.push_back(p);
  }
  void setDeceleratePoint(const geometry_msgs::Point &p)
  {
    decelerate_points_.push_back(p);
  }
  void clearStopPoints()
  {
    stop_points_.clear();
  }
  void clearDeceleratePoints()
  {
    decelerate_points_.clear();
  }

  ObstaclePoints() : stop_points_(0), decelerate_points_(0)
  {
  }

  geometry_msgs::Point getObstaclePoint(const EControl &kind) const
  {
    geometry_msgs::Point point;

    if (kind == EControl::STOP || kind == EControl::STOPLINE)
    {
      for (const auto &p : stop_points_)
      {
        point.x += p.x;
        point.y += p.y;
        point.z += p.z;
      }
      point.x /= stop_points_.size();
      point.y /= stop_points_.size();
      point.z /= stop_points_.size();

      return point;
    }
    else  // kind == DECELERATE
    {
      for (const auto &p : decelerate_points_)
      {
        point.x += p.x;
        point.y += p.y;
        point.z += p.z;
      }
      point.x /= decelerate_points_.size();
      point.y /= decelerate_points_.size();
      point.z /= decelerate_points_.size();

      return point;
    }
  }
};

inline double calcSquareOfLength(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

// Calculate waypoint index corresponding to distance from begin_waypoint
inline int calcWaypointIndexReverse(const common_msgs::Trajectory &lane, const int begin_waypoint, const double distance)
{
  double dist_sum = 0;
  for (int i = begin_waypoint; i > 0; i--)
  {
    tf::Vector3 v1(lane.points[i].x, lane.points[i].y, 0);

    tf::Vector3 v2(lane.points[i - 1].x, lane.points[i - 1].y, 0);

    dist_sum += tf::tfDistance(v1, v2);

    if (dist_sum > distance)
      return i;
  }

  // reach the first waypoint
  return 0;
}

#endif /* _VELOCITY_SET_H */
