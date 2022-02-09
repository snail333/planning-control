#ifndef __TRAJECTORY_REPLANNER_H__
#define __TRAJECTORY_REPLANNER_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include <common_msgs/Trajectory.h>
#include <libtrajectory_follower/libtrajectory_follower.h>

namespace trajectory_maker
{
typedef std::unordered_map<unsigned long, std::pair<unsigned long, double>> KeyVal;
typedef boost::circular_buffer<geometry_msgs::Point> CbufGPoint;

struct WaypointReplannerConfig
{
  double velocity_max = 0.0;
  double curvel_max = 0.0;
  double velocity_min = 0.0;
  double velocity_param = 0.0;
  double accel_limit = 0.0;
  double decel_limit = 0.0;
  double radius_thresh = 0.0;
  double radius_min = 0.0;
  double radius_inf = 0.0;
  bool resample_mode = false;
  double resample_interval = 0.0;
  bool replan_curve_mode = false;
  bool replan_endpoint_mode = false;
  bool overwrite_vmax_mode = false;
  double velocity_offset = 0.0;
  double end_point_offset =  0.0;
  double braking_distance = 0.0;
  int lookup_crv_width = 5;
};

class WaypointReplanner
{
private:
  WaypointReplannerConfig config_;

public:
  WaypointReplanner();
  ~WaypointReplanner();
  void updateConfig(const WaypointReplannerConfig& config);
  void replanLaneWaypointVel(common_msgs::Trajectory& lane);

protected:
  void changeVelSign(common_msgs::Trajectory& lane, bool positive) const;
  void resampleLaneWaypoint(const double resample_interval, common_msgs::Trajectory& lane, LaneDirection dir);
  void resampleOnStraight(const CbufGPoint& curve_point, common_msgs::Trajectory& lane, LaneDirection dir);
  void resampleOnCurve(const geometry_msgs::Point& target_point,
    const std::vector<double>& param,common_msgs::Trajectory& lane, LaneDirection dir);

  const CbufGPoint getCrvPointsOnResample(const common_msgs::Trajectory& lane,
    const common_msgs::Trajectory& original_lane, unsigned long original_index) const;
  const CbufGPoint getCrvPoints(const common_msgs::Trajectory& lane, unsigned long index) const;

  void createRadiusList(const common_msgs::Trajectory& lane, std::vector<double>& curve_radius);
  const double calcVelParam(double vmax) const;
  void createCurveList(const std::vector<double>& curve_radius, KeyVal& curve_list);
  void createVmaxList(const common_msgs::Trajectory& lane, const KeyVal &curve_list,
    unsigned long offset, KeyVal &vmax_list);
  double searchVmaxByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
    const common_msgs::Trajectory &lane) const;
  void setVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vel,
    common_msgs::Trajectory& lane);
  void raiseVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
    double vmin, common_msgs::Trajectory& lane);

  void limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vmin,
    common_msgs::Trajectory& lane);
  void limitAccelDecel(const unsigned long idx, common_msgs::Trajectory& lane);

  const std::vector<double> calcCurveParam(CbufGPoint point) const;
  const double calcPathLength(const common_msgs::Trajectory& lane) const;
};
}
#endif
