#include "trajectory_replanner.h"

namespace trajectory_maker
{

WaypointReplanner::WaypointReplanner()
{
}

WaypointReplanner::~WaypointReplanner()
{
}

void WaypointReplanner::updateConfig(const WaypointReplannerConfig& config)
{
  config_ = config;
  config_.radius_inf = 10 * config_.radius_thresh;
  config_.velocity_param = calcVelParam(config_.velocity_max);
}


void WaypointReplanner::changeVelSign(common_msgs::Trajectory& lane, bool positive) const
{
  const int sgn = positive ? 1 : -1;
  for (auto& el : lane.points)
  {
    el.velocity = sgn * fabs(el.velocity);
  }
}

void WaypointReplanner::replanLaneWaypointVel(common_msgs::Trajectory& lane)
{
  if (lane.points.empty())
  {
    return;
  }
  const LaneDirection dir = getLaneDirection(lane);
  unsigned long last = lane.points.size() - 1;
  changeVelSign(lane, true);
  limitVelocityByRange(0, last, 0, config_.velocity_max, lane);
  if (config_.resample_mode)
  {
    resampleLaneWaypoint(config_.resample_interval, lane, dir);
    last = lane.points.size() - 1;
  }
  if (config_.replan_curve_mode)
  {
    std::vector<double> curve_radius;
    KeyVal curve_list;
    createRadiusList(lane, curve_radius);
    createCurveList(curve_radius, curve_list);
    if (config_.overwrite_vmax_mode)
    {// set velocity_max for all_point
      setVelocityByRange(0, last, 0, config_.velocity_max, lane);
    }
    // set velocity by curve
    for (const auto& el : curve_list)
    {
      const double& radius = el.second.second;
      //double vmin = config_.velocity_max - config_.velocity_param * (config_.radius_thresh - radius);
      double vmin = config_.curvel_max - config_.velocity_param * (config_.radius_thresh - radius);
      vmin = (vmin < config_.velocity_min) ? config_.velocity_min : vmin;
      limitVelocityByRange(el.first, el.second.first, config_.velocity_offset, vmin, lane);
    }
  }
  // set velocity on start & end of lane
  if (config_.replan_endpoint_mode)
  {
    const unsigned long zerospeed_start = last - config_.end_point_offset;
    const unsigned long lowspeed_start = zerospeed_start - config_.braking_distance;
    raiseVelocityByRange(0, last, 0, config_.velocity_min, lane);
    limitVelocityByRange(0, 0, 0, config_.velocity_min, lane);
    limitVelocityByRange(lowspeed_start, last, 0, config_.velocity_min, lane);
    setVelocityByRange(zerospeed_start, last, 0, 0.0, lane);
  }
  if (dir == LaneDirection::Backward)
  {
    changeVelSign(lane, false);
  }
}

void WaypointReplanner::resampleLaneWaypoint(const double resample_interval, common_msgs::Trajectory& lane, LaneDirection dir)
{
  if (lane.points.size() < 2)
  {
    return;
  }
  common_msgs::Trajectory original_lane(lane);
  lane.points.clear();
  lane.points.emplace_back(original_lane.points[0]);
  lane.points.reserve(ceil(1.5 * calcPathLength(original_lane) / config_.resample_interval));

  for (unsigned long i = 1; i < original_lane.points.size(); i++)
  {
    CbufGPoint curve_point = getCrvPointsOnResample(lane, original_lane, i);
    const std::vector<double> curve_param = calcCurveParam(curve_point);
    lane.points.back().velocity = original_lane.points[i - 1].velocity;
    //lane.points.back().twist.twist = original_lane.points[i - 1].twist.twist;
    // lane.points.back().wpstate = original_lane.points[i - 1].wpstate;
    // lane.points.back().change_flag = original_lane.points[i - 1].change_flag;
    // if going straight
    if (curve_param.empty())
    {
      resampleOnStraight(curve_point, lane, dir);
    }
    // else if turnning curve
    else
    {
      resampleOnCurve(curve_point[1], curve_param, lane, dir);
    }
  }
  lane.points[0].angle = lane.points[1].angle;
  lane.points.back().velocity = original_lane.points.back().velocity;
  // lane.points.back().wpstate = original_lane.points.back().wpstate;
  // lane.points.back().change_flag = original_lane.points.back().change_flag;
}

void WaypointReplanner::resampleOnStraight(const CbufGPoint& curve_point, common_msgs::Trajectory& lane, LaneDirection dir)
{
  if (curve_point.size() != 3)
  {
    return;
  }
  common_msgs::TrajectoryPoint wp(lane.points.back());
  geometry_msgs::Point pt;
  pt.x = wp.x;
  pt.y = wp.y;
  pt.z = 0;
  const int sgn = (dir == LaneDirection::Forward) ? 0 : 1;
  const double yaw = atan2(curve_point[2].y - curve_point[0].y, curve_point[2].x - curve_point[0].x) + sgn * M_PI;
  //wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  wp.angle = yaw;

  const std::vector<double> nvec = { curve_point[1].x - pt.x, curve_point[1].y - pt.y, curve_point[1].z - pt.z };
  double dist = sqrt(nvec[0] * nvec[0] + nvec[1] * nvec[1]);
  std::vector<double> resample_vec = nvec;
  const double coeff = config_.resample_interval / dist;
  for (auto& el : resample_vec)
  {
    el *= coeff;
  }
  for (; dist > config_.resample_interval; dist -= config_.resample_interval)
  {
    wp.x += resample_vec[0];
    wp.y += resample_vec[1];
    //wp.pose.pose.position.z += resample_vec[2];
    lane.points.emplace_back(wp);
  }
}

void WaypointReplanner::resampleOnCurve(const geometry_msgs::Point& target_point,
                                        const std::vector<double>& curve_param, common_msgs::Trajectory& lane, LaneDirection dir)
{
  if (curve_param.size() != 3)
  {
    return;
  }
  common_msgs::TrajectoryPoint wp(lane.points.back());
  const double& cx = curve_param[0];
  const double& cy = curve_param[1];
  const double& radius = curve_param[2];
  const double reverse_angle = (dir == LaneDirection::Backward) ? M_PI : 0.0;

  geometry_msgs::Point p0;
  p0.x = wp.x;
  p0.y = wp.y;
  p0.z = 0;
  const geometry_msgs::Point& p1 = target_point;
  double theta = fmod(atan2(p1.y - cy, p1.x - cx) - atan2(p0.y - cy, p0.x - cx), 2 * M_PI);
  int sgn = (theta > 0.0) ? (1) : (-1);
  if (fabs(theta) > M_PI)
  {
    theta -= 2 * sgn * M_PI;
  }
  sgn = (theta > 0.0) ? (1) : (-1);
  // interport
  double t = atan2(p0.y - cy, p0.x - cx);
  double dist = radius * fabs(theta);
  double rescurve_interval = 0.6 * config_.resample_interval;            //set curve resample_interval
  //const double resample_dz = rescurve_interval * (p1.z - p0.z) / dist;
  for (; dist > rescurve_interval; dist -= rescurve_interval)
  {
    if (lane.points.size() == lane.points.capacity())
    {
      break;
    }
    t += sgn * rescurve_interval / radius;
    const double yaw = fmod(t + sgn * M_PI / 2.0, 2 * M_PI) + reverse_angle;
    wp.x = cx + radius * cos(t);
    wp.y = cy + radius * sin(t);
    wp.angle = yaw;
    //wp.pose.pose.position.z += resample_dz;
    //wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    lane.points.emplace_back(wp);
  }
}

// Three points used for curve detection (the target point is the center)
// [0] = previous point, [1] = target point, [2] = next point
const CbufGPoint WaypointReplanner::getCrvPointsOnResample(
    const common_msgs::Trajectory& lane, const common_msgs::Trajectory& original_lane, unsigned long original_index) const
{
  unsigned long id = original_index;
  CbufGPoint curve_point(3);
  const unsigned int n = (config_.lookup_crv_width - 1) / 2;
  const common_msgs::TrajectoryPoint cp[3] = {
    (lane.points.size() < n) ? lane.points.front() : lane.points[lane.points.size() - n],
    original_lane.points[id],
    (id < original_lane.points.size() - n) ? original_lane.points[id + n] : original_lane.points.back()
  };
  for (int i = 0; i < 3; i++)
  {
    geometry_msgs::Point pt;
    pt.x = cp[i].x;
    pt.y = cp[i].y;
    pt.z = 0;
    curve_point.push_back(pt);
  }
  return curve_point;
}

const CbufGPoint WaypointReplanner::getCrvPoints(const common_msgs::Trajectory& lane, unsigned long index) const
{
  CbufGPoint curve_point(3);
  const unsigned int n = (config_.lookup_crv_width - 1) / 2;
  const unsigned long curve_index[3] = { (index < n) ? 0 : (index - n), index, (index >= lane.points.size() - n) ?
                                                                                   (lane.points.size() - 1) :
                                                                                   (index + n) };
  for (int i = 0; i < 3; i++)
  {
    geometry_msgs::Point pt;
    pt.x = lane.points[curve_index[i]].x;
    pt.y = lane.points[curve_index[i]].y;
    pt.z = 0;
    curve_point.push_back(pt);
  }
  return curve_point;
}

void WaypointReplanner::createRadiusList(const common_msgs::Trajectory& lane, std::vector<double>& curve_radius)
{
  if (lane.points.empty())
  {
    return;
  }
  curve_radius.resize(lane.points.size());
  curve_radius.at(0) = curve_radius.back() = config_.radius_inf;

  for (unsigned long i = 1; i < lane.points.size() - 1; i++)
  {
    CbufGPoint curve_point(getCrvPoints(lane, i));
    const std::vector<double> curve_param(calcCurveParam(curve_point));

    // if going straight
    if (curve_param.empty())
    {
      curve_radius.at(i) = config_.radius_inf;
    }
    // else if turnning curve
    else
    {
      curve_radius.at(i) = (curve_param[2] > config_.radius_inf) ? config_.radius_inf : curve_param[2];
    }
  }
}

const double WaypointReplanner::calcVelParam(double vmax) const
{
  if (fabs(config_.radius_thresh - config_.radius_min) < 1e-8)
  {
    return DBL_MAX;  // error
  }
  return (vmax - config_.velocity_min) / (config_.radius_thresh - config_.radius_min);
}

void WaypointReplanner::createCurveList(const std::vector<double>& curve_radius, KeyVal& curve_list)
{
  unsigned long index = 0;
  bool on_curve = false;
  double radius_localmin = DBL_MAX;
  for (unsigned long i = 1; i < curve_radius.size(); i++)
  {
    if (!on_curve && curve_radius[i] <= config_.radius_thresh && curve_radius[i - 1] > config_.radius_thresh)
    {
      index = i;
      on_curve = true;
    }
    else if (on_curve && curve_radius[i - 1] <= config_.radius_thresh && curve_radius[i] > config_.radius_thresh)
    {
      on_curve = false;
      if (radius_localmin < config_.radius_min)
      {
        radius_localmin = config_.radius_min;
      }
      curve_list[index] = std::make_pair(i, radius_localmin);
      radius_localmin = DBL_MAX;
    }
    if (!on_curve)
    {
      continue;
    }
    if (radius_localmin > curve_radius[i])
    {
      radius_localmin = curve_radius[i];
    }
  }
}


void WaypointReplanner::setVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vel, common_msgs::Trajectory& lane)
{
  if (lane.points.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.points.size()) ? lane.points.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    lane.points[idx].velocity = vel;
  }
}

void WaypointReplanner::raiseVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                           double vmin, common_msgs::Trajectory& lane)
{
  if (lane.points.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.points.size()) ? lane.points.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane.points[idx].velocity >= vmin)
    {
      continue;
    }
    lane.points[idx].velocity = vmin;
  }
}

void WaypointReplanner::limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vmin, common_msgs::Trajectory& lane)
{
  if (lane.points.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.points.size()) ? lane.points.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane.points[idx].velocity < vmin)
    {
      continue;
    }
    lane.points[idx].velocity = vmin;
  }
  limitAccelDecel(start_idx, lane);
  limitAccelDecel(end_idx, lane);
}

void WaypointReplanner::limitAccelDecel(const unsigned long idx, common_msgs::Trajectory& lane)
{
  const double acc[2] = { config_.accel_limit, config_.decel_limit };
  const unsigned long end_idx[2] = { lane.points.size() - idx, idx + 1 };
  const int sgn[2] = { 1, -1 };
  for (int j = 0; j < 2; j++)  // [j=0]: config_.accel_limitprocess, [j=1]: config_.decel_limitprocess
  {
    double v = lane.points[idx].velocity;
    unsigned long next = idx + sgn[j];
    for (unsigned long i = 1; i < end_idx[j]; i++, next += sgn[j])
    {
      geometry_msgs::Point p0;
      p0.x = lane.points[next - sgn[j]].x;
      p0.y = lane.points[next - sgn[j]].y;
      geometry_msgs::Point p1;
      p1.x = lane.points[next].x;
      p1.y = lane.points[next].y;
      const double dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
      v = sqrt(2 * acc[j] * dist + v * v);
      if (v > config_.velocity_max || v > lane.points[next].velocity)
      {
        break;
      }
      lane.points[next].velocity = v;
    }
  }
}

// get curve 3-Parameter [center_x, center_y, radius] with 3 point input. If error occured, return empty vector.
const std::vector<double> WaypointReplanner::calcCurveParam(CbufGPoint p) const
{
  for (int i = 0; i < 3; i++, p.push_back(p.front()))  // if exception occured, change points order
  {
    const double d = 2 * ((p[0].y - p[2].y) * (p[0].x - p[1].x) - (p[0].y - p[1].y) * (p[0].x - p[2].x));
    if (fabs(d) < 1e-8)
    {
      continue;
    }
    const std::vector<double> x2 = { p[0].x * p[0].x, p[1].x * p[1].x, p[2].x * p[2].x };
    const std::vector<double> y2 = { p[0].y * p[0].y, p[1].y * p[1].y, p[2].y * p[2].y };
    const double a = y2[0] - y2[1] + x2[0] - x2[1];
    const double b = y2[0] - y2[2] + x2[0] - x2[2];
    std::vector<double> param(3);
    const double cx = param[0] = ((p[0].y - p[2].y) * a - (p[0].y - p[1].y) * b) / d;
    const double cy = param[1] = ((p[0].x - p[2].x) * a - (p[0].x - p[1].x) * b) / -d;
    param[2] = sqrt((cx - p[0].x) * (cx - p[0].x) + (cy - p[0].y) * (cy - p[0].y));
    if(param[2] < 50)
      return param;
  }
  return std::vector<double>();  // error
}

const double WaypointReplanner::calcPathLength(const common_msgs::Trajectory& lane) const
{
  double distance = 0.0;
  for (unsigned long i = 1; i < lane.points.size(); i++)
  {
    geometry_msgs::Point p0;
    p0.x = lane.points[i - 1].x;
    p0.y = lane.points[i - 1].y;
    geometry_msgs::Point p1;
    p1.x = lane.points[i].x;
    p1.y = lane.points[i].y;
    tf::Vector3 tf0(p0.x, p0.y, 0.0);
    tf::Vector3 tf1(p1.x, p1.y, 0.0);
    distance += tf::tfDistance(tf0, tf1);
  }
  return distance;
}

};
