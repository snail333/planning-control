#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>

#include <motion_planner/velocity_set/libvelocity_set.h>
#include <motion_planner/velocity_set/velocity_set_info.h>
#include <motion_planner/velocity_set/velocity_set_path.h>

namespace
{
constexpr int LOOP_RATE = 30;   //
constexpr double DECELERATION_SEARCH_DISTANCE = 50;  //减速waypoint搜索范围,40个点
constexpr double STOP_SEARCH_DISTANCE = 50;          //停止waypoint搜索范围,80个点

void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA &color, const double alpha=0.5)
{
  if (kind == EControl::STOP)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = alpha;  // red
  }
  else if (kind == EControl::STOPLINE)
  {
    color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = alpha;  // blue
  }
  else if (kind == EControl::DECELERATE)
  {
    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = alpha;  // yellow
  }
  else
  {
    color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = alpha;  // white
  }
}

// Display a detected obstacle
void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  static geometry_msgs::Point prev_obstacle_point;
  if (kind == EControl::STOP || kind == EControl::STOPLINE || kind == EControl::DECELERATE)
  {
    marker.pose.position = obstacle_points.getObstaclePoint(kind);
    prev_obstacle_point = marker.pose.position;
  }
  else  // kind == OTHERS
  {
    marker.pose.position = prev_obstacle_point;
  }
  geometry_msgs::Quaternion quat;
  marker.pose.orientation = quat;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;
  obstacleColorByKind(kind, marker.color, 0.7);

  obstacle_pub.publish(marker);
}


void displayDetectionRange(const common_msgs::Trajectory& lane, const int closest_waypoint,
                           const EControl& kind, const int obstacle_waypoint, const double stop_range,
                           const double deceleration_range, const ros::Publisher& detection_range_pub)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  waypoint_marker_stop.header.frame_id = "/map";
  waypoint_marker_stop.header.stamp = ros::Time();
  waypoint_marker_stop.id = 0;
  waypoint_marker_stop.type = visualization_msgs::Marker::SPHERE_LIST;
  waypoint_marker_stop.action = visualization_msgs::Marker::ADD;
  waypoint_marker_decelerate = waypoint_marker_stop;
  stop_line = waypoint_marker_stop;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2 * stop_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.8;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (stop_range + deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.5;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (obstacle_waypoint > -1)
  {
    stop_line.pose.position.x = lane.points[obstacle_waypoint].x;
    stop_line.pose.position.y = lane.points[obstacle_waypoint].y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, lane.points[obstacle_waypoint].angle);
    quaternionTFToMsg(quaternion, stop_line.pose.orientation);
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;
  obstacleColorByKind(kind, stop_line.color, 0.3);

  // set marker points coordinate
  for (int i = 0; i < STOP_SEARCH_DISTANCE; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.points.size()) - 1)
      break;

    geometry_msgs::Point point;
    point.x = lane.points[closest_waypoint + i].x;
    point.y = lane.points[closest_waypoint + i].y;

    waypoint_marker_stop.points.push_back(point);

    if (i > DECELERATION_SEARCH_DISTANCE)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind != EControl::KEEP)
    marker_array.markers.push_back(stop_line);
  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

int detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                       const common_msgs::Trajectory& lane, double stop_range,
                       double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                       ObstaclePoints* obstacle_points, EObstacleType* obstacle_type)
{
  int stop_obstacle_waypoint = -1;
  *obstacle_type = EObstacleType::NONE;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + STOP_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.points.size()))
      break;

    // waypoint seen by localizer
    geometry_msgs::Point point_lane;
    point_lane.x = lane.points[i].x;
    point_lane.y = lane.points[i].y;
    geometry_msgs::Point waypoint = calcRelativeCoordinate(point_lane, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt < stop_range)
      {
        stop_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (stop_point_count > points_threshold)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::ON_WAYPOINTS;
      break;
    }

    obstacle_points->clearStopPoints();

    // check next waypoint...
  }

  return stop_obstacle_waypoint;
}

int detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                             const common_msgs::Trajectory& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                             ObstaclePoints* obstacle_points)
{
  int decelerate_obstacle_waypoint = -1;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + DECELERATION_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.points.size()))
      break;

    // waypoint seen by localizer
    geometry_msgs::Point point_lane;
    point_lane.x = lane.points[i].x;
    point_lane.y = lane.points[i].y;
    geometry_msgs::Point waypoint = calcRelativeCoordinate(point_lane, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int decelerate_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt > stop_range && dt < stop_range + deceleration_range)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setDeceleratePoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > points_threshold)
    {
      decelerate_obstacle_waypoint = i;
      break;
    }

    obstacle_points->clearDeceleratePoints();

    // check next waypoint...
  }

  return decelerate_obstacle_waypoint;
}

// Detect an obstacle by using pointcloud
EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                         const common_msgs::Trajectory& lane, const VelocitySetInfo& vs_info,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points)
{
  // no input for detection || no closest waypoint
  if (points.empty() == true  || closest_waypoint < 0)
    return EControl::KEEP;

  EObstacleType obstacle_type = EObstacleType::NONE;
  int stop_obstacle_waypoint =
      detectStopObstacle(points, closest_waypoint, lane, vs_info.getStopRange(),
                         vs_info.getPointsThreshold(), vs_info.getLocalizerPose(),
                         obstacle_points, &obstacle_type);

  // skip searching deceleration range
  if (vs_info.getDecelerationRange() < 0.01)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    if (stop_obstacle_waypoint < 0)
      return EControl::KEEP;
    else if (obstacle_type == EObstacleType::ON_WAYPOINTS || obstacle_type == EObstacleType::ON_CROSSWALK)
      return EControl::STOP;
    else if (obstacle_type == EObstacleType::STOPLINE)
      return EControl::STOPLINE;
    else
      return EControl::OTHERS;
  }

  int decelerate_obstacle_waypoint =
      detectDecelerateObstacle(points, closest_waypoint, lane, vs_info.getStopRange(), vs_info.getDecelerationRange(),
                               vs_info.getPointsThreshold(), vs_info.getLocalizerPose(), obstacle_points);

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return decelerate_obstacle_waypoint < 0 ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }

  // about 5.0 meter
  geometry_msgs::Point target1;
  geometry_msgs::Point target2;
  target1.x = lane.points[0].x;
  target1.y = lane.points[0].y;
  target2.x = lane.points[1].x;
  target2.y = lane.points[1].y;
  double waypoint_interval =
      getPlaneDistance(target1, target2);
  int stop_decelerate_threshold = 5 / waypoint_interval;

  // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return EControl::DECELERATE;
  }
  else
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }
}

EControl obstacleDetection(int closest_waypoint, const common_msgs::Trajectory& lane,
                           const VelocitySetInfo vs_info, const ros::Publisher& detection_range_pub,
                           const ros::Publisher& obstacle_pub, int* obstacle_waypoint)
{
  ObstaclePoints obstacle_points;
  EControl detection_result = pointsDetection(vs_info.getPoints(), closest_waypoint, lane, vs_info,
                                              obstacle_waypoint, &obstacle_points);
  displayDetectionRange(lane, closest_waypoint, detection_result, *obstacle_waypoint, vs_info.getStopRange(),
                        vs_info.getDecelerationRange(), detection_range_pub);

  static int false_count = 0;
  static EControl prev_detection = EControl::KEEP;
  static int prev_obstacle_waypoint = -1;

  // stop or decelerate because we found obstacles
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE)
  {
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
    prev_obstacle_waypoint = *obstacle_waypoint;
    return detection_result;
  }

  // there are no obstacles, but wait a little for safety
  if (prev_detection == EControl::STOP || prev_detection == EControl::STOPLINE || prev_detection == EControl::DECELERATE)
  {
    false_count++;

    if (false_count < LOOP_RATE / 5)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
      displayObstacle(EControl::OTHERS, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }

  // there are no obstacles, so we move forward
  *obstacle_waypoint = -1;
  false_count = 0;
  prev_detection = EControl::KEEP;
  return detection_result;
}

void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path)
{
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {  // STOP for obstacle/stopline
    // stop_waypoint is about stop_distance meter away from obstacles/stoplines
    int stop_distance = (detection_result == EControl::STOP)
      ? vs_info.getStopDistanceObstacle() : vs_info.getStopDistanceStopline();
    double deceleration = (detection_result == EControl::STOP)
      ? vs_info.getDecelerationObstacle() : vs_info.getDecelerationStopline();
    int stop_waypoint =
        calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, stop_distance);
    // change waypoints to stop by the stop_waypoint
    vs_path->changeWaypointsForStopping(stop_waypoint, obstacle_waypoint, closest_waypoint, deceleration);
    //vs_path->avoidSuddenAcceleration(deceleration, closest_waypoint);
    vs_path->avoidSuddenAcceleration(vs_info.getAcceleration(), closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), deceleration, closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else if (detection_result == EControl::DECELERATE)
  {  // DECELERATE for obstacles
    vs_path->initializeNewWaypoints();
    vs_path->changeWaypointsForDeceleration(vs_info.getDecelerationObstacle(), closest_waypoint, obstacle_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    //vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenAcceleration(vs_info.getAcceleration(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else
  {  // ACCELERATE or KEEP
    vs_path->initializeNewWaypoints();
    vs_path->avoidSuddenAcceleration(vs_info.getAcceleration(), closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
}

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string points_topic;
  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");

  // class
  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;

  //ros::Subscriber points_sub = nh.subscribe(points_topic, 10, &VelocitySetInfo::pointsCallback, &vs_info);
  //ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 10, &VelocitySetInfo::localizerPoseCallback, &vs_info);
  ros::Subscriber control_pose_sub = nh.subscribe("localization", 10, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber current_pose_sub = nh.subscribe("localization", 10, &VelocitySetPath::currentPoseCallback, &vs_path);
  ros::Subscriber waypoints_sub = nh.subscribe("base_waypoints", 10, &VelocitySetPath::waypointsCallback, &vs_path);

  // publisher
  ros::Publisher detection_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);
  ros::Publisher stopline_waypoint_pub = nh.advertise<std_msgs::Int32>("stopline_waypoint", 1, true);
  ros::Publisher final_waypoints_pub;
  final_waypoints_pub = nh.advertise<common_msgs::Trajectory>("final_waypoints", 1, true);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();
    int closest_waypoint = 0;

    if (!vs_info.getSetPose() || !vs_path.getSetPath() || vs_path.getPrevWaypointsSize() == 0)
    {
      loop_rate.sleep();
      continue;
    }


    int obstacle_waypoint = -1;
    EControl detection_result = obstacleDetection(closest_waypoint, vs_path.getPrevWaypoints(), vs_info,
                                                  detection_range_pub, obstacle_pub, &obstacle_waypoint);

    changeWaypoints(vs_info, detection_result, closest_waypoint,
                    obstacle_waypoint, final_waypoints_pub, &vs_path);

    vs_info.clearPoints();

    // publish obstacle waypoint index
    std_msgs::Int32 obstacle_waypoint_index;
    std_msgs::Int32 stopline_waypoint_index;
    if (detection_result == EControl::STOP || detection_result == EControl::DECELERATE)
    {
      obstacle_waypoint_index.data = obstacle_waypoint;
      stopline_waypoint_index.data = -1;
    }
    else if (detection_result == EControl::STOPLINE)
    {
      obstacle_waypoint_index.data = -1;
      stopline_waypoint_index.data = obstacle_waypoint;
    }
    else
    {
      obstacle_waypoint_index.data = -1;
      stopline_waypoint_index.data = -1;
    }
    obstacle_waypoint_pub.publish(obstacle_waypoint_index);
    stopline_waypoint_pub.publish(stopline_waypoint_index);

    vs_path.resetFlag();

    loop_rate.sleep();
  }

  return 0;
}
