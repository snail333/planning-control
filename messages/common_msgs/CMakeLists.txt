cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  jsk_recognition_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Business.msg
    CanRecieve.msg
    ControlState.msg
    DRPoseWithTime.msg
    KeyState.msg
    Latcontrol.msg
    lidarmatching.msg
    LLH.msg
    localization.msg
    Loncontrol.msg
    NavStatus.msg
    Position.msg
    Rotation.msg
    Pose.msg
    sensorgps.msg
    sensorimu.msg
    SystemOverall.msg
    SystemState.msg
    Trajectory.msg
    TrajectoryArray.msg
    TrajectoryPoint.msg
    Twist.msg
    Vector3.msg
    VehicleInfo.msg
    VehicleState.msg
    wheelspeed.msg
)

#add_service_files(
#  FILES RecognizeLightState.srv
#)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    jsk_recognition_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    jsk_recognition_msgs
    message_runtime
    sensor_msgs
    std_msgs
)
