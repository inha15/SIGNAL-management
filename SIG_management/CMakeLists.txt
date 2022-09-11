cmake_minimum_required(VERSION 3.0.2)
project(SIG_management)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  std_msgs
  message_generation
  rospy
  cv_bridge
  image_transport
)

add_message_files(
  FILES
  object_msg.msg
  object_msg_arr.msg
  lidar_signal.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs rospy cv_bridge image_transport message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(SIG_proc src/SIGNAL_processing.cpp)
target_link_libraries(SIG_proc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})


add_executable(SIGNAL_manage src/SIG_manage.cpp)
target_link_libraries(SIGNAL_manage SIG_proc)