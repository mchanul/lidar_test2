cmake_minimum_required(VERSION 3.0.2)
project(lidar_test2)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_test2 src/lidar_test2.cpp)

target_link_libraries(lidar_test2 ${catkin_LIBRARIES})

