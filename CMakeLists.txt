cmake_minimum_required(VERSION 3.0.2)
project(lidar_system)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  livox_ros_driver
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
)




catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs livox_ros_driver cv_bridge image_transport
  DEPENDS Eigen3 PCL 
  INCLUDE_DIRS include
)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)




add_executable(recive_data src/recive/recive_data.cpp src/recive/main.cpp)
target_link_libraries(recive_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)


add_executable(transform_data src/transform/transform_data.cpp src/transform/main.cpp)
target_link_libraries(transform_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
)


add_executable(cluster_data src/cluster/cluster_data.cpp src/cluster/main.cpp)
target_link_libraries(cluster_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
)



add_executable(image_ui src/image_ui/image_ui.cpp src/image_ui/main.cpp)
target_link_libraries(image_ui
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
)
