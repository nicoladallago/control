cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  mav_msgs	
  geometry_msgs
  roscpp
  rospy
  cmake_modules
  ros_kflytelemetry
  dynamic_reconfigure
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/dynamic_parameters.cfg
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES PID_cascade_controller angular_rate_controller lee_position_controller kalman_filter
  CATKIN_DEPENDS mav_msgs roscpp rospy std_msg ros_kflytelemetry geometry_msgs dynamic_reconfigure
  DEPENDS system_lib Eigen
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(PID_cascade_controller 
  src/library/PID_cascade_controller.cpp
)

add_library(angular_rate_controller
  src/library/angular_rate_controller.cpp
)

add_library(lee_position_controller
  src/library/lee_position_controller.cpp
)

add_library(kalman_filter
  src/library/kalman_filter.cpp
)

add_executable(control_node src/control_node.cpp)
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  control_node
  PID_cascade_controller
  angular_rate_controller
  lee_position_controller
  kalman_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
