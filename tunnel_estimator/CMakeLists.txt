cmake_minimum_required(VERSION 2.8.3)
project(tunnel_estimator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(cmake_modules REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_ros
  quadrotor_ukf_lite
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  utils
  #uniform_feature_extractor
  #uniform_feature_tracker
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

find_package(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
# )

catkin_package(
  INCLUDE_DIRS include/tunnel_estimator/
  LIBRARIES
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs pcl_ros quadrotor_ukf_lite roscpp rospy sensor_msgs std_msgs uniform_feature_extractor uniform_feature_tracker utils visualization_msgs
  DEPENDS Eigen OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/tunnel_estimator
)

## Declare a C++ library
add_library(pc_to_surfaces src/pc_to_surfaces.cc)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(pc_to_surfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(tunnel_estimator_node src/tunnel_estimator_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tunnel_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(tunnel_estimator_node
#   ${catkin_LIBRARIES}
# )