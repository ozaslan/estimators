cmake_minimum_required(VERSION 2.8.3)
project(tunnel_estimator)

# I don't know whether the below code upto the "^"'s make any difference!
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS") 
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
find_program(CCACHE_FOUND ccache)     
if(CCACHE_FOUND)                      
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache) 
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
endif(CCACHE_FOUND)        

find_package(cmake_modules REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

find_package(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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
  cmake_modules
  #uniform_feature_extractor
  #uniform_feature_tracker
  )



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
include_directories(${PCL_INCLUDE_DIRS} 
  include           
  include/tunnel_estimator
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
add_library(pc_to_surfaces src/pc_to_surfaces.cc)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
target_link_libraries(pc_to_surfaces ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Declare a C++ executable
add_executable(test_pc_to_surfaces test/test_pc_to_surfaces.cc)

## Add cmake target dependencies of the executable
## same as for the library above
target_link_libraries(test_pc_to_surfaces ${catkin_LIBRARIES} pc_to_surfaces)

## Specify libraries to link a library or executable target against
# target_link_libraries(tunnel_estimator_node
#   ${catkin_LIBRARIES}
# )
