cmake_minimum_required(VERSION 2.8.12)

project(velodyne_odom)

# Get the information about this package's buildtime dependencies
find_package(PCL 1.7.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              mavros 
              geometry_msgs 
              sensor_msgs
              nav_msgs
              pcl_ros
              cv_bridge
              utils 
              cmake_modules)

# Declare the message files to be built
#  add_message_files(FILES
#      MyMessage1.msg
#      MyMessage2.msg
#      )

# Actually generate the language-specific message and service files
# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
              mavros 
              sensor_msgs 
              nav_msgs 
              pcl_ros
              cv_bridge 
	      utils
              geometry_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES velodyne_odom
   CATKIN_DEPENDS utils
   DEPENDS eigen)


include_directories(${PCL_INCLUDE_DIRS} 
                    include
                    ${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# I don't know whether the below code upto the "^"'s make any difference!
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS") 
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
find_program(CCACHE_FOUND ccache) 
if(CCACHE_FOUND) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
endif(CCACHE_FOUND) 
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
add_library(velodyne_odom src/velodyne_odom.cc)
# add_library(utils src/utils.cc src/trans_utils.cc)
target_link_libraries(velodyne_odom ${PCL_LIBRARIES} utils)
add_executable(velodyne_odom_node src/velodyne_odom_node.cc)
# target_link_libraries(velodyne_odom_node velodyne_odom utils ${catkin_LIBRARIES}) 
target_link_libraries(velodyne_odom_node velodyne_odom ${catkin_LIBRARIES}) 
