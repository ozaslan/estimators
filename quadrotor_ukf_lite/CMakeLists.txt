cmake_minimum_required(VERSION 2.8.12)

project(quadrotor_ukf_lite)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              mavros 
              sensor_msgs
              nav_msgs
	      tf
              pose_utils
              cmake_modules)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
              roscpp 
              mavros 
              sensor_msgs
              nav_msgs
	      tf
              pose_utils)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS pose_utils 
   DEPENDS armadillo)

include_directories(${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIRS} include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(quadrotor_ukf src/quadrotor_ukf_node.cc
                                      src/quadrotor_ukf.cc)
target_link_libraries(quadrotor_ukf pose_utils ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})
