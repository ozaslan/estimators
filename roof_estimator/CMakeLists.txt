cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_libraries(${OCTOMAP_LIBRARIES})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(rangeBasedRoofLocalizer  src/range_based_roof_localizer.cc)
rosbuild_add_library(visionBasedRoofLocalizer src/vision_based_roof_localizer.cc)
rosbuild_add_library(roofLocalizer src/roof_localizer.cc)
target_link_libraries(roofLocalizer rangeBasedRoofLocalizer visionBasedRoofLocalizer)

rosbuild_add_library(rangeBasedRoofMapper  src/range_based_roof_mapper.cc)
rosbuild_add_library(visionBasedRoofMapper src/vision_based_roof_mapper.cc)
rosbuild_add_library(roofMapper src/roof_mapper.cc)
target_link_libraries(roofMapper rangeBasedRoofMapper visionBasedRoofMapper)

#target_link_libraries(${PROJECT_NAME}_node rangeBasedTunnelLocalizer)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/roof_estimator_node.cc)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} roofLocalizer roofMapper)
