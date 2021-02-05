cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam)

###########
## Build ##
###########

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

#############
## Package ##
#############

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)
find_package (Eigen3 3.3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.2 REQUIRED)

############
## Catkin ##
############

catkin_package(
  DEPENDS PCL 

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# include directories
include_directories(${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add libraries
add_library (Scan src/scan.cpp src/scan.h)
add_library (Feature src/feature.cpp src/feature.h)
add_library (ScanProvider src/scanprovider.cpp src/scanprovider.h)
add_library (Display src/display.cpp src/display.h)
add_library (Matching src/matching.cpp src/matching.h)
add_library (TransformationSolver src/transformationsolver.cpp src/transformationsolver.h)
add_library (Plot src/plot.cpp src/plot.h)
add_library (RANSAC src/ransac.cpp src/ransac.h)
add_library (FeatureDetector src/featuredetector.cpp src/featuredetector.h)
add_executable (main src/main.cpp)

# target the libraries
target_link_libraries (main ${PCL_LIBRARIES} ${CERES_LIBRARIES} Eigen3::Eigen Scan Feature ScanProvider Display Matching TransformationSolver Plot RANSAC FeatureDetector ${catkin_LIBRARIES})
target_link_libraries (Scan ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries (Feature ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries (ScanProvider ${PCL_LIBRARIES} Eigen3::Eigen Scan)
target_link_libraries (Display ${PCL_LIBRARIES} Eigen3::Eigen Scan Feature)
target_link_libraries (Matching ${PCL_LIBRARIES} Eigen3::Eigen Scan Feature)
target_link_libraries (TransformationSolver ${PCL_LIBRARIES} Eigen3::Eigen ${CERES_LIBRARIES} Scan Feature)
target_link_libraries (Plot Eigen3::Eigen pangolin)
target_link_libraries (RANSAC Eigen3::Eigen pangolin TransformationSolver Scan Feature)
target_link_libraries (FeatureDetector ${PCL_LIBRARIES} Eigen3::Eigen Scan Feature)

