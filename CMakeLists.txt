cmake_minimum_required(VERSION 2.8.3)
project(loop_closure_detector)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
add_definitions(-w)

# Dbow3 
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )
include_directories(${DBoW3_LIBS})

set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  roslib
  dynamic_reconfigure
  cv_bridge
  image_transport
  inekf_msgs
  pcl_ros
)

# PCL
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(${OpenCV_LIBS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o 
LIST(APPEND CMAKE_MODULE_PATH /home/bipedlab/g2o/cmake_modules ) 
set( G2O_ROOT /usr/local/include/g2o) 

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Pangolin 
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# FMT
find_package(fmt REQUIRED)
set(FMT_LIBRARIES fmt::fmt)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Eigen3
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
    image_transport
    inekf_msgs
    pcl_ros
)

include_directories(
  include
  external/cereal/include
  /usr/local/include/
  /usr/local/lib
  ${catkin_INCLUDE_DIRS}  
)

add_executable(store_images_node
    src/store_images_node.cpp
    src/store_images.cpp
)

add_dependencies(store_images_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(store_images_node 
  ${catkin_LIBRARIES}
  ${DBoW3_LIBS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${FMT_LIBRARIES}
)

add_executable(closeloop_node
    src/closeloop_node.cpp
    src/closeloop.cpp
)

add_dependencies(closeloop_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(closeloop_node 
  ${catkin_LIBRARIES}
  ${DBoW3_LIBS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${FMT_LIBRARIES}
)