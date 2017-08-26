cmake_minimum_required(VERSION 2.8.3)
project(open_in_hand_scanning)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  gflags
  glog_catkin
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(CUDA REQUIRED) # Uncomment this line if you have trouble compiling
find_package(OpenCV REQUIRED)

set(CAFFE_DIR /home/pr2/openpose/3rdparty/caffe)
set(OPENPOSE_DIR /home/pr2/openpose)

set(CAFFE_INCLUDE_DIRS ${CAFFE_DIR}/distribute/include)
set(CAFFE_LINK_LIBRARY ${CAFFE_DIR}/distribute/lib/libcaffe.so)

set(OPENPOSE_INCLUDE_DIRS ${OPENPOSE_DIR}/include)
set(OPENPOSE_LINK_LIBRARY ${OPENPOSE_DIR}/build/lib/libopenpose.so)


add_message_files(
  FILES
  keypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openpose_ros
   CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories( include )
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${CAFFE_INCLUDE_DIRS}
  ${OPENPOSE_INCLUDE_DIRS}
)

SET(SRC
"src/hand_keypoint_detector.cpp"
"src/cam_motion_tracker.cpp"
)


add_executable(hand_keypoint ${SRC})
add_dependencies(hand_keypoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_keypoint
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CAFFE_LINK_LIBRARY}
  ${OPENPOSE_LINK_LIBRARY}
  ${Eigen3_LIBS}
)

