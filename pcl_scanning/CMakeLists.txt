cmake_minimum_required(VERSION 2.8.3)
project(pcl_scanning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp roslib rospack sensor_msgs std_msgs geometry_msgs image_transport tf cv_bridge
  message_filters pcl_ros rospy
)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
#find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_scanning
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
)


#file(GLOB_RECURSE pcl_scanning_SRC "src/*.cpp")

SET(SRC
"src/object_edge_detector.cpp"
"src/depthedge.cpp"
)

SET(LIBS
${Eigen3_LIBS}
${OpenCV_LIBS} 
${catkin_LIBRARIES}
)

add_executable (hand_transform src/hand_transform.cpp)
target_link_libraries (hand_transform ${LIBS})

add_executable (depthedge ${SRC})
target_link_libraries (depthedge ${LIBS})

#add_executable (object_edge_detection src/object_edge_detection.cpp)
#target_link_libraries (object_edge_detection ${PCL_LIBS}${OpenCV_LIBS} ${catkin_LIBRARIES})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
