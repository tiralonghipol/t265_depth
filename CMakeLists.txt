cmake_minimum_required(VERSION 2.8.3)
project(t265_depth)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    roslib
    image_transport
    cv_bridge
    sensor_msgs
    stereo_msgs
    image_geometry
    pcl_ros
)
find_package(OpenCV 3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs
    DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} 
  src/t265_depth_node.cpp 
  src/t265_depth.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)
