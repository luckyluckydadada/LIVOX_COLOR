cmake_minimum_required(VERSION 2.8)
project(livox_color)

set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
  message_generation 
  std_msgs
  cv_bridge
  image_transport 
)

catkin_package(
 
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}  ${catkin_LIBRARIES}  ${OpenCV_LIBS})
