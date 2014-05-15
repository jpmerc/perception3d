cmake_minimum_required(VERSION 2.8.3)
project(perception3d)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_msgs
)

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  #INCLUDE_DIRS  ${PCL_INCLUDE_DIRS}
 #  LIBRARIES ${PCL_LIBRARY_DIRS}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


add_executable(segmentation src/segmentation.cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(snapshot src/snapshot.cpp)
target_link_libraries(snapshot ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(snapshot perception3d_gencpp)

add_executable(objects_extraction src/objects_extraction.cpp)
target_link_libraries(objects_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(objects_extraction perception3d_gencpp)



