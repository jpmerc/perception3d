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
  image_transport
)

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include  #${PCL_INCLUDE_DIRS}
 #  LIBRARIES ${PCL_LIBRARY_DIRS}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


#add_executable(segmentation src/segmentation.cpp)
#target_link_libraries(segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(snapshot src/snapshot.cpp)
#target_link_libraries(snapshot ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(snapshot perception3d_gencpp)

#add_executable(objects_extraction src/objects_extraction.cpp)
#target_link_libraries(objects_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(objects_extraction perception3d_gencpp)

#add_executable(connected_segmentation src/connected_segmentation.cpp)
#target_link_libraries(connected_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(connected_segmentation perception3d_gencpp)


#add_executable(find_cloud src/find_cloud.cpp)
#target_link_libraries(find_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(find_cloud perception3d_gencpp)

#add_executable(tracking src/tracking.cpp)
#target_link_libraries(tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(tracking perception3d_gencpp)

#add_executable(image_transport_node src/image_transport_node.cpp)
#target_link_libraries(image_transport_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(image_transport_node perception3d_gencpp)

add_executable(perception
  src/perception/perception.cpp
  src/perception/objectExtractor.cpp
  src/perception/jaco_custom.cpp
  src/perception/communication.cpp
  src/perception/fileAPI.cpp
  src/perception/object_recognition.cpp
)
target_link_libraries(perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(perception perception3d_gencpp)

add_executable(segmentation
  src/segmentation/mainProgram.cpp
  src/segmentation/planSegmentor.cpp
)
target_link_libraries(segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(segmentation perception3d_gencpp)



#add_executable(grasp src/jacoTests/grasp.cpp)
#target_link_libraries(grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(grasp perception3d_gencpp)

#add_executable(jaco_joint_states src/jacoTests/jaco_joint_states.cpp)
#target_link_libraries(jaco_joint_states ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(jaco_joint_states perception3d_gencpp)

add_executable(jaco_calibration src/jacoTests/calibration.cpp)
target_link_libraries(jaco_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(jaco_calibration perception3d_gencpp)

