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
  jaco_driver
  jaco_msgs
  moveit_ros_planning_interface
  tf
  tf_conversions
)

find_package( PCL REQUIRED )

include_directories(  ${catkin_INCLUDE_DIRS}
                      ${PCL_INCLUDE_DIRS}
                      include
                #gtest-1.7.0/include
)

#add_subdirectory(gtest-1.7.0)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_library(codeToTest  test/testApi.cpp
#			test/testObjectBd.cpp)
add_library(lperception src/perception/perception.cpp
  				src/perception/objectExtractor.cpp
  				src/perception/jaco_custom.cpp
  				src/perception/communication.cpp
  				src/perception/fileAPI.cpp
  				src/perception/object_recognition.cpp
  				src/perception/objectBd.cpp)
target_link_libraries(lperception ${catkin_LIBRARIES} ${PCL_LIBRARIES})


catkin_package(
  INCLUDE_DIRS include
  #gtest-1.7.0/include
  #${PCL_INCLUDE_DIRS}
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

add_executable(image_transport_node src/image_transport_node.cpp)
target_link_libraries(image_transport_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(image_transport_node perception3d_gencpp)

add_executable(perception
  src/perception/perception.cpp
  src/perception/objectExtractor.cpp
  src/perception/jaco_custom.cpp
  src/perception/communication.cpp
  src/perception/fileAPI.cpp
  src/perception/object_recognition.cpp
  src/perception/objectBd.cpp
)
target_link_libraries(perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(perception perception3d_gencpp)

#add_executable(unit-test   test/testApi.cpp
#				test/testObjectBd.cpp)
#target_link_libraries(unit-test gtest gtest_main rt pthread boost_system lperception)

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

add_executable(testMoveit src/moveitTest/testMoveit.cpp)
target_link_libraries(testMoveit ${catkin_LIBRARIES} ${PCL_LIBRARIES} lperception)
add_dependencies(testMoveit perception3d_gencpp)

add_executable(terminalListener src/moveitTest/terminalListener.cpp)
target_link_libraries(terminalListener ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arm_action_server src/moveitPlugin/arm_action_server.cpp)
target_link_libraries(arm_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable( moveit_jaco_listener src/moveitPlugin/moveit_jaco_listener.cpp)
target_link_libraries(moveit_jaco_listener ${catkin_LIBRARIES} ${PCL_LIBRARIES} lperception)

add_executable( data_publisher_joint_test src/data_publisher_joint_test.cpp)
target_link_libraries(data_publisher_joint_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} lperception)

add_executable(redoPose src/moveitTest/redoPose.cpp)
target_link_libraries(redoPose ${catkin_LIBRARIES} ${PCL_LIBRARIES} lperception )
