cmake_minimum_required(VERSION 3.0.2)
project(s1000_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dji_sdk
  gbeam_library
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES s1000-interface
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros turtlesim
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node src/s1000-interface_node.cpp)

add_executable(tf2_publisher src/tf2_publisher.cpp)
add_executable(bag_recorder src/bag_recorder.cpp)
add_executable(lidar_merger src/lidar_merger.cpp)
add_executable(auto_flight src/auto_flight.cpp)
add_executable(gbeam_flight_manager src/gbeam_flight_manager.cpp)
add_executable(mpc_flight_manager src/mpc_flight_manager.cpp)
add_executable(rc_position_controller src/rc_position_controller.cpp)
add_executable(position_controller src/position_controller.cpp)
add_executable(position_controller_mpc src/position_controller_mpc.cpp)
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)


target_link_libraries(tf2_publisher
	${catkin_LIBRARIES}
	)
target_link_libraries(bag_recorder
	${catkin_LIBRARIES}
	)
target_link_libraries(lidar_merger
	${catkin_LIBRARIES}
	)
target_link_libraries(auto_flight
	${catkin_LIBRARIES}
	)
target_link_libraries(gbeam_flight_manager
	${catkin_LIBRARIES}
	)
target_link_libraries(mpc_flight_manager
	${catkin_LIBRARIES}
	)
target_link_libraries(rc_position_controller
	${catkin_LIBRARIES}
	)
target_link_libraries(position_controller
	${catkin_LIBRARIES}
	)
target_link_libraries(position_controller_mpc
	${catkin_LIBRARIES}
	)
target_link_libraries(obstacle_avoidance
	${catkin_LIBRARIES}
	)

#############
## Install ##
#############

install(FILES

	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
