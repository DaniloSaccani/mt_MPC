cmake_minimum_required(VERSION 3.0.2)
project(mpc_s1000_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
	message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 PolytopeMatrices.msg
 PolytopeMatricesStamped.msg
)

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
generate_messages(
 DEPENDENCIES
 sensor_msgs
 nav_msgs
 geometry_msgs
 std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
INCLUDE_DIRS include
LIBRARIES polytope_functions
CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(polytope_functions
src/polytope_functions.cpp
)


add_dependencies(polytope_functions
${catkin_EXPORTED_TARGETS}
mpc_s1000_controller_generate_messages_cpp
)

target_link_libraries(polytope_functions
${catkin_LIBRARIES}
)

add_executable(polytope_matrices_generation src/polytope_matrices_generation.cpp)
add_executable(polytope_matrices_viewer src/polytope_matrices_viewer.cpp)
add_executable(mtMPC_rc_target_generator src/mtMPC_rc_target_generator.cpp)

add_dependencies(polytope_matrices_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(polytope_matrices_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mtMPC_rc_target_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(polytope_matrices_generation
  ${catkin_LIBRARIES} polytope_functions
)
target_link_libraries(polytope_matrices_viewer
  ${catkin_LIBRARIES} polytope_functions
)
target_link_libraries(mtMPC_rc_target_generator
  ${catkin_LIBRARIES} polytope_functions
)


# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS polytope_functions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/mpc_s1000_controller.launch
	launch/mpc_s1000_parameters.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gbeam_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
