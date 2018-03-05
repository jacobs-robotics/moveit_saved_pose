cmake_minimum_required(VERSION 2.8.3)
project(moveit_saved_pose)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib message_generation)

add_service_files(
  FILES
  MoveToSavedPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(SYSTEM)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(moveit_saved_pose src/move_to_saved_pose.cpp)
target_link_libraries(moveit_saved_pose ${catkin_LIBRARIES})
install(TARGETS moveit_saved_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(moveit_saved_pose ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

