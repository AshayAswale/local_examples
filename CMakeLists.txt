cmake_minimum_required(VERSION 2.8.3)
project(local_examples)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tough_controller_interface
  tough_common
  tough_footstep
  tough_moveit_planners
  navigation_common
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_examples
  CATKIN_DEPENDS 	roscpp
                        rospy
                        std_msgs
                        tough_controller_interface
                        tough_common
                        tough_footstep
                        tough_moveit_planners
                        navigation_common
                        geometry_msgs
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS} include)

add_library(local_examples
    src/manipulation_planner_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
drill_grasp
walk_2d_pose_goal_example
manipulation_planner_node
manipulation_node
trial_manipulation_node
gaze_controller
get_frames
test_com
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

######################
## Control examples ##
######################

## Test arm navigation
add_executable(drill_grasp src/drill_grasp.cpp)
add_dependencies(drill_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drill_grasp  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(walk_2d_pose_goal_example src/walk_2d_pose_goal_example.cpp)
add_dependencies(walk_2d_pose_goal_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(walk_2d_pose_goal_example  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(manipulation_planner_node src/manipulation_planner_node.cpp)
add_dependencies(manipulation_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulation_planner_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(manipulation_node src/manipulation_node.cpp)
add_dependencies(manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulation_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(trial_manipulation_node src/trial_manipulation_node.cpp)
add_dependencies(trial_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trial_manipulation_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(gaze_controller src/gaze_controller.cpp)
add_dependencies(gaze_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gaze_controller  ${catkin_LIBRARIES} ${PROJECT_NAME})

## Test arm navigation
add_executable(get_frames src/get_frames.cpp)
add_dependencies(get_frames ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_frames  ${catkin_LIBRARIES} ${PROJECT_NAME})

## CSV trial 
add_executable(test_com src/test_com.cpp)
add_dependencies(test_com ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_com  ${catkin_LIBRARIES})
