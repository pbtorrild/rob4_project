cmake_minimum_required(VERSION 2.8.3)
project(rob4_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  nav_msgs
  OpenCV
  image_transport
  cv_bridge
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ClosestObj.msg
   EmergStop.msg
   ObjectPos.msg
   SignsFound.msg
   LineDist.msg
   RoadObj.msg
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rob4_project
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_runtime
    geometry_msgs
    nav_msgs
    image_transport
    cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a C++ executable

add_executable(control      src/control.cpp)
add_executable(lidar_info   src/lidar_info.cpp)
add_executable(emerg_stop   src/emerg_stop.cpp)
add_executable(test_things  src/test.cpp)
add_executable(follow_line  src/follow_line.cpp)
add_executable(find_signs   src/find_signs.cpp)
add_executable(find_car     src/find_car.cpp)
add_executable(find_human   src/find_human.cpp)

## Add cmake target dependencies of the executable

add_dependencies(control      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_info   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(emerg_stop   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_things  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(follow_line  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(find_signs   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(find_car     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(find_human   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(control      ${catkin_LIBRARIES})
target_link_libraries(lidar_info   ${catkin_LIBRARIES})
target_link_libraries(emerg_stop   ${catkin_LIBRARIES})
target_link_libraries(test_things  ${catkin_LIBRARIES})
target_link_libraries(follow_line  ${catkin_LIBRARIES})
target_link_libraries(find_signs   ${catkin_LIBRARIES})
target_link_libraries(find_car     ${catkin_LIBRARIES})
target_link_libraries(find_human   ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
