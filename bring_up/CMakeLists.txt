cmake_minimum_required(VERSION 2.8.3)
project(bring_up)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  std_msgs
  roslaunch
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


roslaunch_add_file_check(launch)


link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    gazebo_plugins
    roscpp 
    gazebo_ros 
)

