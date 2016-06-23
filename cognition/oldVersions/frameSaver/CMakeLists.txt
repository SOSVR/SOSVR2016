cmake_minimum_required(VERSION 2.8)
project(frameSaver)

find_package(catkin REQUIRED cv_bridge image_transport sensor_msgs message_generation std_msgs)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package()

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

