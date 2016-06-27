cmake_minimum_required(VERSION 2.8)
project(colorTracker)
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES colorTracker.cpp)
add_executable(colorTracker ${SOURCE_FILES})
target_link_libraries( colorTracker ${OpenCV_LIBS} )
