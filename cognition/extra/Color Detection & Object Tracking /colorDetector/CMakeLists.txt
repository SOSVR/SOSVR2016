cmake_minimum_required(VERSION 2.8)
project(colorDetector)
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES colorDetector.cpp)
add_executable(colorDetector ${SOURCE_FILES})
target_link_libraries( colorDetector ${OpenCV_LIBS} )
