cmake_minimum_required(VERSION 3.0)
project(zebbi)


find_package(OpenCV 3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(zebbi main.cpp imgUtils.cpp imgUtils.h)
target_link_libraries( zebbi ${OpenCV_LIBS} )
include_directories(${OpenCV_INCLUDE_DIRS})

