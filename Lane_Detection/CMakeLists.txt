cmake_minimum_required(VERSION 3.0)
project(laneDet)


find_package(OpenCV 3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(laneDet main.cpp imgUtils.cpp imgUtils.h lane.cpp lane.hpp)
target_link_libraries(laneDet ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})
