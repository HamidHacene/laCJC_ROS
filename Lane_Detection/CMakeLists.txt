cmake_minimum_required(VERSION 3.0)
project(laneDet)

set(CMAKE_CXX_STANDARD 14)
set(XTENSOR_USE_XSIMD 1)
set(XTENSOR_USE_TBB 1)

find_package(OpenCV 3 REQUIRED)
find_package(xtensor REQUIRED)
find_package(xtl REQUIRED)

add_executable(laneDet main.cpp imgUtils.cpp imgUtils.h lane.cpp lane.hpp)

target_link_libraries(laneDet ${OpenCV_LIBS})
target_link_libraries(laneDet xtensor)

include_directories(${OpenCV_INCLUDE_DIRS})
