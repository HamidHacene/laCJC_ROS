cmake_minimum_required(VERSION 3.0)
project(laneDet)

find_package(OpenCV 3 REQUIRED)
find_package(xtensor REQUIRED)
find_package(xtl REQUIRED)

add_executable(laneDet main.cpp imgUtils.cpp imgUtils.h lane.cpp lane.hpp)

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS /MANIFEST:NO)
endif()

target_link_libraries(laneDet ${OpenCV_LIBS} xtensor)

include_directories(${OpenCV_INCLUDE_DIRS})
