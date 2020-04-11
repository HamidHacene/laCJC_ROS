cmake_minimum_required(VERSION 3.0)
project(laneDet)

add_definitions(-DHAVE_CBLAS=1)
find_package(OpenCV 3 REQUIRED)
find_package(xtensor REQUIRED)
find_package(xtl REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})

add_executable(laneDet main.cpp imgUtils.cpp imgUtils.h lane.cpp lane.hpp)

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS /MANIFEST:NO)
endif()

target_link_libraries(laneDet ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} xtensor)

include_directories(${OpenCV_INCLUDE_DIRS})
