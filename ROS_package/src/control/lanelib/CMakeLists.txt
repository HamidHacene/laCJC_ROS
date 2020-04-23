cmake_minimum_required(VERSION 3.0)
project(lanelib)

add_definitions(-DHAVE_CBLAS=1)
find_package(OpenCV 4 REQUIRED)
find_package(xtensor REQUIRED)
find_package(xtl REQUIRED)
find_package(BLAS REQUIRED)
find_package(Boost REQUIRED system)
find_package(LAPACK REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/lane.cpp
   src/imgUtils.cpp
)

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS /MANIFEST:NO)
endif()

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} xtensor)

include_directories(${OpenCV_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
