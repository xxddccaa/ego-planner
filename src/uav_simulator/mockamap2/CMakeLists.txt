cmake_minimum_required(VERSION 2.8.3)
project(mockamap2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_executable(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_SRCS})

## Add cmake target dependencies of the executable
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 ) 