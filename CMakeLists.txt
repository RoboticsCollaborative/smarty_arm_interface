cmake_minimum_required(VERSION 3.0.2)
project(smarty_arm_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS system python thread program_options REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY
    msg
    FILES
    PTIPacket.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/position_shift.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES rdda_interface
    CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)

###########
## Build ##
###########

# Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Declare a C++/C library
add_library(${PROJECT_NAME}_src
    src/shm.c
)
target_link_libraries(${PROJECT_NAME}_src ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/smarty_arm_interface.cpp)
target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}_src
    -lrt
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# add_executable(${PROJECT_NAME}_panel src/server.cpp)
# target_link_libraries(${PROJECT_NAME}_panel 
#     ${catkin_LIBRARIES}
# )
# add_dependencies(${PROJECT_NAME}_panel ${PROJECT_NAME}_gencfg)
