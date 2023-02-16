cmake_minimum_required(VERSION 3.0.2)
project(mav_swarm_commander)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        actionlib
        actionlib_msgs
        dynamic_reconfigure
        mavros_msgs
        voxblox_ros
        libmavconn
        backward_ros
        )

## System dependencies are found with CMake's conventions
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


generate_dynamic_reconfigure_options(
        cfg/mav_commander.cfg
        cfg/offboard_ctl.cfg
)

catkin_package(
        INCLUDE_DIRS include
        # LIBRARIES ${PROJECT_NAME}
        #  CATKIN_DEPENDS actionlib roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/flight_commander.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/nodes/launch_node.cpp)

 


