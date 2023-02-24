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
        libmavconn
        backward_ros
        manager_msgs
        tf2_ros
        esdf_map
        )

## System dependencies are found with CMake's conventions
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


generate_dynamic_reconfigure_options(
        cfg/swarm_commander.cfg
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
        include
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

#is used to supply a list of libraries (object archives) to the linker. 
link_libraries(
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
  )

## Declare a C++ library  is similar to add_executable, except it adds a target for a library
add_library(${PROJECT_NAME}
        src/SwarmCommander.cpp
        src/line_segment.cpp
        src/path.cpp
)


#target_link_libraries( ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${INCLUDE_FILES}
#        ${PROJECT_NAME} # important to find include libraries
#        ${CERES_LIBRARIES}
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(${PROJECT_NAME}_node src/nodes/launch_node.cpp)

 

## Declare a C++ executable
# Declare a C++ executable
#add_executable(${PROJECT_NAME}
#        src/SwarmCommander.cpp
#        src/line_segment.cpp
#        src/path.cpp
#        # src/cost_functions.hpp
#        )
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
#target_link_libraries(${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${CERES_LIBRARIES}
#        )
