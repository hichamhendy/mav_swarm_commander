cmake_minimum_required(VERSION 3.0.2)
project(mav_swarm_commander)

## Compile as C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
# add_compile_options(-std=c++11)


option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" ON) 


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
        mav_commander
        std_msgs
        libmavconn
        nlopt
        )

## System dependencies are found with CMake's conventions
find_package(Ceres REQUIRED)
# find_package(ifopt)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


generate_dynamic_reconfigure_options(
        cfg/swarm_commander.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        # CATKIN_DEPENDS mav_commander # actionlib roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        SYSTEM
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
        src/SwarmCommanderUtility.cpp
)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/nodes/swarm_commander_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME} 
        ipopt python2.7
        # ifopt::ifopt_ipopt
        )
add_dependencies(${PROJECT_NAME}_node
        ${catkin_EXPORTED_TARGETS}
        )



if(BUILD_EXAMPLE)
    ADD_EXECUTABLE( CppAD_started example/CppAD_started.cpp )
    TARGET_LINK_LIBRARIES(CppAD_started)
    
    ADD_EXECUTABLE( CppAD_Ipopt example/CppAD_Ipopt.cpp )
    TARGET_LINK_LIBRARIES(CppAD_Ipopt ipopt)
endif(BUILD_EXAMPLE)