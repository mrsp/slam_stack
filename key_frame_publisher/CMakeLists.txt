cmake_minimum_required(VERSION 2.8.3)
project(key_frame_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    roscpp
    sensor_msgs
    message_generation
)

add_message_files(FILES
    boolStamped.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package(
#     INCLUDE_DIRS include
    CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters nav_msgs roscpp sensor_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#   include
  ${catkin_INCLUDE_DIRS}  
)

## Declare a C++ executable
add_executable(key_frame_publisher_node src/key_frame_publisher.cpp)

target_link_libraries(key_frame_publisher_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)