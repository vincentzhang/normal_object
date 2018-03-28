cmake_minimum_required(VERSION 2.8.3)
project(normal_object)
#SET(PCL_DIR "/opt/ros/groovy/share/pcl-1.6/")
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs message_generation rgb_visualization)

#######################################
## Declare ROS messages and services ##
#######################################

#add_service_files(
#    FILES
#)

add_message_files(
    FILES
        targetObjects.msg
)

generate_messages(
    DEPENDENCIES
        sensor_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
        message_runtime
        pcl
        pcl_ros
        rgb_visualization
)

###########
## Build ##
###########
include_directories ("${PROJECT_SOURCE_DIR}/include")
add_subdirectory ("${PROJECT_SOURCE_DIR}/lib")

include_directories( $(catkin_INCLUDE_DIRS) )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(normal_object src/normal_object.cpp)
target_link_libraries(normal_object ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(normal_object normal_object_gencpp)

target_link_libraries (normal_object helper_lib)
