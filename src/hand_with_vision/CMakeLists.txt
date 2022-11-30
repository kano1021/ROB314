# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(hand_with_vision)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation)
# Declare our catkin package.

#add_message files
add_message_files(FILES blocks_poses.msg)
add_message_files(FILES Tracker.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(server_shared SHARED blocks_poses_publisher.cpp)
add_dependencies(server_shared ${server_shared_EXPORTTED_TARGETS} ${catkin_EXPORTED_TARGETS} server_shared_generate_messages_cpp)

# Declare the executable, along with its source files.
add_executable(blocks_poses_publisher blocks_poses_publisher.cpp)

add_dependencies(blocks_poses_publisher ${server_shared_EXPORTTED_TARGETS} 
                                ${catkin_EXPORTED_TARGETS}
                                server_shared_generate_messages_cpp
                                server_shared_gencpp)
target_link_libraries(blocks_poses_publisher server_shared ${catkin_LIBRARIES})

add_executable(blocks_spawner blocks_spawner.cpp)
# Specify libraries against which to link.
target_link_libraries(blocks_spawner ${catkin_LIBRARIES})
