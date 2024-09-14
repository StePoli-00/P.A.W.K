cmake_minimum_required(VERSION 3.0.2)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(HeadRotationPlugin SHARED HeadRotationPlugin.cc)
target_link_libraries(HeadRotationPlugin ${GAZEBO_LIBRARIES})
