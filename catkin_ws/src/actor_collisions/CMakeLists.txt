cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# Nome del pacchetto
project(actor_collisions)

# Trova i pacchetti necessari
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros)
find_package(gazebo REQUIRED)

# Dichiarare i pacchetti Catkin
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ActorCollisionsPlugin
#  CATKIN_DEPENDS roscpp gazebo_ros
#  DEPENDS gazebo
#)

# Includere le directory di include
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Creare la libreria del plugin
add_library(ActorCollisionsPlugin SHARED src/ActorCollisionsPlugin.cc)

# Collegare la libreria del plugin con le librerie di Gazebo e Catkin
target_link_libraries(ActorCollisionsPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Specificare le dipendenze del plugin
add_dependencies(ActorCollisionsPlugin ${catkin_EXPORTED_TARGETS})