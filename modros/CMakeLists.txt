cmake_minimum_required(VERSION 2.8.3)
project(modros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TwoSprings.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES modros
  CATKIN_DEPENDS joy roscpp rospy std_msgs
  # DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

### Trials using joystick input to control values in Modelica model

# Main difference from previous iterations is custom message used is TwoSprings.msg, which uses an array of float64[2] for passing values
add_executable(socket_modros src/socket_modros.cpp)
target_link_libraries(socket_modros ${catkin_LIBRARIES})
add_dependencies(socket_modros modros_generate_messages_cpp)

# Subscribes to additional node to recieve scaled joystick node values, to use as setpoint for proportional control.
# Uses TwoSprings.msg for passing values to relay node
add_executable(controller_modros src/controller_modros.cpp)
target_link_libraries(controller_modros ${catkin_LIBRARIES})
add_dependencies(controller_modros modros_generate_messages_cpp)

# Recieves values from Joy node, scales values, and assigns desired axes values to the values within TwoSprings.msg
add_executable(teleop_tuned src/teleop_tuned.cpp)
target_link_libraries(teleop_tuned ${catkin_LIBRARIES})
add_dependencies(teleop_tuned modros_generate_messages_cpp)