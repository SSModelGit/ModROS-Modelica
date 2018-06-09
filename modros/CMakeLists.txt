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
  ModComm.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES modros
  CATKIN_DEPENDS joy roscpp rospy std_msgs message_runtime
  # DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

### ModROS's main node - the relay connection between Modelica and ROS
add_executable(socket_modros src/socket_modros.cpp)
target_link_libraries(socket_modros ${catkin_LIBRARIES})
add_dependencies(socket_modros modros_generate_messages_cpp)

add_executable(modros_node src/modros_node.cpp)
target_link_libraries(modros_node ${catkin_LIBRARIES})
add_dependencies(modros_node modros_generate_messages_cpp)

### Trials using joystick input to control values in Modelica model

## Example of using ModROS with a basic two spring-damper system
# Subscribes to additional node to recieve scaled joystick node values, to use as setpoint for proportional control.
# Uses ModComm.msg for passing values to relay node
add_executable(controller_spring src/spring_example/controller_spring.cpp)
target_link_libraries(controller_spring ${catkin_LIBRARIES})
add_dependencies(controller_spring modros_generate_messages_cpp)

# Recieves values from Joy node, scales values, and assigns desired axes values to the values within ModComm.msg
add_executable(teleop_spring src/spring_example/teleop_spring.cpp)
target_link_libraries(teleop_spring ${catkin_LIBRARIES})
add_dependencies(teleop_spring modros_generate_messages_cpp)

## Example of using ModROS with a more involved example of modeling a BlueROV2 system
# This focuses on passing tuned values through ModComm.msg based on the values coming from the joystick
add_executable(tuned_joy_ROV src/bluerov2_example/tuned_joy_ROV.cpp)
target_link_libraries(tuned_joy_ROV ${catkin_LIBRARIES})
add_dependencies(tuned_joy_ROV modros_generate_messages_cpp)