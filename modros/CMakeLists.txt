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
  RovProps.msg
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

### Trials using joystick input to control values in Modelica model

## Example of using ModROS with a basic two spring-damper system
# Communicates as a relay node to Modelica
add_executable(socket_modros src/spring_example/socket_modros.cpp)
target_link_libraries(socket_modros ${catkin_LIBRARIES})
add_dependencies(socket_modros modros_generate_messages_cpp)

# Subscribes to additional node to recieve scaled joystick node values, to use as setpoint for proportional control.
# Uses TwoSprings.msg for passing values to relay node
add_executable(controller_modros src/spring_example/controller_modros.cpp)
target_link_libraries(controller_modros ${catkin_LIBRARIES})
add_dependencies(controller_modros modros_generate_messages_cpp)

# Recieves values from Joy node, scales values, and assigns desired axes values to the values within TwoSprings.msg
add_executable(teleop_tuned src/spring_example/teleop_tuned.cpp)
target_link_libraries(teleop_tuned ${catkin_LIBRARIES})
add_dependencies(teleop_tuned modros_generate_messages_cpp)

## Example of using ModROS with a more involved example of modeling a BlueROV2 system
# Waits to recieve request message from socket to send in control values
# Communicates with controller via RovProps.msg, which contains a float64[6] array
add_executable(socket_ROV src/bluerov2_example/socket_ROV.cpp)
target_link_libraries(socket_ROV ${catkin_LIBRARIES})
add_dependencies(socket_ROV modros_generate_messages_cpp)

# As there is no actual feedback based control happening, there is no need for a separate mod_teleop-like node
# This focuses on passing tuned values through RovProps.msg based on the values coming from the joystick
add_executable(tuned_joy_ROV src/bluerov2_example/tuned_joy_ROV.cpp)
target_link_libraries(tuned_joy_ROV ${catkin_LIBRARIES})
add_dependencies(tuned_joy_ROV modros_generate_messages_cpp)