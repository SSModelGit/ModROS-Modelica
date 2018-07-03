# ModROS-Modelica

This is a depracated version of the **`modelica_bridge`** and **`ROS_Bridge`** packages, collectively called **`ModROS`** (and later **`ModROS-Modelica`**). To see the current packages, please go to one of the following:

- [**`ROS_Bridge`**](https://github.com/ModROS/ROS_Bridge.git): 
  - This is a Modelica package meant to provide an interface with ROS. It operates using an external C-function, which runs a tcp/ip socket.
- [**`modelica_bridge`**](https://github.com/ModROS/modelica_bridge.git):
  - The complementary package to **`ROS_Bridge`**, **`modelica_bridge`** is a ROS package which acts as an interface to Modelica, using tcp/ip sockets. 
  - Read more about the **`modelica_bridge`** package on the [wiki](wiki.ros.org/modelica_bridge)
- [**`modelica_bridge_examples`**](https://github.com/ModROS/modelica_bridge_examples.git):
  - This contains ROS code (nodes and launch file) to use as an example on how to use the **`modelica_bridge`** package. It is for the *`TwoSpringsJoystickControl.mo`* example model in **`ROS_Bridge`**.