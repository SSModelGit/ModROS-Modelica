# ModROS

ModROS is a general interface between Modelica and ROS, accomplished by utilizing Modelica's external C function capability, and sockets to communicate with ROS. Modelica will act as the client, and ROS the server - using TCP sockets allow the two to be run on two different machines on the same network.

## Getting Started

ModROS has so far been tested with ROS Kinetic - the following instructions will help setup the system with both ROS and ModROS.

### Prerequisites

ModROS requires a \*nix platfom (Linux, etc.) to run, as of now - the external C function in Modelica has not been ported to Windows yet - and further requires a Modelica tool to run. It has currently been tested with:

* SystemModeler 5.0.0 and Ubuntu 16.04 LTS
* SystemModeler 5.0.0 and macOS High Sierra
* OpenModelica 1.12.0 and Ubuntu 16.04 LTS
* OpenModelica 1.12.0 and macOS High Sierra

### Installing

The following instructions work on Ubuntu 16.04, and forks of this version (ex. Ubuntu Mate, Kubuntu, Ubuntu GNOME, etc). Copy the contents of ```ubuntu_setup.sh``` into your favorite editor, and save as ```~/modros.sh```. Go to terminal, and run the command ```sh ~/modros.sh```. This will install both ROS Kinetic and ModROS on your system.

## Running the examples

### TwoSpringsJoystickControl example

This example demonstrates using a joystick to control the desired state of two spring-damper systems: 
* Open your Modelica tool of choice, load the ModROS package, and open the TwoSpringsJoystickControl.mo example, located in the Examples folder. 
  * Make sure that the ```hostName``` parameter in the ```controller``` block contains the IP of the machine running ROS, ex. ```localhost``` or ```192.169.2.1```, etc.
  * Ensure that the ```portNumber``` parameter matches the ```portno``` parameter in the ```two_springs_modros.launch``` file. (located ```~/catkin_ws/src/modros/launch/two_springs_modros.launch```)
* Connect a joystick controller to the computer running ROS. 
* Open your terminal, and enter ```roslaunch modros two_springs_modros.launch``` to start ROS.
* Simulate the ```TwoSpringsJoystickControl.mo``` file. Graph the positions of the two springs, and see it change with movement of the two joysticks.

If the joystick movement is not being logged, run ```rostopic echo /joy``` in a new terminal to see if there are any messages being produced from the joystick. If there are not, then follow the instructions in the ROS Tutorials to configure your joystick. Update the parameter values within the ```two_springs_modros.launch``` file accordingly. Close all open terminals, and go through the steps above.

## Deployment

The ROS file vital in ModROS is ```socket_modros.cpp```. It publishes feedback from the Modelica model to the topic ```model_values```, and takes in control values from ROS on the topic ```control_values```. Currently, the file expects to recieve values of type ```double``` that have been written into a character array buffer sent via the socket - update that as needed. Ensure that the ROS msg type is consistent with what is being used by the controller node.

The files vital to Modelica in ModROS are ```ROS_Socket_Call.mo```, ```ROS_Sampler.mo```, and ```ROS_Socket_Call.c```.
* ```ROS_Socket_Call.mo``` contains the external function declaration. Check that the inputs and outputs match what is desired by the model, and that the external C function pointed to within the ```annotation``` is correct.
* ```ROS_Sampler.mo``` is the sampling block that utilizes the external C function. Check that the function call is correct within the block. Update parameters (port number, hostname, sample rate, sampling initialization time, and number of inputs) accordingly.
* ```ROS_Socket_Call.c``` is the external C function. Check that the arguments being recieved from Modelica match what is declared within the ```ROS_Socket_Call.mo```. Currently, it expects to send two arguments of type ```double``` through a character array buffer - change that as needed.

## Authors

* **Shashank Swaminathan** - [SSModelGit](https://github.com/SSModelGit)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
