// example node for running joystick values

#include <ros/ros.h>
#include "modros/TwoSprings.h"
#include <sensor_msgs/Joy.h>

ros::Publisher pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  modros::TwoSprings springVal;
  for(int i = 0; i < 8; i++) {
    springVal.sVal.push_back((i+1)*joy->axes[i]);
  }
  springVal.size = springVal.sVal.size();
  pub.publish(springVal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_tuned");
    ros::NodeHandle n;
    
    pub = n.advertise<modros::TwoSprings>("modros_joy", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();

    return 0;
}