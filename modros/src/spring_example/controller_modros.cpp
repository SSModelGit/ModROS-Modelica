#include "ros/ros.h"
#include "modros/TwoSprings.h"

ros::Publisher pub;

double springSetPoints[2] = {0.0, 0.0};

void controlCallBack(const modros::TwoSprings::ConstPtr& inVal) {

    // find control values
    modros::TwoSprings outVal;
    outVal.sVal[0] = 5 * (springSetPoints[0] - inVal->sVal[0]);
    outVal.sVal[1] = 5 * (springSetPoints[1] - inVal->sVal[1]);
    
    printf("Incoming Values: %lf | %lf \n", inVal->sVal[0], inVal->sVal[1]);
    printf("Outgoing Values: %lf | %lf \n\n", outVal.sVal[0], outVal.sVal[1]);

    pub.publish(outVal);
}

void joyCallback(const modros::TwoSprings::ConstPtr& modJoy) {
    springSetPoints[0] = modJoy->sVal[0];
    springSetPoints[1] = modJoy->sVal[1];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_modros"); // takes desired input from teleop_tuned, and feedback from socket_modros, does a proportional gain to get new control values
    ros::NodeHandle n;
    
    pub = n.advertise<modros::TwoSprings>("control_values", 1);
    ros::Subscriber relay_sub = n.subscribe("model_values", 1, controlCallBack);
    ros::Subscriber joy_sub = n.subscribe("modros_joy", 1, joyCallback);

    ros::spin();

    return 0;
}