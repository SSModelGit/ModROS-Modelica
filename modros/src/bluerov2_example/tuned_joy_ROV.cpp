#include <ros/ros.h>
#include <modros/RovProps.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <iostream>

ros::Publisher pub;

const double inverseControlMat[6][6]= {
    {0.331569, 0.0281442, -0.413594, 0.414877, -0.114091, -0.337866}, 
    {0.296695, 0.0307578, 0.413594, -0.414877, 0.114091, -0.369241}, 
    {0.410411, -0.0307578, 0.293512, 0.414877, -0.114091, 0.369241}, 
    {0.375538, -0.0281442, -0.293512, -0.414877, 0.114091, 0.337866}, 
    {0.08967, 0.49328, -0.10604, 0.315365, -0.586725, 0.0806747}, 
    {-0.08967, 0.50672, 0.10604, -0.315365, 0.586725, -0.0806747}
};

const int invCMatDim = 6;
double joyVal[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    float scale = 14.8;
    
    joyVal[0] = joy->axes[1];
    joyVal[1] = joy->axes[0];
    joyVal[2] = joy->axes[4];
    joyVal[3] = ((-0.5 * joy->axes[2]) + 0.5);
    joyVal[4] = ((-0.5 * joy->axes[5]) + 0.5);
    joyVal[5] = joy->axes[3];
    joyVal[6] = joy->axes[6];
    joyVal[7] = joy->axes[7];
    // joystick mappings - http://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux

    /*
    personal mappings:
    joyVal[0] => Forward movement => axes[1]
    joyVal[1] => Lateral movement => axes[0]
    joyVal[2] => Vertical movement => axes[4]
    joyVal[3] => Roll => axes[2]
    joyVal[4] => Pitch => axes[5]
    joyVal[5] => Yaw => axes[3]
    joyVal[6] => Not set
    joyVal[7] => Not set 
    */

    // set the values of the vertical props
    
    for(int s = 0; s < sizeof(joyVal); s++) {
        joyVal[s] *= scale; // scale up joystick values
    }
}

void rqCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "Servicing callback number " << msg->data.c_str();
    
    modros::RovProps rovVal;
    rovVal.rVal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    printf("\n\n_  _    _                 _  _  _\n");
    // Multiply inverse matrix by 'desired' joystick values
    for(int i = 0; i < invCMatDim; i++) {
        for(int j = 0; j < invCMatDim; j++) {
            rovVal.rVal[i] += inverseControlMat[i][j] * joyVal[j];
        }
        printf("| %lf |    | %lf %lf %lf %lf %lf %lf | | %lf |\n", rovVal.rVal[i], inverseControlMat[i][0], inverseControlMat[i][1], inverseControlMat[i][2], 
        inverseControlMat[i][3], inverseControlMat[i][4], inverseControlMat[i][5], joyVal[i]);
    }
    printf("_  _    _                 _  _  _\n\n");
    
    pub.publish(rovVal); // publish joystick-based control values
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rov_joystick_tuned_control_relay");
    ros::NodeHandle n;
    
    pub = n.advertise<modros::RovProps>("control_values", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    ros::Subscriber rq_sub = n.subscribe<std_msgs::String>("request_channel", 1, rqCallback);

    ros::spin();

    return 0;
}