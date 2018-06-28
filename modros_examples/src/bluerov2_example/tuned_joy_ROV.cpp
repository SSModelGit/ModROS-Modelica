#include <ros/ros.h>
#include <modros/ModComm.h>
#include <sensor_msgs/Joy.h>
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

void controlCallBack(const modros::ModComm::ConstPtr& inVal)
{
    modros::ModComm rovVal;
    double rVal[6] = {0.0};
    // Multiply inverse matrix by 'desired' joystick values
    for(int i = 0; i < invCMatDim; i++) {
        for(int j = 0; j < invCMatDim; j++) {
            rVal[i] += inverseControlMat[i][j] * joyVal[j];
        }
        rovVal.data.push_back(rVal[i]);
    }
    rovVal.size = rovVal.data.size();
    
    pub.publish(rovVal); // publish joystick-based control values
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rov_joystick_tuned_control_relay");
    ros::NodeHandle n;
    
    pub = n.advertise<modros::ModComm>("control_values", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    ros::Subscriber relay_sub = n.subscribe<modros::ModComm>("model_values", 1, controlCallBack);

    ros::spin();

    return 0;
}