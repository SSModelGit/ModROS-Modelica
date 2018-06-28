/** @file modros_node.cpp
 * Node to connect Modelica to ROS.
*/

#include <ros/ros.h>
#include <modros/ModComm.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <signal.h>
#include <functional>

#include <stdlib.h>
#include <string.h>

#define MAX_BUF 1024 // define maximum length of character buffer
#define MAX_ARRAY 256 // define maximum size of internal buffer arrays

/**
 * Class to act as relay between ROS and Modelica.
 * It initializes a tcp/ip socket to act as a relay between ROS and Modelica. It 
 * can take character buffers up to 1024 bytes.
 */
class Modros
{
    public:
        /**
         * Defult constructor initialzes RCmsg, joy stick flags, subs and pubs.
         * No inputs.
         * @return none
         */
        Modros();
        /**
         * Keep program from closing.
         * Run callbacks for ROS nodes, and keep socket connection to Modelica alive.
         * No inputs.
         * @return none
         */
        void spin();

    private:
        /**
         * Callback for receiving controller values.
         * @param[in] inVal ModComm message holding controller values.
         * @return none
         */
        void controllerCallback(const modros::ModComm::ConstPtr& inVal);
        
        /**
         * Breaks up the incoming character buffer into data array.
         * Splits the character buffer into an array of tokens of data.
         * The first token is the size of the array.
         * @see buffer_()
         * @see ros_buffer_()
         * @see commaConcatanater()
         * @return none
         */
        void splitter();
        
        /**
         * Concatenates the control values into a character buffer.
         * @see splitter()
         * @see buffer_()
         * @see socket_buffer_()
         * @return none
         */
        void commaConcatanater();

        /**
         * Prints error message when interacting with socket.
         * @param [in] error message
         * @see error_check_()
         * @return none
         */
        void error(const char *msg);

        ros::NodeHandle nh_; ///< ROS node handle
        ros::Subscriber sub_; ///< subscriber to ROS controller values
        ros::Publisher pub_; ///< publisher for model feedback values

        int socket_fd_; ///< ROS socket binding
        int new_socket_fd_; ///< Modelica socket binding
        int port_num_; ///< Port number
        struct sockaddr_in serv_addr; ///< ROS socket address
        socklen_t client_len_; ///< Modelica socket size
        struct sockaddr_in client_address_; ///< Modelica socket address

        char buffer_[MAX_BUF]; ///< string buffer used to transmit information over the socket
        double socket_buffer_[MAX_ARRAY]; ///< buffer array for interaction with socket
        double ros_buffer_[MAX_ARRAY]; ///< buffer array for transmission via ROS
        int update_rate_; ///< ROS node maximum loop rate.

        int error_check_; ///< Parameter for checking errors on socket interaction.
};

Modros::Modros()
{
    sub_ = nh_.subscribe("control_values", 1, &Modros::controllerCallback, this);
    pub_ = nh_.advertise<modros::ModComm>("model_values", 1);

    ros::NodeHandle nh_param("~");
    nh_param.param<int>("port_num", port_num_, 9091);
    nh_param.param<int>("update_rate", update_rate_, 20);

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        error("ERROR opening socket");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port_num_);
    if (bind(socket_fd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR on binding");
    }
}

void Modros::spin() 
{
    ros::Rate loop(20);

    while(ros::ok())
    {
        modros::ModComm outVal;

        listen(socket_fd_,5);
        client_len_ = sizeof(client_address_);

        new_socket_fd_ = accept(socket_fd_, (struct sockaddr *) &client_address_, &client_len_); 
        if (new_socket_fd_ < 0) {
            error("ERROR on accept");
        }

        error_check_ = read(new_socket_fd_, buffer_, MAX_BUF-1); 
        if (error_check_ < 0) {
            error("ERROR reading from socket");
        }

        splitter();
        commaConcatanater();

        error_check_ = write(new_socket_fd_, buffer_, strlen(buffer_));
        if (error_check_ < 0) {
            error("ERROR writing to socket");
        }

        close(new_socket_fd_);

        outVal.data.clear();
        outVal.size = ros_buffer_[0];
        for (int rosI = 1; rosI <= outVal.size; rosI ++) {
            outVal.data.push_back(ros_buffer_[rosI]);
        }
        pub_.publish(outVal);

        ros::spinOnce();
        loop.sleep();
    }
    close(socket_fd_);
}

void Modros::controllerCallback(const modros::ModComm::ConstPtr& inVal)
{
    int i;
    for(i = 0; i < inVal->size; i++) {
        socket_buffer_[i] = inVal->data[i];
    }
}

void Modros::splitter()
{
    int j, i = 0;
    char *token[80];
    const char * delim = ",";

    token[0] = strtok(buffer_, delim);
    while (token[i] != NULL) {
        i++;
        token[i] = strtok(NULL, delim);
    }

    for (j=0; j<=i-1; j++) {
        ros_buffer_[j] = atof(token[j]);
    }
}

void Modros::commaConcatanater()
{
    buffer_[0] = '\0';
	char s1[100];
	for (int i = 0; i< sizeof(*socket_buffer_); i++)
	{
		sprintf(s1,"%f,",socket_buffer_[i]);
		strcat(buffer_,s1);
	}   
}

void Modros::error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "modros_node");

    Modros modros;
    
    modros.spin();
    
    return 0;
}
