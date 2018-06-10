//ROS
#include <ros/ros.h>

// Modros message
#include <modros/ModComm.h>

// Socket-related code
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <signal.h>
#include <functional>

// For standard messages
#include <stdlib.h>
#include <string.h>

#define MAX_BUF 1024 // define maximum length of character buffer
#define MAX_ARRAY 256 // define maximum size of internal buffer arrays

// Class to act as relay between ROS and Modelica
class Modros
{
    public:
        //Default Constructor
        Modros();
        //Function to keep program from closing
        void spin();
        // SIGINT handler
        void socketSigInt(int sig);

    private:
        // callback for control values
        void controllerCallback(const modros::ModComm::ConstPtr& inVal);
        // split incoming buffer messages
        void splitter();
        // concatanate outgoing buffer
        void commaConcatanater();
        // error message function
        void error(const char *msg);

        //ROS node handle, sub, pub
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        int socket_fd_;
        int new_socket_fd_; 
        int port_num_;
        struct sockaddr_in serv_addr; 
        // define socket information - bindings and port number
        
        // define client information
        socklen_t client_len_;
        struct sockaddr_in client_address_;

        char buffer_[MAX_BUF]; // string buffer used to transmit information over the socket
        double socket_buffer_[MAX_ARRAY]; // buffer array for interaction with socket
        double ros_buffer_[MAX_ARRAY]; // buffer array for transmission via ROS
        int update_rate_;

        int error_check_;
};

/*
    Defult constructor initialzes RCmsg, joy stick flags, subs and pubs
        Inputs: None
        Returns: None
*/
Modros::Modros()
{
    // Initialize subscriber and publisher
    sub_ = nh_.subscribe("control_values", 1, &Modros::controllerCallback, this);
    pub_ = nh_.advertise<modros::ModComm>("model_values", 1);

    ros::NodeHandle nh_param("~");
    nh_param.param<int>("port_num", port_num_, 9091);
    nh_param.param<int>("update_rate", update_rate_, 20);

    // begin tcp/ip socket
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

/*
    Spin function to stop the program from closing, send control/feedback messages. 
    Inputs: None
    Returns: None
    
*/
void Modros::spin() 
{
    //set publish rate
    ros::Rate loop(20);

    // listen(socket_fd_,5);
    // client_len_ = sizeof(client_address_);

  //check if the node is running
    while(ros::ok())
    {
        modros::ModComm outVal;

        listen(socket_fd_,5);
        client_len_ = sizeof(client_address_);

        // create new handle for client socket
        new_socket_fd_ = accept(socket_fd_, (struct sockaddr *) &client_address_, &client_len_); 
        if (new_socket_fd_ < 0) {
            error("ERROR on accept");
        }

        error_check_ = read(new_socket_fd_, buffer_, MAX_BUF-1); 
        // read from socket que, store read information to char buffer, max strlen of MAX_BUF - 1
        if (error_check_ < 0) {
            error("ERROR reading from socket");
        }

        splitter(); // parse buffer for values, stored in data buffer (ros_buffer_)
        commaConcatanater(); // places socket_buffer_ into char buffer

        error_check_ = write(new_socket_fd_, buffer_, strlen(buffer_)); // write char buffer to socket
        if (error_check_ < 0) {
            error("ERROR writing to socket");
        }

        close(new_socket_fd_); // close connection to the client socket

        outVal.data.clear();
        outVal.size = ros_buffer_[0]; // sets the size of the message
        for (int rosI = 1; rosI <= outVal.size; rosI ++) {
            outVal.data.push_back(ros_buffer_[rosI]);
        }
        pub_.publish(outVal);

        // run callbacks once
        ros::spinOnce();
        // enforce a max publish rate
        loop.sleep();
    }
    close(socket_fd_);
    ros::shutdown();
}

/*
    Callback function runs when a modcomm message is recieved. It stores the data
    into internal structure.
    
    Inputs: pointer to modcomm message from ROS
    Returns: None
*/
void Modros::controllerCallback(const modros::ModComm::ConstPtr& inVal)
{
    // stores information coming from the controller, read by 
    int i;
    for(i = 0; i < inVal->size; i++) {
        socket_buffer_[i] = inVal->data[i];
    }
}

/*
    Function: Split incoming character buffer from socket into tokens around provided delimiter
    Inputs: None
    Returns: None
*/
void Modros::splitter()
{
    // splits char buffer s into tokens around the char delimiter delim
    // stores tokens as double elements in double array outVal
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

/*
    Function: concatanate outgoing socket buffer messages into one string. 
    Inputs: character buffer to add to; data to be concatenated
    Returns: None
*/
void Modros::commaConcatanater()
{
    // concatenates all data from dat into char buffer s around a preset delemiter

    buffer_[0] = '\0';
	char s1[100];
	for (int i = 0; i< sizeof(*socket_buffer_); i++)
	{
		sprintf(s1,"%f,",socket_buffer_[i]);
		strcat(buffer_,s1);
	}   
}

/*
    Function: Print error message to screen
    Inputs: Error message
*/
void Modros::error(const char *msg)
{
    // outputs error msg
    perror(msg);
    exit(1);
}

//Main
int main(int argc, char** argv)
{
    //Initialize ros node with "modros_node" as the name of the node
    ros::init(argc, argv, "modros_node");
    
    //Create object of type Modros
    Modros modros;

    //call spin function from Modros class
    modros.spin();
    
    return 0;
}
