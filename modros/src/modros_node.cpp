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

    private:
        //callback for joy messages
        void controllerCallback(const modros::ModComm::ConstPtr& inVal);
        //Arm ardusub
        void splitter(double *outVal, char *s, char *delim);
        //Change rover mode
        void commaconcatanater(char *s, double *dat);
        //Arduino map function to get value between 1000-2000
        void error(const char *msg);

        //ROS node handle, subs, pubs, service clients
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;

        int sockfd;
        int newsockfd; 
        int portno;
        struct sockaddr_in serv_addr; 
        // define socket information - bindings and port number
        
        // define client information
        socklen_t clilen;
        struct sockaddr_in cli_addr;

        char buffer[MAX_BUF]; // string buffer used to transmit information over the socket
        double sockBuf[MAX_ARRAY]; // buffer array for interaction with socket
        double rosBuf[MAX_ARRAY]; // buffer array for transmission via ROS
        double rec[MAX_ARRAY]; // global variable used to recieve data from callback, and then later transmit over socket
        int recSize; // indicates the amount of data coming from controller; initializes to maximum possible
        int update_rate;

        int errorCheck;
};

/*
    Defult constructor initialzes RCmsg, joy stick flags, subs and pubs
        Inputs: None
        Returns: None
*/
Modros::Modros()
{
    // Initialize subscriber and publisher
    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);
    ros::Publisher pub = nh.advertise<modros::ModComm>("model_values", 1);

    rec = {0.0};
    recSize = MAX_ARRAY;


    ros::NodeHandle nh_param("~");
    nh_param.param<int>("portno", portno, 9091);
    nh_param.param<int>("update_rate", update_rate, 20);

    // begin tcp/ip socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR on binding");
    }
}

/*
    Spin function to stop the program from closing and send control/feedback messages at provided parameter speed. 
    Inputs: None
    Returns: None
    
*/
void Modros::spin() 
{
    //set publish rate
    ros::Rate loop(20);

    listen(sockfd,5);
    clilen = sizeof(cli_addr);

  //check if the node is running
    while(ros::ok())
    {
        modros::ModComm outVal;
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen); // createnew handle for client socket
        if (newsockfd < 0) {
            error("ERROR on accept");
        }

        errorCheck = read(newsockfd, buffer, MAX_BUF-1); 
        // read from socket que, store read information to char buffer, max strlen of MAX_BUF - 1
        if (errorCheck < 0) {
            error("ERROR reading from socket");
        }

        splitter(rosBuf, buffer, ","); // parse buffer for values, stored in data buffer (rosBuf)

        memcpy(sockBuf, rec, recSize*sizeof(rec[0])); // copies the information stored on rec (from subscriber callback) to sockBuf

        commaconcatanater(buffer, sockBuf); // places sockBuf values into buffer
        errorCheck = write(newsockfd, buffer, strlen(buffer)); // write char buffer to socket
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }

        close(newsockfd); // close connection to the client socket

        outVal.data.clear();
        outVal.size = rosBuf[0]; // sets the size of the message
        for (int rosI = 1; rosI <= outVal.size; rosI ++) {
            outVal.data.push_back(rosBuf[rosI]);
        }
        pub.publish(outVal);

        // run callbacks once
        ros::spinOnce();
        // enforce a max publish rate
        loop.sleep();
    }
    close(sockfd); // shut down if SIGTERM
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
    recSize = inVal->size; // sets to size of incoming msg from controller
    for(i = 0; i < recSize; i++) {
        rec[i] = inVal->data[i];
    }
}

/*
    Function: Split incoming character buffer from socket into tokens around provided delimiter
    Inputs: array to store data split from buffer; incoming character buffer; delimiter
    Returns: None
*/
void Modros::splitter(double *outVal, char *s, char *delim)
{
    // splits char buffer s into tokens around the char delimiter delim
    // stores tokens as double elements in double array outVal
    int j, i = 0;
    char *token[80];

    token[0] = strtok(s, delim);
    while (token[i] != NULL) {
        i++;
        token[i] = strtok(NULL, delim);
    }

    for (j=0; j<=i-1; j++) {
        outVal[j] = atof(token[j]);
    }
}

/*
    Function: concatenate outgoing socket buffer messages into one string. 
    Inputs: character buffer to add to; data to be concatenated
    Returns: None
*/
void Modros::commaconcatanater(char *s, double *dat)
{
    // concatenates all data from dat into char buffer s around a preset delemiter

    s[0] = '\0';
	char s1[100];
	for (int i = 0; i< sizeof(dat); i++)
	{
		sprintf(s1,"%f,",dat[i]);
		strcat(s,s1);
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
