#include <ros/ros.h>
#include <modros/TwoSprings.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

#define MAX_BUF 1024 // define maximum length of character buffer
#define MAX_ARRAY 256 // define maximum size of internal buffer arrays

void commaconcatanater(char *s, double *dat)
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

void splitter(double *outVal, char *s, char *delim) 
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

double rec[MAX_ARRAY] = {0.0}; // global variable used to recieve data from callback, and then later transmit over socket
int recSize = MAX_ARRAY; // indicates the amount of data coming from controller; initializes to maximum possible

void storeCallback(const modros::TwoSprings::ConstPtr& inVal)
{
    // stores information coming from the controller, read by 
    int i;
    recSize = inVal->size; // sets to size of incoming msg from controller
    for(i = 0; i < recSize; i++) {
        rec[i] = inVal->sVal[i];
    }
}

void error(const char *msg)
{
    // outputs error msg
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "socket_modros"); // connection between Modelica and ROS
    
    // setup ros publisher and subscriber
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    ros::Publisher pub = nh.advertise<modros::TwoSprings>("model_values", 1);
    modros::TwoSprings outVal;

    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);

    int sockfd, newsockfd, portno; // define socket information - bindings and port number
    socklen_t clilen;

    char buffer[MAX_BUF]; // string buffer used to transmit information over the socket
    double sockBuf[MAX_ARRAY]; // buffer array for interaction with socket
    double rosBuf[MAX_ARRAY]; // buffer array for transmission via ROS

    struct sockaddr_in serv_addr, cli_addr;

    int errorCheck;

    nh_param.param<int>("portno", portno, 9091);

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

    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    // Interacting with socket (read and write):
    while(ros::ok())
    {
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen); // createnew handle for client socket
        if (newsockfd < 0) {
            error("ERROR on accept");
        }

        errorCheck = read(newsockfd, buffer, MAX_BUF-1); // read from socket que, store read information to char buffer, max strlen of MAX_BUF - 1
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

        outVal.sVal.clear();
        outVal.size = rosBuf[0]; // sets the size of the message
        for (int rosI = 1; rosI <= outVal.size; rosI ++) {
            outVal.sVal.push_back(rosBuf[rosI]);
        }
        pub.publish(outVal);
        ros::spinOnce();
    }
    close(sockfd); // shut down if SIGTERM
    ros::shutdown();

    return 0;
}