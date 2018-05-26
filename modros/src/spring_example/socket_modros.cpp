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

#define MAX_BUF 1024
#define MAX_ARRAY 256

void commaconcatanater(char *s, double *dat)
{
    // concatanates data into a string, separated by a preset splitter -> comma
    s[0] = '\0';
	char s1[100];
	for (int i = 0; i< sizeof(dat); i++)
	{
		sprintf(s1,"%f,",dat[i]);
		strcat(s,s1);
	}
}

void splitter(double *outVal, char *s, char *delim) {
    // http://www.rosipov.com/blog/c-strtok-usage-example/
    // splits a string into tokens
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
int recSize = MAX_ARRAY;

void storeCallback(const modros::TwoSprings::ConstPtr& inVal)
{
    // stores information into callback
    int i;
    recSize = inVal->size;
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

    ros::NodeHandle nh;
    ros::NodeHandle nh_param;
    ros::Publisher pub = nh.advertise<modros::TwoSprings>("model_values", 1);
    modros::TwoSprings outVal;

    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);

    // setup ros publisher and subscriber

    int sockfd, newsockfd, portno; // define socket information - bindings and port number
    socklen_t clilen; // socket structure parameter, storing client socket length

    char buffer[MAX_BUF]; // string buffer used to transmit information over the socket
    double sockBuf[MAX_ARRAY]; // buffer array for interaction with socket
    double rosBuf[MAX_ARRAY]; // buffer array for transmission via ROS

    struct sockaddr_in serv_addr, cli_addr; // socket structure parameters storing the socket addresses

    int errorCheck; // stores the return value from sending information via the socket, used for error checks

    nh_param.param<int>("portno", portno, 9091);

    sockfd = socket(AF_INET, SOCK_STREAM, 0); // creates an INET socket and binds the handle to sockfd
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr)); // zeroes out param

    portno = 9090; // manually storing port number

    serv_addr.sin_family = AF_INET; // defines information of socket
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR on binding"); // checks if the result from binding has worked
    }

    listen(sockfd,5); // holds the socket, and keeps it waiting at this point till it recieves a connection from a client socket.
    // Can hold up to five client socket connections, at any given point.
    clilen = sizeof(cli_addr); // when a client socket arrives, it's information is stored in cli_addr, and clilen stores its 'size'

    while(ros::ok())
    {
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen); // creates a new handle used to point to the client socket - assuming only one client is involved
        if (newsockfd < 0) {
            error("ERROR on accept"); // fail if cannot bind handle to client socket
        }

        errorCheck = read(newsockfd, buffer, MAX_BUF-1); // writes information stored on incoming queue from client socket to buffer, max strlen of MAX_BUF - 1
        // stores a value in errorCheck >= 0 if able to read, otherwise <0
        if (errorCheck < 0) {
            error("ERROR reading from socket");
        }

        // buf = 15 * (2 - atof(buffer));
        splitter(rosBuf, buffer, ","); // splits the incoming message on buffer into tokens, based on a delimiter -> "," -- writes this into outgoing ROS publisher data buffer

        memcpy(sockBuf, rec, recSize*sizeof(rec[0])); // copies the information stored on rec (from subscriber callback) to sockBuf, the outgoing socket's data buffer

        commaconcatanater(buffer, sockBuf); // places sockBuf values into buffer - done separately from rec, so that information doesn't get lost somewhere in the middle
        errorCheck = write(newsockfd, buffer, strlen(buffer)); // writes information in buffer to outgoing queue between client socket and oneself
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }

        close(newsockfd); // close connection to the client socket

        outVal.sVal.clear();
        outVal.size = rosBuf[0]; // sets the size of the message
        for (int rosI = 1; rosI <= outVal.size; rosI ++) {
            outVal.sVal.push_back(rosBuf[rosI]);
        }
        pub.publish(outVal); // publish
        ros::spinOnce(); // check callbacks - do spinOnce over spin(), so that callbacks (changing of rec value) happens only after the current rec value has been sent out
    }
    close(sockfd); // if ctrl-c, exit ros::ok and shut down
    ros::shutdown();

    return 0;
}