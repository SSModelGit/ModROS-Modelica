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

double rec[2] = {0.0, 0.0}; // global variable used to recieve data from callback, and then later transmit over socket

void storeCallback(const modros::TwoSprings::ConstPtr& inVal)
{
    // stores information into callback
    rec[0] = inVal->sVal[0];
    rec[1] = inVal->sVal[1];
    printf("Incoming values: %lf | %lf \n", rec[0], rec[1]);
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
    modros::TwoSprings val;

    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);

    // setup ros publisher and subscriber

    int sockfd, newsockfd, portno; // define socket information - bindings and port number
    socklen_t clilen; // socket structure parameter, storing client socket length

    char buffer[256]; // string buffer used to transmit information over the socket
    double sockBuf[2]; // buffer array for interaction with socket
    double rosBuf[2]; // buffer array for transmission via ROS

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

        errorCheck = read(newsockfd,buffer,255); // writes information stored on incoming queue from client socket to buffer, max strlen of 255
        // stores a value in errorCheck >= 0 if able to read, otherwise <0
        if (errorCheck < 0) {
            error("ERROR reading from socket");
        }
        printf("Outgoing socket message: %s\n",buffer); // used for keeping track of information flow

        // buf = 15 * (2 - atof(buffer));
        splitter(rosBuf, buffer, ","); // splits the incoming message on buffer into tokens, based on a delimiter -> "," -- writes this into outgoing ROS publisher data buffer

        memcpy(sockBuf, rec, sizeof(rec)); // copies the information stored on rec (from subscriber callback) to sockBuf, the outgoing socket's data buffer

        printf("To socket values: %lf | %lf\n", sockBuf[0], sockBuf[1]); // debug statement
        commaconcatanater(buffer, sockBuf); // places sockBuf values into buffer - done separately from rec, so that information doesn't get lost somewhere in the middle
        printf("Ingoing socket message: %s \n\n", buffer); // debug
        errorCheck = write(newsockfd, buffer, strlen(buffer)); // writes information in buffer to outgoing queue between client socket and oneself
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }

        close(newsockfd); // close connection to the client socket

        val.sVal[0] = rosBuf[0]; // writes information from data buffer to outgoing ros msg
        val.sVal[1] = rosBuf[1];
        printf("Outgoing values: %lf | %lf \n\n", val.sVal[0], val.sVal[1]); // debug
        pub.publish(val); // publish
        ros::spinOnce(); // check callbacks - do spinOnce over spin(), so that callbacks (changing of rec value) happens only after the current rec value has been sent out
    }
    close(sockfd); // if ctrl-c, exit ros::ok and shut down
    ros::shutdown();

    return 0;
}