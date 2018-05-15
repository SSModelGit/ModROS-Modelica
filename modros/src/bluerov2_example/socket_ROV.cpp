#include <ros/ros.h>
#include <modros/RovProps.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

ros::Publisher pub;

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

double rec[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // global variable used to recieve data from callback, and then later transmit over socket

void storeCallback(const modros::RovProps::ConstPtr& inVal)
{
    // stores information into callback
    rec[0] = inVal->rVal[0];
    rec[1] = inVal->rVal[1];
    rec[2] = inVal->rVal[2];
    rec[3] = inVal->rVal[3];
    rec[4] = inVal->rVal[4];
    rec[5] = inVal->rVal[5];
    printf("In: %lf | %lf | %lf | %lf | %lf | %lf \n", rec[0], rec[1], rec[2], rec[3], rec[4], rec[5]);
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inet_socket_ROV_relay_joy");

    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::String>("request_channel", 1);
    std_msgs::String msg;
    int count = 0;

    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);

    int sockfd, newsockfd, portno;
    socklen_t clilen;

    char buffer[256];
    double sockBuf[6]; // buffer array for interaction with socket
    int counter = 1; // keeps track of request calls

    struct sockaddr_in serv_addr, cli_addr;

    int errorCheck;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));

    portno = 9090;

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR on binding");
    }

    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    while(ros::ok())
    {
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) {
            error("ERROR on accept");
        }

        errorCheck = read(newsockfd,buffer,255);
        if (errorCheck < 0) {
            error("ERROR reading from socket");
        }
        printf("Fielding: %s %d\n",buffer, counter);
        counter++;

        memcpy(sockBuf, rec, sizeof(rec));
        printf("Out: %lf | %lf | %lf | %lf | %lf | %lf |\n", sockBuf[0], sockBuf[1], sockBuf[2], sockBuf[3], sockBuf[4], sockBuf[5]);
        commaconcatanater(buffer, sockBuf);

        printf("Ingoing socket message: %s \n\n", buffer);
        errorCheck = write(newsockfd, buffer, strlen(buffer));
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }

        close(newsockfd);

        count++;
        std::stringstream ss;
        ss << count;
        msg.data = ss.str();
        pub.publish(msg);
        
        ros::spinOnce();
    }
    close(sockfd);
    ros::shutdown();

    return 0;
}