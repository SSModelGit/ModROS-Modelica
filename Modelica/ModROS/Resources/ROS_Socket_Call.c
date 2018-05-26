// Calls and sends feedback to the ROS socket, and recieves control input for the model

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <assert.h>

#define h_addr h_addr_list[0] /* for backward compatibility */
#define MAX_BUF 1024
#define MAX_ARRAY 256

void commaconcatanater(char *s, double *dat, int datasize)
{
    s[0] = '\0';
	char s1[100];
    sprintf(s1, "%d,", datasize);
    strcat(s, s1); // specifies size of the message in the first element of the array
	for (int i = 0; i< datasize; i++)
	{
		sprintf(s1,"%f,",dat[i]);
		strcat(s,s1);
	}
}

void splitter(double *outVal, char *s, char *delim) {
    // http://www.rosipov.com/blog/c-strtok-usage-example/
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

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void ROS_Socket_Call(double time, int portno, const char *hostname, int querySize, double * query, int resSize, double * res)
{
    printf("time: %f\n", time); // time keeper

    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    
    double buf[MAX_ARRAY];
    memcpy(buf, query, querySize*sizeof(query[0]));
    // double buf[2] = {query[0], query[1]}; // keeps the input values in a double array buffer
    char buffer[MAX_BUF]; // initializes character array buffer for socket transmission

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    // Interact with socket (read and write):
    
    commaconcatanater(buffer, buf, querySize);
    printf("Feedback Values:");
    for(int fi = 0; fi < querySize; fi++) {
        printf(" %lf |", buf[fi]);
    }
    printf("\n");
    n = write(sockfd, buffer, strlen(buffer));
    if (n < 0) 
        error("ERROR writing to socket");

    n = read(sockfd, buffer, 255);
    if (n < 0) 
        error("ERROR reading from socket");
    splitter(buf, buffer, ",");
    memcpy(res, buf, resSize*sizeof(buf[0]));
    printf("Control Values:");
    for(int ci = 0; ci < resSize; ci++) {
        printf(" %lf |", res[ci]);
    };
    printf("\n\n");

    close(sockfd); // close socket
}
