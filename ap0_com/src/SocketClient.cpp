#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <ap0_com/SocketClient.h>
#include <iostream>
#include <sstream>
//Create a Socket for server communication

SocketClient::SocketClient()
{
  hSocket = SocketCreate();
  if(hSocket == -1) // if nt while
  {
      printf("Could not create socket\n");
  }
  printf("Socket is created\n");
  //Connect to remote server
  if (SocketConnect(90190) < 0)
  {
      perror("connect failed.\n");
  }
  printf("Sucessfully conected with server\n");
}

short SocketClient::SocketCreate()
{
    short hSocket;
    printf("Create the socket\n");
    hSocket = socket(AF_INET, SOCK_STREAM, 0);
    return hSocket;
}
//try to connect with server
int SocketClient::SocketConnect(int port_)
{
    int iRetval=-1;
    int ServerPort = port_;
    struct sockaddr_in remote= {0};
    remote.sin_addr.s_addr = inet_addr("127.0.0.1"); //Local Host 192.168.1.4 127.0.0.1
    remote.sin_family = AF_INET;
    remote.sin_port = htons(ServerPort);
    iRetval = connect(hSocket,(struct sockaddr *)&remote,sizeof(struct sockaddr_in));
    return iRetval;
}


// Send the data to the server and set the timeout of 20 seconds
int SocketClient::SocketSend(char* Rqst,short lenRqst)
{
    int shortRetval = -1;
    struct timeval tv;
    tv.tv_sec = 1200;  /* 20 Secs Timeout */
    tv.tv_usec = 0;
    if(setsockopt(hSocket,SOL_SOCKET,SO_SNDTIMEO,(char *)&tv,sizeof(tv)) < 0)
    {
        printf("Time Out\n");
        return -1;
    }
    std::cout<<Rqst<<std::endl;

    shortRetval = send(hSocket, Rqst, lenRqst, 0);
    return shortRetval;
}
//receive the data from the server
int SocketClient::SocketReceive(char* Rsp,short RvcSize)
{
    int shortRetval = -1;
    struct timeval tv;
    tv.tv_sec = 1200;  /* 20 Secs Timeout */
    tv.tv_usec = 0;
    if(setsockopt(hSocket, SOL_SOCKET, SO_RCVTIMEO,(char *)&tv,sizeof(tv)) < 0)
    {
        printf("Time Out\n");
        return -1;
    }
    shortRetval = recv(hSocket, Rsp, RvcSize, 0);
    printf("Response %s\n",Rsp);
    return shortRetval;
}

int SocketClient::listenToGui()
{
    int read_size;
    struct sockaddr_in server;
    char server_reply[200] = {0};

    read_size = SocketReceive(server_reply, 200);
    int id = atoi( server_reply );
    return id;
}

void SocketClient::informGoalReached(int id, int battery)
{
    int read_size;
    struct sockaddr_in server;

    // char SendToServer[4] = {0,0,0,0};
    //
    // SendToServer[0] = id;
    // SendToServer[1] = 'K';

    std::ostringstream oss;
    oss.clear();
    oss <<id<<battery;
    std::string s = oss.str();
    char* pString = new char[s.length() + 1];
    std::copy(s.c_str(), s.c_str() + s.length() + 1, pString);
    // std::cout<<pString<<std::endl;

    //Send data to the server
    SocketSend( pString, strlen(pString));

}
