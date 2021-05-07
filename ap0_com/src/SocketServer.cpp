#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <ap0_com/SocketServer.h>


SocketServer::SocketServer()
{
  printf("Constructor\n"); //TODO
  keepOnListening = true;
  SocketCreate();
  if (hSocket == -1)
  {
      printf("Could not create socket");
  }
  printf("Socket created\n");

  //Bind
  if( BindCreatedSocket() < 0)
  {
      //print the error message
      perror("bind failed.");
  }
  printf("bind done\n");

}

void SocketServer::SocketCreate(void)
{
    printf("Create the socket\n");
    hSocket = socket(AF_INET, SOCK_STREAM, 0);
}

int SocketServer::BindCreatedSocket()
{
    int iRetval=-1;
    int ClientPort = 90190;
    struct sockaddr_in  remote= {0};
    /* Internet address family */
    remote.sin_family = AF_INET;
    /* Any incoming interface */
    remote.sin_addr.s_addr = htonl(INADDR_ANY);
    remote.sin_port = htons(ClientPort); /* Local port */
    iRetval = bind(hSocket,(struct sockaddr *)&remote,sizeof(remote));
    return iRetval;
}

void SocketServer::AlwaysListen()
{
  char client_message[200]= {0};

  memset(client_message, '\0', sizeof client_message);
  while(keepOnListening)
  {
    //Receive a reply from the client
    if( recv(sock, client_message, 200, 0) < 0)
    {
        printf("recv failed");
    }
    printf("RECIEVED : %s\n",client_message[0]);
  }
}


int SocketServer::AcceptSocket()
{
  int clientLen, read_size;
  struct sockaddr_in client;

  //Listen
  listen(hSocket, 1);

  printf("Waiting for incoming connections...\n");
  clientLen = sizeof(struct sockaddr_in);

  //accept connection from an incoming client
  sock = accept(hSocket,(struct sockaddr *)&client,(socklen_t*)&clientLen);
  if (sock < 0)
  {
      perror("accept failed");
      return 1;
  }
  printf("Connection accepted\n");
  return 0;
}
