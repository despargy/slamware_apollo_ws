#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <pthread.h>	//include pthread
#include "SocketServer.cpp"

using namespace std;

//wrapper function
void *callMyFunction(void *object)
{
  ((SocketServer *)object)->AlwaysListen();
  return NULL;
}

int main(int argc,char** argv)
{
  // // create CONNECTION
  SocketServer mySocket;
  mySocket.AcceptSocket();

  pthread_t thread1;
  int  iret1 ;
  iret1 = pthread_create( &thread1, NULL, &callMyFunction, &mySocket);
  // int status_AlwaysListen = mySocket.AlwaysListen();

  char message[100] = {0};
  do
  {
    // Send some data
    printf("Enter message to send or 'stop':\n");
    scanf("%s",message);
    // std::cin>>message;
    if( send(mySocket.sock, message, strlen(message), 0) < 0)
    {
        printf("Send failed");
    }

  }while(true);

  close(mySocket.sock);

  return 0;
}
