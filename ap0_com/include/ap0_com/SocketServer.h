#ifndef SOCKETSERVER_H
#define SOCKETSERVER_H

class SocketServer
{
  public:
    short hSocket;
    int sock;
    bool keepOnListening;
    SocketServer();
    void SocketCreate();
    int BindCreatedSocket();
    int AcceptSocket();
    void AlwaysListen();
};
#endif
