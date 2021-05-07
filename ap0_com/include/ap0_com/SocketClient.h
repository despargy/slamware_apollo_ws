#ifndef SOCKETCLIENT_H
#define SOCKETCLIENT_H

class SocketClient
{
  public:
    short hSocket;
    SocketClient();
    short SocketCreate();
    int SocketConnect( int port_);
    int SocketSend(char* Rqst,short lenRqst);
    int SocketReceive(char* Rsp,short RvcSize);
    int listenToGui();
    void informGoalReached(int id, int battery);
};
#endif
