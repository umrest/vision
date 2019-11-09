#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sys/socket.h>

class Socket
{
public:
    Socket() : connected(false)
    {
        // socket create and varification
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd == -1)
        {
            std::cout << "Failed to create socket... " << std::endl;
        }

        // assign IP, PORT
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr.sin_port = htons(8091);

        // connect the client socket to server socket
        if (connect(sockfd, (SA *)&servaddr, sizeof(servaddr)) != 0)
        {
            connected = false;
        }
        else
        {
            connected = true;
        }
    }
    ~Socket()
    {
        close(sockfb);
    }

    void send(char *data)
    {
    }

private:
    int sockfd, connfd;
    struct sockaddr_in servaddr, cli;

    bool connected;
};