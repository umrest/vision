#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

class Socket
{
public:
    Socket() : _connected(false)
    {
        

        reconnect();
    }
    ~Socket()
    {
        disconnect();
    }

    void disconnect(){
        close(sockfd);
    }

    void reconnect(){
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
        if (connect(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr)) != 0)
        {
            _connected = false;
        }
        else
        {
            _connected = true;
            std::cout << "Socket reconnected" << std::endl;
            int yes = 1;
            int res = setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char*) &yes, sizeof(int));
            std::cout << res << std::endl;
            char identifier[128];
            identifier[0] = 250;
            identifier[1] = 2;
            send_data(identifier);
            std::cout << "Sent identifier" << std::endl;
        }
    }

    void send_data(char *data, int size = 128)
    {
        if(send(sockfd, data, size, 0) < 0) {
            std::cout << "Socket send failed..." << std::endl;
            _connected = false;
            disconnect();
            reconnect();
        }
    }

    bool recieve_data(char* data)
    {
        int res = recv(sockfd, data, 128, MSG_DONTWAIT);

        if(res < 0){
            return false;
        }
        return true;
    }

    bool connected(){
        return _connected;
    }

private:
    int sockfd, connfd;
    struct sockaddr_in servaddr, cli;

    bool _connected;
};