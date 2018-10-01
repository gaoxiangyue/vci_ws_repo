#ifndef UDP_H
#define UDP_H
#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

namespace udp_space{
    class udp
    {
    public:
        udp();
        ~udp();
        bool recv_from_udp(std::string &recv_data);
        bool send_to_udp(std::string send_data);
    
    private:
        int sock;
        int addr_len, bytes_read;
        std::string recv_data, send_data;
        struct sockaddr_in server_addr , client_addr;
        uint16_t server_port, client_port;
    };
}//namespace udp_space

#endif