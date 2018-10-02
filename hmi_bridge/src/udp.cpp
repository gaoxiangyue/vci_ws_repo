#include "udp.hpp"

using namespace std;
namespace udp_space{


udp::udp()
{
    server_port=5000;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(&(server_addr.sin_zero),0,8);

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("Socket");
        exit(1);
        }

    if (bind(sock,(struct sockaddr *)&server_addr,
            sizeof(struct sockaddr)) == -1)
        {
        perror("Bind");
        exit(1);
        }

    addr_len = sizeof(struct sockaddr);
    cout << "\nICPServer Waiting for HMIClient on port: "<<ntohs(server_addr.sin_port)<< endl;
    fflush(stdout);
}

udp::~udp()
{}

bool udp::recv_from_udp(std::string &recv_data){
    char dataBuffer[1024];
    bytes_read = recvfrom(sock,dataBuffer,1024,0,
                                (struct sockaddr *)&client_addr, (socklen_t *) &addr_len);
    dataBuffer[bytes_read] = '\0';
    recv_data=dataBuffer;
    // cout << endl << inet_ntoa(client_addr.sin_addr) << " , " << ntohs(client_addr.sin_port);
    // cout << " said: " << recv_data;
    // cout.flush();
    return true;
}
bool udp::send_to_udp(std::string send_data){
    char cs[1024];
    strcpy(cs,send_data.c_str());
	sendto(sock, cs, strlen(cs), 0,
			   (struct sockaddr *)&client_addr, sizeof(struct sockaddr));
    return true;
}

}//namespace udp_space