#ifndef PROTOCOL_PARSE
#define PROTOCOL_PARSE
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

namespace protocol_parse{
    
struct HMI_DISP_ {   
    /* Object cog */
    uint16_t object_id;
    uint16_t object_class;
    double object_x;
    double object_y;
    double object_vx;
    double object_vy;
    double object_width;
    double object_length;
    double object_height;
    double object_angle;
    /* Lane cog */
    uint16_t lane_id;
    uint16_t lane_class;
    double lane_a; 
    double lane_b;
    double lane_c;
    double lane_width;
    //tfc
    uint8_t traffic_light;
    /* Vehi state */
    uint32_t navi_id;
    double vehicle_lon;
    double vehicle_lat;
    double vehicle_alt;
    double vehicle_hdangle;
    double vehicle_speed;
    double vehicle_stangle;
    uint32_t system_state;
    uint8_t drive_mode;      
} ;

struct HMI_CONFIG_ {   
    /* Drv config */
    uint8_t drive_mode;
    double hope_speed;
    double time_headway; 
    uint32_t navi_id;
    double navi_lon;
    double navi_lat;
    double navi_alt;
    double navi_speed;      
} ;

class hmi_protocol{
    public:
        hmi_protocol();
        ~hmi_protocol();
        bool unpack(std::string str_recv, HMI_CONFIG_ &hmi_config);
        std::string pack(HMI_DISP_ hmi_disp); 
    private:
        struct NAVI{
            uint32_t navi_id;
            double navi_lon;
            double navi_lat;
            double navi_alt;
            double navi_speed;
        } navi_array[10];
};

}
#endif//namespace udp_space