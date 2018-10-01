#include "protocol_parse.hpp"
#include <iomanip> 

namespace protocol_parse{

hmi_protocol::hmi_protocol(){}

hmi_protocol::~hmi_protocol(){}


bool hmi_protocol::unpack(std::string str_recv, HMI_CONFIG_ &hmi_config){
  const int data_size=10;
  std::string str=str_recv;
  std::size_t sz = str.find('<');
  double data[data_size];
  for (int i=0;i<data_size;i++)
  {
      str=str.substr(sz+1);
      if(((int)str[0]>47 &&(int)str[0]<58)||(int)str[0]==43||(int)str[0]==45)
            data[i]=std::stod (str,&sz);
      else
      {
            data[i]=0;
            return false;
      }    
      // std::cout<<std::setprecision(10)<<","<<data[i];
  }
  hmi_config.drive_mode=data[0];
  hmi_config.hope_speed=data[1];
  return true;
}


std::string hmi_protocol::pack(HMI_DISP_ hmi_disp){

  const int data_size=25;
  std::string data[data_size];
  data[0]=std::to_string(hmi_disp.drive_mode);
  data[1]=std::to_string(hmi_disp.system_state);
  data[2]=std::to_string(hmi_disp.vehicle_stangle);
  data[3]=std::to_string(hmi_disp.navi_id);
  data[4]=std::to_string(hmi_disp.vehicle_lon);
  data[5]=std::to_string(hmi_disp.vehicle_lat);
  data[6]=std::to_string(hmi_disp.vehicle_alt);
  data[7]=std::to_string(hmi_disp.vehicle_speed);
  data[8]=std::to_string(hmi_disp.traffic_light);
  data[9]=std::to_string(hmi_disp.lane_id);
  data[10]=std::to_string(hmi_disp.lane_a);
  data[11]=std::to_string(hmi_disp.lane_b);
  data[12]=std::to_string(hmi_disp.lane_c);
  data[13]=std::to_string(hmi_disp.lane_class);
  data[14]=std::to_string(hmi_disp.lane_width);
  data[15]=std::to_string(hmi_disp.object_id);
  data[16]=std::to_string(hmi_disp.object_x);
  data[17]=std::to_string(hmi_disp.object_y);
  data[18]=std::to_string(hmi_disp.object_vx);
  data[19]=std::to_string(hmi_disp.object_vy);
  data[20]=std::to_string(hmi_disp.object_class);
  data[21]=std::to_string(hmi_disp.object_width);
  data[22]=std::to_string(hmi_disp.object_length);
  data[23]=std::to_string(hmi_disp.object_height);
  data[24]=std::to_string(hmi_disp.object_angle);

  std::string packed="<";
  for(int i=0;i<data_size;i++)
  {
    packed+=data[i];
    packed+=",";
  }
  packed[packed.length()-1]='/';
  packed+=">";
  return packed;
}

}//namespace protocol_parse