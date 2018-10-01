#include "ros/ros.h"
#include "hmi_bridge.h"
#include "Common.hpp"
#include <memory>
#include <cmath>

namespace hmi_bridge
{

udp_bridge::udp_bridge(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  //,timesync_(nh,nh_private)
{
  READ_PARAM_BEGIN;
  /* node parameter*/
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, fps_pub_, 10);
  READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, topic_name_rx_, "/hmi_display");
  READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, topic_name_tx_, "/hmi_config");
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, fps_udp_, 1000);

  /* publishers*/
  pub_topic_ = nh.advertise<vci_msgs::HmiConfig>(topic_name_tx_, 1);

  /* subscribers */
  sub_topic_ = nh.subscribe(topic_name_rx_, 100, &udp_bridge::processTopic,this);

  /* timer */
  //scheduling publishers
  timer_pub_ = nh.createWallTimer(ros::WallDuration(1./fps_pub_), &udp_bridge::publishTopic, this);
  //scheduling udp listen
  timer_udp_ = nh.createWallTimer(ros::WallDuration(1./fps_udp_), &udp_bridge::listenUdp, this);

  //last_timestamp_=ros::Time::now();
}

udp_bridge::~udp_bridge()
{

}

void udp_bridge::processTopic(const vci_msgs::HmiConfig::ConstPtr &msg)
{

}

void udp_bridge::publishTopic(const ros::WallTimerEvent& event)
{
  vci_msgs::HmiConfig config_topic;
  pub_topic_.publish(config_topic);

}

void udp_bridge::listenUdp(const ros::WallTimerEvent& event)
{
  //ROS_INFO("..");
  // std::string recv_str;
  // protocol_parse::HMI_CONFIG_ my_config;
  // protocol_parse::HMI_DISP_ my_disp;
  // protocol_parse::hmi_protocol my_proto;
  // udp_space::udp myudp;

  // /* Test field which can be removed when using*/
  // // the ivHMI should be made minor adjustments
  // {
  // // char recv_data[]="<1,2,3,4,5,6,7,8,9,10.1234567/>";
  // // recv_str=std::string(recv_data);
  // // my_proto.unpack(recv_str,my_config);
  // // std::cout<<"pack(my_disp):"<<my_proto.pack(my_disp)<<std::endl;
  // // std::cout<<"unpack(my_config)"<<my_config.hope_speed<<std::endl;
  // }
  // /*while 1 which should be moved into independent thread*/
  // while(1){
  //     if(myudp.recv_from_udp(recv_str))
  //     {
  //       std::cout<<"recv:"<<recv_str<<std::endl;
  //       if(!my_proto.unpack(recv_str,my_config))
  //       {
  //         std::cout<<"protol.unpack Failure..."<<std::endl;
  //       }
  //       else
  //       {
  //         //get my_config here
  //         {//for example as below:
  //            std::cout<<"hope_speed from HMI:"<< my_config.hope_speed<<std::endl;
  //         }
  //         //assign my_disp here
  //         {//for example as below:
  //           my_disp.drive_mode=my_config.drive_mode;
  //           my_disp.system_state=0;
  //           my_disp.vehicle_stangle=60;
  //           my_disp.navi_id=1;
  //           my_disp.vehicle_lon=125.1234567;
  //           my_disp.vehicle_lat=44.1234567;
  //           my_disp.vehicle_alt=230.1;
  //           my_disp.vehicle_speed=36.12;
  //           my_disp.traffic_light=2;
  //           my_disp.lane_id=1;
  //           my_disp.lane_a=50.6;
  //           my_disp.lane_b=12.3;
  //           my_disp.lane_c=23.4;
  //           my_disp.lane_class=1;
  //           my_disp.lane_width=0.15;
  //           my_disp.object_id=12;
  //           my_disp.object_x=10.1;
  //           my_disp.object_y=5.3;
  //           my_disp.object_vx=10.1;
  //           my_disp.object_vy=0.2;
  //           my_disp.object_class=1;
  //           my_disp.object_width=3;
  //           my_disp.object_length=5;
  //           my_disp.object_height=1.8;
  //           my_disp.system_state=0;
  //         }
  //         //-----------------------------------
  //         std::string send_str=my_proto.pack(my_disp);
  //         myudp.send_to_udp(send_str);
  //       }
  //     }
  //   }
}

}//namespace hmi_bridge
