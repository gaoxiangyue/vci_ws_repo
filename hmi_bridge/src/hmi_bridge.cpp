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

  /* publishers*/
  pub_topic_ = nh.advertise<can_msgs::Frame>(topic_name_tx_, 1);

  /* subscribers */
  sub_topic_ = nh.subscribe(topic_name_rx_, 100, &udp_bridge::processTopic,this);

  /* scheduling publishers*/
  timer_pub_ = nh.createWallTimer(ros::WallDuration(1./fps_pub_), &udp_bridge::publishTopic, this);

  //last_timestamp_=ros::Time::now();
}

udp_bridge::~udp_bridge()
{

}

void udp_bridge::processTopic(const can_msgs::Frame::ConstPtr &can_msg)
{

}

void udp_bridge::publishTopic(const ros::WallTimerEvent& event)
{
    can_msgs::Frame config_frame;
    pub_topic_.publish(config_frame);
}

}//namespace hmi_bridge
