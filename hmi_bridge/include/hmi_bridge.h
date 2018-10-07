#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/timer.h>
#include <map>

#include <can_msgs/Frame.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <vci_msgs/RadarObject.h>
#include <vci_msgs/RadarObjectArray.h>
#include <vci_msgs/HmiConfig.h>

#include "udp.hpp"
#include "protocol_parse.hpp"

#define NAVI_SIZE 10

namespace hmi_bridge {
//using namespace timesync;
using namespace std;

class udp_bridge
{
public:
  udp_bridge(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~udp_bridge();
  
private:
  void processTopic(const vci_msgs::HmiConfig::ConstPtr& msg);
  void publishTopic(const ros::WallTimerEvent& event);
  void listenUdp(const ros::WallTimerEvent& event);
  int isColline( double x,double y, std::vector<geometry_msgs::Point> points, double H_PRECISION);


  /* Subscribers & publishers */
     ros::Subscriber sub_topic_;
     ros::Publisher pub_topic_;
  /* ROS parameters */
     int fps_pub_;               // fps publishing topics
     std::string topic_name_rx_; // 
     std::string topic_name_tx_; // 
     std::string hmi_frame_id_;
  /* timers */
     ros::WallTimer timer_pub_;
     ros::WallTimer timer_udp_;
  /* UDP parameters */
     int fps_udp_;                // fps lishening udp msgs 
     udp_space::udp myudp;

     vci_msgs::HmiConfig hmi_config;

};

}//namespace hmi_bridge

#endif // hmi_bridge
