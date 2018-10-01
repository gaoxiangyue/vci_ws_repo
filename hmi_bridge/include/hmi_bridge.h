#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/timer.h>
#include <map>

#include <can_msgs/Frame.h>
#include <vci_msgs/RadarObject.h>
#include <vci_msgs/RadarObjectArray.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>

#include "udp.hpp"
#include "protocol_parse.hpp"

namespace hmi_bridge {
//using namespace timesync;
using namespace std;

class udp_bridge
{
public:
  udp_bridge(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~udp_bridge();
  
private:
  void processTopic(const can_msgs::Frame::ConstPtr& can_msg);
  void publishTopic(const ros::WallTimerEvent& event);

  /* Subscribers & publishers */
     ros::Subscriber sub_topic_;
     ros::Publisher pub_topic_;
  /* ROS parameters */
     int fps_pub_;              // fps publishing topics
     std::string topic_name_rx_; // radar -> driver ("can_rx")
     std::string topic_name_tx_; // driver -> radar
  /* timers */
     ros::WallTimer timer_pub_;
  /* UDP parameters */

};

}//namespace hmi_bridge

#endif // hmi_bridge
