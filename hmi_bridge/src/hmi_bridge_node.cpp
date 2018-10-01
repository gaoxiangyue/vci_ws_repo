#include "ros/ros.h"
#include "std_msgs/String.h"
#include "can_msgs/Frame.h"
#include "hmi_bridge.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hmi_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  hmi_bridge::udp_bridge parse(nh, private_nh);
  ROS_INFO("hmi_bridge_node......");
  ros::spin();
  return 0;
}
