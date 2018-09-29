#include "ros/ros.h"
#include "std_msgs/String.h"
#include "can_msgs/Frame.h"
#include "radar_config.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_config_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  radar_config::parse_frames parse(nh, private_nh);
  ROS_INFO("radar_config_node......");
  ros::spin();
  return 0;
}
