#include <ros/ros.h>
#include "offline_node.hpp"

int main(int argc, char **argv)
{
ros::init(argc, argv, "offline_node");
ros::NodeHandle nh;
ros::NodeHandle nh_private("~");
offline_node::offline_node node(nh, nh_private);
ROS_INFO("Node initializing finished.");
ros::spin();
return 0;
}