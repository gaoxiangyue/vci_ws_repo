#include <ros/ros.h>
#include "perception_node.hpp"

int main(int argc, char **argv)
{
ros::init(argc, argv, "perception_node");
ros::NodeHandle nh;
ros::NodeHandle nh_private("~");
perception::perception percep(nh, nh_private);
ROS_INFO("Node:initializing finished.");
ros::spin();
return 0;
}