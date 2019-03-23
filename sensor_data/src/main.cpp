#include <ros/ros.h>
#include "unicalibrator.hpp"

int main(int argc, char **argv)
{
ros::init(argc, argv, "calibration");
ros::NodeHandle nh;
ros::NodeHandle nh_private("~");
unicalibrator::unicalibrator unicali(nh, nh_private);
ROS_INFO("Node:initializing finished.");
ros::spin();
return 0;
}