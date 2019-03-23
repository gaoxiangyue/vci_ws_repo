#ifndef OFFLINE_NODE_H
#define OFFLINE_NODE_H
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <ros/timer.h>
#include <map>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

//ros cv_bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

namespace offline_node {

class offline_node
{
public:
  offline_node(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~offline_node();             
private:
  void publishImage(const ros::WallTimerEvent& event);
  void loadImage(std::string file_name);
  
  /* Subscribers & publishers */
  image_transport::Publisher image_pub_, image_pub1_;

  /* ROS parameters */
  int fps_pub_;                                                             // max fps publish image
  std::string image_tx_, image_tx1_;                                        // image topic array choose
  std::string data_path_;                                                   // kitti data path to load
  std::string configuration_file_;
  std::string pkg_path_;

  /* ROS timers */
  ros::WallTimer timer_pub_image_;

  /* Sensor parameters */
  cv::Mat pub_img, pub_img1;

};

}//namespace offline_node

#endif // OFFLINE_NODE_H
