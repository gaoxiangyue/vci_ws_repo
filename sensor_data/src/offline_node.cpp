#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include "json.hpp"
//
#include <thread>
#include "camera_wrapper.hpp"
#include "mouse_helper.hpp"
#include "draw_calib_board.hpp"
#include "Common.hpp"
#include "offline_node.hpp"
using namespace cv;
using namespace std;
using json = nlohmann::json;

namespace offline_node {


  offline_node::offline_node(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  {
    image_transport::ImageTransport it(nh);

    /* read parameter */
    READ_PARAM_BEGIN;
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, configuration_file_, "/home/user/catkin_ws/[DEFAULT]");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, pkg_path_, "/home/user/catkin_ws/src/auto_ws_repo/sensor_data[DEFAULT]");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, data_path_, "/home/user/catkin_ws/KITTI/2011_09_26_drive_0048_extract[DEFAULT]");

    /* timer */
    fps_pub_=10;
    timer_pub_image_=nh.createWallTimer(ros::WallDuration(1./fps_pub_), &offline_node::publishImage, this);

    /* publish topics*/
    image_tx_ = "image_tx";
    image_tx1_ = "image_tx1";
    image_pub_ = it.advertise(image_tx_, 1);
    image_pub1_ = it.advertise(image_tx1_, 1);

    pub_img=cv::imread(data_path_+"/image_02/data/0000000000.png");
    pub_img1=cv::imread(data_path_+"/image_02/data/0000000001.png");
    
  }


  offline_node::~offline_node()
  {}


  void offline_node::publishImage(const ros::WallTimerEvent& event)
  {
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg();
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img1).toImageMsg();
    image_pub_.publish(msg);
    image_pub1_.publish(msg1);
  }

}//namespace offline_node
