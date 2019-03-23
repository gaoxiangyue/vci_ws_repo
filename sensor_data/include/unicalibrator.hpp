#ifndef UNI_CALIBRATOR_H
#define UNI_CALIBRATOR_H
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <ros/timer.h>
//ros cv_bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <map>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

//=================
/*
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
//#include "mouse_helper.hpp"
//#include "draw_calib_board.hpp"
#include "Common.hpp"
using namespace cv;
using namespace std;
using json = nlohmann::json;
*/

namespace unicalibrator {

class unicalibrator
{
public:
  unicalibrator(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~unicalibrator();
  int set_hood_line_from_file(std::string configFile
                          , std::string intrinsicParamFile
                          , std::string imgPath 
                          , std::string savePath
                          , bool saveParam
                          );
  int calibrateExtrinsicParam_mob3(std::string configFile
                          , std::string intrinsicParamFile
                          , std::string measurementFile 
                          , std::string imgPath
                          , std::string savePath
                          , bool saveParam
                          );   
  void refreshPanel(std::string win_name, int &step);
  bool set_hood_line();
  bool calibrate_extrinsic_param();                     
private:
  void processImageMsg(const sensor_msgs::ImageConstPtr& msg);
  // void processRadarMsg(const momenta_msgs::ArsObjectArray::ConstPtr &msg);
  void publishImage(const ros::WallTimerEvent& event);
  void infoStates(const ros::WallTimerEvent& event);
  void scanKey(const ros::WallTimerEvent& event);
  inline bool ends_with(std::string const & value, std::string const & ending);
  int json_read(const char* configFile);
  int json_write(const char* configFile, bool is_styled);

  /* Subscribers & publishers */
  ros::Subscriber sub_can_radar_;
  image_transport::Subscriber sub_image_camera_;
  image_transport::Publisher image_pub_;

  /* ROS parameters */
  int fps_pub_;          // max fps publish test_image
  int fps_info_;         // max fps info stat in screen
  int fps_refresh_;      // max fps refresh cv_window
  int fps_scankey_;      // max fps scankey interrupter
  std::string radar_rx_; // radar -> calibrator ("radar/ars40821/objects")
  std::string image_rx_; // image topic array choose
  //
  std::string home_path_;
  std::string pkg_path_;
  std::string configuration_file_;
  std::string measurement_file_;
  std::string intrinsic_param_file_;
  std::string extrinsic_param_file_;

  /* ROS timers */
  ros::WallTimer timer_pub_image_;
  ros::WallTimer timer_info_;
  ros::WallTimer timer_refresh_;
  ros::WallTimer timer_scankey_;

  /* LRR params */
  double min_prob_obj_;
  double min_prob_obstacle_;

  /*Calibration */
  int cali_step;
  cv::Mat pub_img;
  cv::Mat sub_img;
  cv::Mat main_img;
  
};

}//namespace unicalibrator

#endif // UNI_CALIBRATOR_H
