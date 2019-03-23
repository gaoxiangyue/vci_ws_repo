#ifndef PERCEPTION_H
#define PERCEPTION_H
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

namespace perception {
using namespace cv;
using namespace std;

class perception
{
public:
  perception(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~perception();
 
  void refreshPanel(std::string win_name, int &step);  
  void feature_extraction(Mat img_1, Mat img_2);
  /****************************************************
   * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
   * **************************************************/
  void find_feature_matches (
      const Mat& img_1, const Mat& img_2,
      std::vector<KeyPoint>& keypoints_1,
      std::vector<KeyPoint>& keypoints_2,
      std::vector< DMatch >& matches );

  void pose_estimation_2d2d (
      std::vector<KeyPoint> keypoints_1,
      std::vector<KeyPoint> keypoints_2,
      std::vector< DMatch > matches,
      Mat& R, Mat& t, Mat K);
  // 像素坐标转相机归一化坐标
  Point2d pixel2cam ( const Point2d& p, const Mat& K );
  void pose_estimation(Mat img_1, Mat img_2);                 
private:
  void processImageMsg(const sensor_msgs::ImageConstPtr& msg);

  /* Subscribers & publishers */
  image_transport::Subscriber sub_image_camera_;
  ros::Publisher pub_ov_;

  /* ROS parameters */
  std::string image_rx_; // image topic array choose
  std::string ov_tx_;


  /* */
  int cali_step;
  cv::Mat sub_img;
  cv::Mat main_img;

  double o_time=0;
  cv::Mat o_sub_img;
  double ddt=0.01;
  std_msgs::Header hd;
  
};

}//namespace perception

#endif // PERCEPTION_H
