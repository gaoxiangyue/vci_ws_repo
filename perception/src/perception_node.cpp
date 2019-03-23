#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
//
#include <thread>
#include "Common.hpp"
#include "perception_node.hpp"
#include "cvui.h"

using namespace cv;
using namespace std;

//GUI param
static const std::string OPENCV_WINDOW = "Calibration Tool";
const int alpha_slider_max = 100;
int alpha_slider;

namespace perception {

  perception::perception(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  {
    image_transport::ImageTransport it(nh);
    /* read parameter */
    READ_PARAM_BEGIN;
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, image_rx_, "/kitti/camera_color_left/image_raw");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, ov_tx_, "/ovresult/tx");
    
    /* subscribe topics*/
    sub_image_camera_=it.subscribe(image_rx_,1, &perception::processImageMsg,this);
    /* publish topics*/
    pub_ov_ = nh.advertise<sensor_msgs::Imu>(ov_tx_, 1);

    /* GUI initial*/
    o_sub_img = cv::Mat(cv::Size(768, 480), CV_8UC3);// no image
    sub_img = cv::Mat(cv::Size(768, 480), CV_8UC3);// no image

    
    //test
    refreshPanel(OPENCV_WINDOW,cali_step);
  
  }

  perception::~perception()
  {}

  void perception::processImageMsg(const sensor_msgs::ImageConstPtr& msg)
  {
    //ROS_INFO("received sensor_msgs::Image");
    #if 1   //Always copy, returning a mutable CvImage
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    #else  //Share if possible, returning a const CvImage
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        else
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    #endif

    /* Process cv_ptr->image using OpenCV */
    
    std_msgs::Header h =msg->header;
    hd = h;
    double c_time = double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
    double dt =c_time-o_time;
    ddt = dt;
    sub_img = cv_ptr->image;
    if(o_time ==0) o_sub_img = sub_img;//init
    //sub_img &o_sub_img OV TEST
    feature_extraction(o_sub_img, sub_img);
    pose_estimation(o_sub_img, sub_img);
    cout<<("dt:%f",dt)<<endl;
    o_sub_img = sub_img;
    o_time = c_time;
    
  }

  void perception::refreshPanel(std::string win_name, int &step)
  {
    // cv::Mat img= cv::Mat::zeros(cv::Size(768, 480), CV_8UC3); 
    // img.setTo(cv::Scalar(100, 0, 0));
    // //
    // cv::putText(img, "calibration tool",
    //             cv::Point(60, 20), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(50, 50, 180), 2);
    // cv::putText(img, "Press 's' to start step"+std::to_string(cali_step)+"/3",
    //             cv::Point(60, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(50, 50, 180), 2);
    // //test
    // cv::circle(img, cv::Point(50, 50), 20, CV_RGB(255,0,0));
    // cv::imshow(win_name, img);
    cv::Size img_show_size(340, 220);
    cv::Mat frame=cv::Mat(Size(768, 480), CV_8UC3);
    //int count = 2;
    cv::namedWindow(win_name,WINDOW_AUTOSIZE);
    cv::moveWindow(win_name, 100, 100);
    cvui::init(win_name, 20);

	while (ros::ok) {   
        cv::Mat img;
        cv::resize(sub_img, img, img_show_size);
        cv::String space="                               ";
        bool bt1=cvui::button(frame, 40, 80, 350, 60,  "&1"+space);
        bool bt2=cvui::button(frame, 40, 150, 350, 60, "&2"+space);
        bool bt3=cvui::button(frame, 40, 220, 350, 60, "&3"+space);
        bool bt4=cvui::button(frame, 40, 290, 350, 60, "&4"+space);
        cvui::text(frame, 40, 40, "  Calibration tool",0.7);
        cvui::text(frame, 40, 100, "  Step : fill_measurement",0.6);
        cvui::text(frame, 40, 170, "  Step : set_hood_line",0.6); 
        cvui::text(frame, 40, 240, "  Step : calibrate_extrinsic_param",0.6);
        cvui::text(frame, 40, 310, "  Step : evaluate_extrinsic_param",0.6); 
        if(cvui::button(frame, 410, 310, 110, 40, "Show &Image"))
        ;
        else if(cvui::button(frame, 525, 310, 110, 40, "Show &Radar"))
        ;
        else if(cvui::button(frame, 640, 310, 110, 40, "Show &Lidar"))
        ;
        cvui::image(frame, 410, 80, img);
        //cvui::window(frame, 40, 180, 600, 600, "Title");
        //cvui::counter(frame, 160, 50, &count);
        
		if (cvui::button(frame, 40, 400, 100, 40, "&Quit")) { 
            break;
		}
        if (cvui::button(frame, 150, 400, 100, 40, "&Save")) { 
            break;
		}
		cvui::update();
		cv::imshow(win_name, frame);
        ros::spinOnce();
    }
    cv::destroyWindow(win_name);
  }

void perception::feature_extraction(cv::Mat img_1, cv::Mat img_2){
    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORB Keypoints",outimg1);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    //min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    //max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "Matches", img_match );
    imshow ( "Optimized Matches", img_goodmatch );
}

void perception::pose_estimation(cv::Mat img_1, cv::Mat img_2){
    // 相机内参,TUM Freiburg2
    //Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    Mat K = ( Mat_<double> ( 3,3 ) << 1000.0, 0, 160.0, 0, 1000.0, 120.0, 0, 0, 1 );//for panosim

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t, K );

    //-- 验证E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );

    cout<<"t^R="<<endl<<t_x*R<<endl;
    //
    double vx=t.at<double>(0,0);
    double vy=t.at<double>(1,0);
    double vz=t.at<double>(2,0);   
    sensor_msgs::Imu msg;
    msg.header =hd;
    msg.angular_velocity.x =vx;
    msg.angular_velocity.y =vy;
    msg.angular_velocity.z =vz;
    msg.linear_acceleration.x=ddt;
    for (int i=0;i<3;i++)
    for (int j=0;j<3;j++)
    {
        msg.angular_velocity_covariance[3*i+j]=R.at<double>(i,j);
    }
    pub_ov_.publish(msg);

    //-- 验证对极约束
    for ( DMatch m: matches )
    {
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
        //cout << "epipolar constraint = " << d << endl;
    }

}

void perception::find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3 
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2 
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}


Point2d perception::pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


void perception::pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t, Mat K )
{
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
    
}


}//namespace perception
