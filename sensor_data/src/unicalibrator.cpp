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
#include "unicalibrator.hpp"
#include "cvui.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

#include "unicalibrator.hpp"

//GUI param
static const std::string OPENCV_WINDOW = "Calibration Tool";
const int alpha_slider_max = 100;
int alpha_slider;

namespace unicalibrator {

  unicalibrator::unicalibrator(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  {
    image_transport::ImageTransport it(nh);
    /* read parameter */
    READ_PARAM_BEGIN;
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, radar_rx_, "/radar/ars40821/objects");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, image_rx_, "/sensor/camera_1a01/image_test");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, home_path_, "/home/ros");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, pkg_path_, "/home/ros/catkin_ws/src/auto_ws_repo/calibration");///home/ros/catkin_ws/
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, configuration_file_, pkg_path_+"/config/config.json");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, measurement_file_, pkg_path_+"/config/sample_measurement.json");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, intrinsic_param_file_, pkg_path_+"/config/calib.yml");
    READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, extrinsic_param_file_, pkg_path_+"/config/extrinsic_param.json");
    
    /* subscribe topics*/
    //sub_can_radar_ = nh.subscribe(radar_rx_, 10, &unicalibrator::processRadarMsg,this);
    sub_image_camera_=it.subscribe(image_rx_,1, &unicalibrator::processImageMsg,this);

    /* publish topics*/
    image_pub_ = it.advertise(image_rx_, 1);

    /* timer */
    fps_pub_=10;
    fps_info_=1; 
    fps_refresh_=20;
    fps_scankey_=15; 
    timer_pub_image_=nh.createWallTimer(ros::WallDuration(1./fps_pub_), &unicalibrator::publishImage, this);
    timer_info_=nh.createWallTimer(ros::WallDuration(1./fps_info_), &unicalibrator::infoStates, this);
    //timer_refresh_=nh.createWallTimer(ros::WallDuration(1./fps_refresh_), &unicalibrator::refreshPanel, this);
    //timer_scankey_=nh.createWallTimer(ros::WallDuration(1./fps_scankey_), &unicalibrator::scanKey, this);

    /* GUI initial*/
    cali_step=0;
    pub_img=cv::imread(home_path_+"/catkin_ws/doc_img/simboard.jpg");
    sub_img = cv::Mat(cv::Size(768, 480), CV_8UC3);// no image
    //cv::createTrackbar( "TrackbarName", OPENCV_WINDOW, &alpha_slider, alpha_slider_max );
    //int bar=cv::getTrackbarPos( "TrackbarName",OPENCV_WINDOW);
    //int bar=alpha_slider;

    //test
    refreshPanel(OPENCV_WINDOW,cali_step);
    //======================================================================
    //system("python /home/ros/.ros/set_config.py");
    //system("fill_measurement.py -m "+"../config/sample_measurement.json");
    //return;
   
/*
    imgPath=home_path_+"/catkin_ws/doc_img/simboard.jpg";
    savePath=extrinsic_param_file_;

    std::cout<<"calibrateExtrinsicParam_mob3:"
            <<calibrateExtrinsicParam_mob3(
                configFile
                ,intrinsicParamFile
                ,measurementFile
                ,imgPath
                ,savePath
                ,saveParam
                )
            <<endl;

    //read/write json
    std::string init_config_file=(pkg_path_+"config/int_config.json");
    std:string example_write_file=(pkg_path_+"config/example_writer.json");
    std::cout<<"json_read:"<<json_read(init_config_file.data())<<endl;//open file example.json
    std::cout<<"json_write:"<<json_write(example_write_file.data(),true)<<endl;
    */
  }

  unicalibrator::~unicalibrator()
  {}

  void unicalibrator::processImageMsg(const sensor_msgs::ImageConstPtr& msg)
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
    sub_img = cv_ptr->image;
    
  }

//   void unicalibrator::processRadarMsg(const momenta_msgs::ArsObjectArray::ConstPtr &msg)
//   {
//       ROS_INFO("received momenta_msgs::ArsObjectArray");
//   }

  void unicalibrator::publishImage(const ros::WallTimerEvent& event)
  {
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg();
    image_pub_.publish(msg);
  }

  void unicalibrator::infoStates(const ros::WallTimerEvent& event)
  {
     // std::cout<<"infoStates onece."<<std::endl;
  }

  void unicalibrator::refreshPanel(std::string win_name, int &step)
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
        
        if(bt1&& step==0)
        {
            system("python /home/ros/catkin_ws/src/auto_ws_repo/calibration/script/set_config.py");
            system("python /home/ros/catkin_ws/src/auto_ws_repo/calibration/script/fill_measurement.py -m /home/ros/catkin_ws/src/auto_ws_repo/calibration/config/measurement.json");
            step=1;
        }
        if(bt2&& step==1)
        {
            set_hood_line();
            cvui::init(win_name, 20);
            step=2;         
        }
        if(bt3)
        {
            calibrate_extrinsic_param();
            cvui::init(win_name, 20);
            step=3;         
        }
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

  bool unicalibrator::set_hood_line()
  {
       /* init calib */
    Calibrator calib(configuration_file_);
    bool read_cam_ret = false;
    if (ends_with(intrinsic_param_file_, "json")) {
        read_cam_ret = calib.readCamParam_json(intrinsic_param_file_, 0);
    } else if (ends_with(intrinsic_param_file_, "yml")) {
        read_cam_ret = calib.readCamParam(intrinsic_param_file_, 0);
    }
    if (!read_cam_ret) {
        std::cerr << "read intrinsic_param_file failed." << std::endl;
        return false;
    }
    
    /* set_hood_line */
    if (sub_img.empty()) {
        std::cerr << "sub image msg failed." << std::endl;
        return false;
    }
    
    int yline;
    std::string window_name=OPENCV_WINDOW;
    set_yline(nullptr, sub_img, yline,window_name);
    calib.set_hood_line(yline);

    if (true) {
        calib.saveCameraParam_json(extrinsic_param_file_);
    }
    return true;
  }

  int unicalibrator::set_hood_line_from_file(std::string configFile
                                  , std::string intrinsicParamFile
                                  , std::string imgPath 
                                  , std::string savePath
                                  , bool saveParam
                                  )
  {
      Calibrator calib(configFile);

      bool read_cam_ret = false;
      if (ends_with(intrinsicParamFile, "json")) {
          read_cam_ret = calib.readCamParam_json(intrinsicParamFile, 0);
      } else if (ends_with(intrinsicParamFile, "yml")) {
          read_cam_ret = calib.readCamParam(intrinsicParamFile, 0);
      }
      if (!read_cam_ret) {
          return -1;
      }

      cv::Size img_show_size(768, 480);
      cv::Scalar instruction_color(50, 50, 130);

      cv::Mat img;
      img = cv::imread(imgPath);
      if (img.empty()) {
          std::cerr << "read image failed." << std::endl;
          return -1;
      }

      int yline;
      set_yline(nullptr, img, yline);

      calib.set_hood_line(yline);

      if (saveParam) {
          calib.saveCameraParam_json(savePath);
      }
      
      return 0;
  }

  bool unicalibrator::calibrate_extrinsic_param()
  {
      bool saveParam=true;
      std::string savePath=extrinsic_param_file_;
      #ifdef WITH_FLYCAPTURE
      std::ifstream configIfstream(configuration_file_);
      json config;
      configIfstream >> config;

      unsigned int snum = config["cam_serial_num"];
      PGCamera cam;
      cam.debug_ = true;
      cam.use_serial_num = true;
      bool cam_ret = cam.Init(snum);
      if (!cam_ret) {
          return -1;
      }
  #else
      FakeCamera cam;
      bool cam_ret = cam.Init("/dev/shm/temp/");
      if (!cam_ret) {
          return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  #endif

      cv::Mat R;
      double cam_h;

      Calibrator calib(configuration_file_);
      calib.readMeasurement(measurement_file_);
      bool read_cam_ret = false;
      if (ends_with(intrinsic_param_file_, "json")) {
          read_cam_ret = calib.readCamParam_json(intrinsic_param_file_, 1);
      } else if (ends_with(intrinsic_param_file_, "yml")) {
          read_cam_ret = calib.readCamParam(intrinsic_param_file_, 1);
      }
      if (!read_cam_ret) {
          return -1;
      }

      std::vector<cv::Mat> imgs;
      std::vector<cv::Mat> img_rois;

      cv::Scalar instruction_color(50, 50, 180);

      int index = -1;
      while(true) {
          ros::spinOnce();
          cv::Mat img;
          cv::Rect roi;

          index++;
          //
          img=sub_img;
           if (img.empty()) {
            std::cerr << "sub image msg failed." << std::endl;
            return false;
          }
          bool choose_roi_rst = choose_roi(nullptr, img, roi);
          //bool choose_roi_rst = choose_roi(&cam, img, roi);
          if (!choose_roi_rst) return -1;
          string img_path = "result/"+string("image_")+to_string(index)+".jpg";
          std::cout << img_path<<std::endl;
          cv::imwrite(img_path,img);

          cv::Mat img_roi = cv::Mat::zeros(img.size(), img.type());
          cv::Mat src_mat = img(roi);
          cv::Mat dst_mat = img_roi(roi);
          src_mat.copyTo(dst_mat);


          cv::Mat img_show;
          std::vector<cv::Point2f> mid_line;
          std::vector<cv::Point2f> undist_mid_line;
          std::vector<std::vector<int>> code;
          std::vector<cv::Point2f> reproj_pts;
          bool rst = calib.detect_board(img_roi, mid_line, undist_mid_line, code, reproj_pts);
          
          if (!rst) continue;

          draw_reprojection(img_roi, img_show, mid_line, reproj_pts);
          cv::putText(img_show, std::string("enter q to quit"),
                      cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::putText(img_show, std::string("enter c to choose"),
                      cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::putText(img_show, std::string("enter r to rechoose"),
                      cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::putText(img_show, std::string("progress: ") + std::to_string(imgs.size()) + std::string(" / 3"),
                      cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::imshow(OPENCV_WINDOW, img_show);

          bool quit = false;
          bool choose = false;
      
          while(true) {
              ros::spinOnce();
              bool validKey = false;
              int key = cv::waitKey(0);

              switch (key & 0xFF) {
                  case 'q':
                      quit = true;
                      validKey = true;
                      break;
                  case 'c':
                      validKey = true;
                      choose = true;
                  case 'r':
                      validKey = true;
                      break;
                  default:
                      break;
              }
              if (validKey)
                  break;
          }

          if (choose) {
              imgs.push_back(img);
              img_rois.push_back(img_roi);
          }else{
              calib.boards_pop_last_data();
          }
          if (imgs.size() == 3) quit = true;
          if (quit)
              break;
      }

      std::vector<CalibBoard> calib_boards;
      std::vector<cv::Mat> tvecs;
      bool ret = calib.estimateExtrinsicParam_mob(R, cam_h, calib_boards, tvecs);
      std::cout<<"estimateExtrinsicParam_mob:"<<ret<<endl;

      cv::Mat img_show;
      draw_match_mid_pts(imgs, img_show, calib_boards);
      cv::imshow(OPENCV_WINDOW, img_show);
      //cv::waitKey(0);

      if (ret) {
          std::string img_dir = savePath.substr(0, savePath.find_last_of("/\\") + 1);
          std::cout << "save image in: " << img_dir << std::endl;
          for (size_t i=0; i<imgs.size(); i++) {
              std::string img_name = std::string("mob_") + std::to_string(i) + std::string(".jpg");
              cv::imwrite(img_dir + img_name, imgs[i]);
          }

      }

      if (!ret) {
          return -1;
      }
      if (!calib.get_roi_valid()) {
          return -1;
      }

      if (saveParam) {
          std::string img_dir = savePath.substr(0, savePath.find_last_of("/\\") + 1);
          calib.saveCameraParam_json(savePath);
          json meta_data;
          meta_data["calib_board_coord"] = json::array();
          for (auto &tvec : tvecs) {
              json t = json::array();
              cout << "tvec" <<  tvec << endl;
              for (int r = 0; r < tvec.rows; r++) {
                  for (int c = 0; c < tvec.cols; ++c) {
                      t.push_back(tvec.at<double>(r, c));
                  }
              }
              meta_data["calib_board_coord"].push_back(t);
          }
          std::string meta_data_path = img_dir + "calib_meta_data.json";
          ofstream meta_data_file(meta_data_path);
          meta_data_file << setw(4) << meta_data;
      }

      cam.Stop();
  /*   */
      return 0;
  }

  int unicalibrator::calibrateExtrinsicParam_mob3(std::string configFile
                                        , std::string intrinsicParamFile
                                        , std::string measurementFile 
                                        , std::string imgPath
                                        , std::string savePath
                                        , bool saveParam
                                        )
  {
  #ifdef WITH_FLYCAPTURE
      std::ifstream configIfstream(configFile);
      json config;
      configIfstream >> config;

      unsigned int snum = config["cam_serial_num"];
      PGCamera cam;
      cam.debug_ = true;
      cam.use_serial_num = true;
      bool cam_ret = cam.Init(snum);
      if (!cam_ret) {
          return -1;
      }
  #else
      FakeCamera cam;
      bool cam_ret = cam.Init("/dev/shm/temp/");
      if (!cam_ret) {
          return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  #endif

      cv::Mat R;
      double cam_h;

      Calibrator calib(configFile);
      calib.readMeasurement(measurementFile);
      bool read_cam_ret = false;
      if (ends_with(intrinsicParamFile, "json")) {
          read_cam_ret = calib.readCamParam_json(intrinsicParamFile, 1);
      } else if (ends_with(intrinsicParamFile, "yml")) {
          read_cam_ret = calib.readCamParam(intrinsicParamFile, 1);
      }
      if (!read_cam_ret) {
          return -1;
      }

      std::vector<cv::Mat> imgs;
      std::vector<cv::Mat> img_rois;

      cv::Scalar instruction_color(50, 50, 180);

      int index = -1;
      while(true) {
          ros::spinOnce();
          cv::Mat img;
          cv::Rect roi;

          index++;
          //
          img=cv::imread(imgPath);
          bool choose_roi_rst = choose_roi(nullptr, img, roi);
          //bool choose_roi_rst = choose_roi(&cam, img, roi);
          if (!choose_roi_rst) return -1;
          string img_path = "result/"+string("image_")+to_string(index)+".jpg";
          std::cout << img_path<<std::endl;
          cv::imwrite(img_path,img);

          cv::Mat img_roi = cv::Mat::zeros(img.size(), img.type());
          cv::Mat src_mat = img(roi);
          cv::Mat dst_mat = img_roi(roi);
          src_mat.copyTo(dst_mat);


          cv::Mat img_show;
          std::vector<cv::Point2f> mid_line;
          std::vector<cv::Point2f> undist_mid_line;
          std::vector<std::vector<int>> code;
          std::vector<cv::Point2f> reproj_pts;
          bool rst = calib.detect_board(img_roi, mid_line, undist_mid_line, code, reproj_pts);
          
          if (!rst) continue;

          draw_reprojection(img_roi, img_show, mid_line, reproj_pts);
          cv::putText(img_show, std::string("enter q to quit"),
                      cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::putText(img_show, std::string("enter c to choose"),
                      cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::putText(img_show, std::string("enter r to rechoose"),
                      cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::putText(img_show, std::string("progress: ") + std::to_string(imgs.size()) + std::string(" / 3"),
                      cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
          cv::imshow(OPENCV_WINDOW, img_show);

          bool quit = false;
          bool choose = false;
      
          while(true) {
              bool validKey = false;
              int key = cv::waitKey(0);

              switch (key & 0xFF) {
                  case 'q':
                      quit = true;
                      validKey = true;
                      break;
                  case 'c':
                      validKey = true;
                      choose = true;
                  case 'r':
                      validKey = true;
                      break;
                  default:
                      break;
              }
              if (validKey)
                  break;
          }

          if (choose) {
              imgs.push_back(img);
              img_rois.push_back(img_roi);
          }else{
              calib.boards_pop_last_data();
          }
          if (imgs.size() == 3) quit = true;
          if (quit)
              break;
      }

      std::vector<CalibBoard> calib_boards;
      std::vector<cv::Mat> tvecs;
      bool ret = calib.estimateExtrinsicParam_mob(R, cam_h, calib_boards, tvecs);
      std::cout<<"estimateExtrinsicParam_mob:"<<ret<<endl;

      cv::Mat img_show;
      draw_match_mid_pts(imgs, img_show, calib_boards);
      cv::imshow(OPENCV_WINDOW, img_show);
      cv::waitKey(0);

      if (ret) {
          std::string img_dir = savePath.substr(0, savePath.find_last_of("/\\") + 1);
          std::cout << "save image in: " << img_dir << std::endl;
          for (size_t i=0; i<imgs.size(); i++) {
              std::string img_name = std::string("mob_") + std::to_string(i) + std::string(".jpg");
              cv::imwrite(img_dir + img_name, imgs[i]);
          }

      }

      if (!ret) {
          return -1;
      }
      if (!calib.get_roi_valid()) {
          return -1;
      }

      if (saveParam) {
          std::string img_dir = savePath.substr(0, savePath.find_last_of("/\\") + 1);
          calib.saveCameraParam_json(savePath);
          json meta_data;
          meta_data["calib_board_coord"] = json::array();
          for (auto &tvec : tvecs) {
              json t = json::array();
              cout << "tvec" <<  tvec << endl;
              for (int r = 0; r < tvec.rows; r++) {
                  for (int c = 0; c < tvec.cols; ++c) {
                      t.push_back(tvec.at<double>(r, c));
                  }
              }
              meta_data["calib_board_coord"].push_back(t);
          }
          std::string meta_data_path = img_dir + "calib_meta_data.json";
          ofstream meta_data_file(meta_data_path);
          meta_data_file << setw(4) << meta_data;
      }

      cam.Stop();
  /*   */
      return 0;
  }

  inline bool unicalibrator::ends_with(std::string const & value, std::string const & ending) 
  {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
  }

  int unicalibrator::json_read(const char* configFile)
  {
    Json::Value root;
    Json::Reader reader;
    std::ifstream ifs(configFile);
    if(!ifs)
    {
      // fail to open file
      return -1;
    }
    if(!reader.parse(ifs, root)){
      // fail to parse
      return -2;
    }
    else{
      // success
      std::cout<<root["cam_direction"].asString()<<endl;
      std::cout<<root["cam_name"].asString()<<endl;
      std::cout<<root["cam_serial_num"].asInt()<<endl;
      Json::Value obj0 = root["chessboard_param"];
          Json::Value obj1=obj0["board_size"];
            std::cout<<obj1["height"].asDouble()<<endl;
            std::cout<<obj1["width"].asDouble()<<endl;
          std::cout<<obj0["square_size"].asDouble()<<endl;
    }
    return 0;
  }

  int unicalibrator::json_write(const char* configFile,bool is_styled)
  {
    Json::Value root;
    Json::FastWriter fwriter;
    Json::StyledWriter swriter;
    Json::Value array;
    Json::Value position;
    position["x"]=1.0;
    position["y"]=1.0;
    position["z"]=1.0;
    array[0]=position;
    array[1]=position;
    root["array"] = array;
    std::string str = fwriter.write(root);
    std::ofstream ofs(configFile);
    if (ofs.is_open()) {
      if(!is_styled){
          ofs << str;
      }
      else{
          str = swriter.write(root);
          ofs << str;
      }
      ofs.close();
      return 0;
    }
    return -1;
  }

}//namespace unicalibrator
