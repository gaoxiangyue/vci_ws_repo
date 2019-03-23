#ifndef CHOOSE_ROI_HPP
#define CHOOSE_ROI_HPP
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "camera_wrapper.hpp"
#include "Calibrator.h"

typedef struct MouseParam_yline {
    MouseParam_yline(): yline(-1), clicked(false) {}

    int yline;
    bool clicked;
} MouseParam_yline;


void click_set_line( int event, int x, int y, int f, void* param) {

    MouseParam_yline *p_param = (MouseParam_yline *) param;
    int &yline = p_param->yline;
    bool &clicked = p_param->clicked;

    // std::cout << clicked << std::endl;

    switch (event) {
        case CV_EVENT_LBUTTONDOWN  :
            yline = y;
            clicked = true;
            break;

        case CV_EVENT_LBUTTONUP    :
            break;

        case CV_EVENT_MOUSEMOVE    :
            if (!clicked) {
                yline = y;
            }
            break;

        default                     :
            break;

    }
}

bool set_yline(CameraWrapper *cam, cv::Mat &img, int &yline) {

    if (cam == NULL && img.empty()) return false;

    cv::Mat img_show;
    cv::Scalar instruction_color(50, 50, 180);
    cv::Size img_show_size(768, 480);


    MouseParam_yline mouse_param;
    cv::namedWindow("Image window");
    cv::setMouseCallback("Image window", click_set_line, &mouse_param);

    while(ros::ok) {
        bool choosed = false;
        ros::spinOnce();
        if (cam != NULL) {
            bool frame_ret = cam->GetFrame(img);
            if ((!frame_ret) || img.empty()) {
                // fprintf(stderr, "Read frame failed\n");
                continue;
            }
        }

        cv::resize(img, img_show, img_show_size);

        cv::putText(img_show, std::string("clip mouse to set horizontal line"),
                    cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        cv::putText(img_show, std::string("enter c to correct"),
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);

        int s_yline = mouse_param.yline;
        cv::line(img_show, cv::Point(0, s_yline), cv::Point(img_show.cols-1, s_yline),
                 cv::Scalar(0,255,0),3);

        // std::cout << s_roi << std::endl;

        cv::imshow("Image window", img_show);
        int key = cv::waitKey(50);

        switch (key&0xFF) {
            case 'c':
                choosed = true;
                break;
            default:
                break;
        }

        if (choosed)
            break;
    }

    float hscale = float(img.size().height) / float(img_show_size.height);

    int &s_yline = mouse_param.yline;
    yline = int(s_yline * hscale);
    return true;
}

bool set_yline(CameraWrapper *cam, cv::Mat &img, int &yline, std::string window_name) {

    if (cam == NULL && img.empty()) return false;

    cv::Mat img_show;
    cv::Scalar instruction_color(50, 50, 180);
    cv::Size img_show_size(768, 480);


    MouseParam_yline mouse_param;
    cv::namedWindow(window_name);
    cv::setMouseCallback(window_name, click_set_line, &mouse_param);

    while(ros::ok) {
        bool choosed = false;
        ros::spinOnce();
        if (cam != NULL) {
            bool frame_ret = cam->GetFrame(img);
            if ((!frame_ret) || img.empty()) {
                // fprintf(stderr, "Read frame failed\n");
                continue;
            }
        }

        cv::resize(img, img_show, img_show_size);

        cv::putText(img_show, std::string("clip mouse to set horizontal line"),
                    cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        cv::putText(img_show, std::string("enter c to correct"),
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);

        int s_yline = mouse_param.yline;
        cv::line(img_show, cv::Point(0, s_yline), cv::Point(img_show.cols-1, s_yline),
                 cv::Scalar(0,255,0),3);

        // std::cout << s_roi << std::endl;

        cv::imshow(window_name, img_show);
        int key = cv::waitKey(50);

        switch (key&0xFF) {
            case 'c':
                choosed = true;
                break;
            default:
                break;
        }

        if (choosed)
            break;
    }

    float hscale = float(img.size().height) / float(img_show_size.height);

    int &s_yline = mouse_param.yline;
    yline = int(s_yline * hscale);

    //cv::destroyWindow(window_name);
    return true;
}

typedef struct MouseParam_ROI {
    cv::Point P1, P2;
    cv::Rect roi;
    bool clicked;

    MouseParam_ROI(): clicked(false) {}
} MouseParam_ROI;

void click_and_crop( int event, int x, int y, int f, void* param) {

    MouseParam_ROI * p_param = (MouseParam_ROI *) param;
    cv::Rect &roi = p_param->roi;
    bool &clicked = p_param->clicked;
    cv::Point &P1 = p_param->P1;
    cv::Point &P2 = p_param->P2;

    // std::cout << clicked << std::endl;

    switch (event) {
        case CV_EVENT_LBUTTONDOWN  :
            P1.x = x;
            P1.y = y;
            if (!clicked) {
                P2.x = x;
                P2.y = y;
            }
            clicked = true;
            break;

        case CV_EVENT_LBUTTONUP    :
            P2.x = x;
            P2.y = y;
            clicked = false;
            break;

        case CV_EVENT_MOUSEMOVE    :
            if (clicked) {
                P2.x = x;
                P2.y = y;
            }
            break;

        default                     :
            break;

    }


    if (clicked) {
        if (P1.x > P2.x) {
            roi.x = P2.x;
            roi.width = P1.x - P2.x;
        } else {
            roi.x = P1.x;
            roi.width = P2.x - P1.x;
        }

        if (P1.y > P2.y) {
            roi.y = P2.y;
            roi.height = P1.y - P2.y;
        } else {
            roi.y = P1.y;
            roi.height = P2.y - P1.y;
        }
        // std::cout << roi << std::endl;
    }
}

bool choose_roi(CameraWrapper *cam, cv::Mat &img, cv::Rect &roi) {

    if (cam == nullptr && img.empty()) return false;

    cv::Mat img_show;
    cv::Scalar instruction_color(50, 50, 180);
    cv::Size img_show_size(1280, 720);


    MouseParam_ROI mouse_param;
    cv::namedWindow("Calibration Tool");
    cv::setMouseCallback("Calibration Tool", click_and_crop, &mouse_param);

    while(ros::ok) {
        bool choosed = false;

        if (cam != nullptr) {
            bool frame_ret = cam->GetFrame(img);
            if ((!frame_ret) || img.empty()) {
                // fprintf(stderr, "Read frame failed\n");
                continue;
            }
        }

        cv::resize(img, img_show, img_show_size);

        cv::putText(img_show, std::string("clip mouse to crop"),
                    cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        cv::putText(img_show, std::string("enter c to choose"),
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);

        cv::Rect &s_roi = mouse_param.roi;
        cv::rectangle(img_show, s_roi, cv::Scalar(0,255,0), 3);

        // std::cout << s_roi << std::endl;

        cv::imshow("Calibration Tool", img_show);
        int key = cv::waitKey(50);

        switch (key&0xFF) {
            case 'c':
                choosed = true;
                break;
            default:
                break;
        }

        if (choosed)
            break;
    ros::spinOnce();
    }

    float hscale = float(img.size().height) / float(img_show_size.height);
    float wscale = float(img.size().width) / float(img_show_size.width);

    cv::Rect &s_roi = mouse_param.roi;
    roi.x = int(s_roi.x * wscale);
    roi.width = int(s_roi.width * wscale);
    roi.y = int(s_roi.y * hscale);
    roi.height = int(s_roi.height * hscale);

    // std::cout << roi << std::endl;

    // cv::Mat src_mat = img(roi);
    // cv::Mat dst_mat = img_roi(roi);
    // src_mat.copyTo(dst_mat);
    // cv::imshow("img", img_roi);
    // cv::waitKey(0);

    return true;
}

//added by zhangdanfeng

typedef struct MouseParam_point{
    bool clicked;
    cv::Point2i img_point;
    MouseParam_point():clicked(false),img_point(0,0){}
} MouseParam_point;

void click_choose_points( int event, int x, int y,int f,void * param)
{

    MouseParam_point* mouse_param = (MouseParam_point*)param;
    switch (event) {
        case CV_EVENT_LBUTTONDOWN  :
            mouse_param->img_point.x = x;
            mouse_param->img_point.y = y;
            mouse_param->clicked = true;
            break;

        case CV_EVENT_LBUTTONUP    :
            break;

        case CV_EVENT_MOUSEMOVE    :
            break;

        default                     :
            break;

    }

}


bool choose_point(Calibrator &calib,cv::VideoCapture cap) {

    if (!cap.isOpened()) return false;

    cv::Mat img_show;
    cv::Point2d _3D_point;
    cv::Scalar instruction_color(0, 255, 0);
    MouseParam_point mouse_param;
    cv::namedWindow("imshow");
    cv::setMouseCallback("imshow", click_choose_points, (&mouse_param));
    bool quit = false;
    bool choose = false;
    while(true)
    {
        cap >> img_show;
        if(img_show.empty())
            return true;
        cv::putText(img_show, std::string("clip mouse to choose"),
                    cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        cv::putText(img_show, std::string("enter q to quit"),
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        if(choose)
        {

            cv::circle(img_show, mouse_param.img_point,5, cv::Scalar(0,0,255),-1);
            std::string str_point;
            std::stringstream ss;
            std::string str1;
            std::string str2;
            ss.clear();
            ss <<std::fixed << std::setprecision(3) << _3D_point.x;
            ss >> str1;
            ss.clear();
            ss <<std::fixed << std::setprecision(3) << _3D_point.y;
            ss >> str2;
            ss.clear();
            str_point = str_point+"("+str1+","+str2+")";
            cv::putText(img_show, str_point,
                        mouse_param.img_point, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);

        }

        cv::imshow("imshow",img_show);
        int  key = cv::waitKey(50);
        switch (key&0xFF) {
            case 'q':
                quit = true;
                break;
            default:
                break;
        }
        if(quit)
            return true;
        if(mouse_param.clicked)
            choose = mouse_param.clicked;
        if(choose)
            calib.compute_3D(mouse_param.img_point,_3D_point);
    }

}


bool choose_point(Calibrator &calib,CameraWrapper *cam) {



    cv::Mat img_show;
    cv::Point2d _3D_point;
    cv::Scalar instruction_color(0, 255, 0);
    MouseParam_point mouse_param;
    cv::namedWindow("imshow");
    cv::setMouseCallback("imshow", click_choose_points, (&mouse_param));
    bool quit = false;
    bool choose = false;
    while(true)
    {
        ros::spinOnce();
        if (cam != nullptr) {
            bool frame_ret = cam->GetFrame(img_show);
            if ((!frame_ret) || img_show.empty()) {
                // fprintf(stderr, "Read frame failed\n");
                continue;
            }
        }
        if(img_show.empty())
            return true;
        cv::putText(img_show, std::string("clip mouse to choose"),
                    cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        cv::putText(img_show, std::string("enter q to quit"),
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2);
        if(choose)
        {

            cv::circle(img_show, mouse_param.img_point,5, cv::Scalar(0,0,255),-1);
            std::string str_point;
            std::stringstream ss;
            std::string str1;
            std::string str2;
            ss.clear();
            ss <<std::fixed << std::setprecision(3) << _3D_point.x;
            ss >> str1;
            ss.clear();
            ss <<std::fixed << std::setprecision(3) << _3D_point.y;
            ss >> str2;
            ss.clear();
            str_point = str_point+"("+str1+","+str2+")";
            cv::putText(img_show, str_point,
                        mouse_param.img_point, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);

        }

        cv::imshow("imshow",img_show);
        int  key = cv::waitKey(50);
        switch (key&0xFF) {
            case 'q':
                quit = true;
                break;
            default:
                break;
        }
        if(quit)
            return true;
        if(mouse_param.clicked)
            choose = mouse_param.clicked;
        if(choose)
            calib.compute_3D(mouse_param.img_point,_3D_point);
    }

}


#endif

