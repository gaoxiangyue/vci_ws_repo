//
// Created by Hoping Tang on 8/7/17.
//

#ifndef CALIBRATIONTOOL_CALIBRATOR_H
#define CALIBRATIONTOOL_CALIBRATOR_H

#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>


class CalibBoard {
public:
    bool erase(size_t idx);
    bool swap_idx(size_t i, size_t j);

    std::vector<cv::Point2f> mid_line_;
    std::vector<cv::Point2f> undist_mid_line_;
    std::vector<int> code_idx_;
    std::vector<std::vector<int>> code_;
    // std::vector<std::vector<cv::Point>> code_coord_;
};

class Calibrator {
public:
    explicit Calibrator(const std::string &config_file);
    ~Calibrator() = default;

    enum class CamDirection {
        Front,
        Back,
        Left,
        Right
    };

    // step 0: read Intrinsic Param
    // step 1: + hood line
    // step 2: + extrinsic param
    // step 3: + static foe
    bool readCamParam(const std::string &camera_file, int step=0);
    bool readCamParam_json(const std::string &camera_file, int step=0);
    bool readMeasurement(const std::string &measure_file);
    // bool readIntrinsicParam(const std::string &camera_file);

    // step 2: calibrate extrinsic parameter
    bool saveCameraParam_json(const std::string &save_file);
    int if_cam_rot_180() {
        return do_rot180_;
    }

    bool set_hood_line(int yline);
    void getIntrinsicParam(cv::Mat &K, cv::Mat &distCoeffs, cv::Size &img_size);
    void getExtrinsicParam(cv::Mat &R, double &cam_h);

    // calibrate extrinsic parameter with movable board
    bool detect_board(const cv::Mat &img,
                      std::vector<cv::Point2f> &mid_line,
                      std::vector<cv::Point2f> &undist_mid_line,
                      std::vector<std::vector<int>> &code,
                      std::vector<cv::Point2f> &reproj_pts);
    bool estimateExtrinsicParam_mob(cv::Mat &R, double &cam_h,
                                    std::vector<CalibBoard> &calib_boards,
                                    std::vector<cv::Mat> &tvecs);

    // const cv::Mat R_rtv_ = (cv::Mat_<double>(3, 3) << 0, -1, 0, 0, 0, -1, 1, 0, 0);
    void match_code();

    bool find_line_mod(const cv::Mat &img, std::vector<cv::Point2f> &det_line,
                       std::vector<cv::Point2f> &undistort_line, std::vector<std::vector<int>> &code);
    bool find_code_idx(const std::vector<int> &code, int &code_idx, bool reverse);
    bool get_roi_valid();
    bool get_translation();
    //added by zhangdanfeng
    void compute_3D(cv::Point2i &image_point,cv::Point2d &_3D_point);
    Calibrator() {};
    void boards_pop_last_data();

private:
    double computeReprojectionErrors( const std::vector<cv::Point3f>& objectPoints,
                                      const std::vector<cv::Point2f>& imagePoints,
                                      std::vector<cv::Point2f>& reprojPoints,
                                      const cv::Mat &rvec, const cv::Mat &tvec,
                                      const cv::Mat &cameraMatrix , const cv::Mat &distCoeffs);

    cv::Size img_size_;
    bool do_rot180_=false;

    // R_ type is cv::CV_64F
    cv::Mat K_;
    cv::Mat distCoeffs_;
    cv::Mat R_;
    double cam_h_=-1.0;

    double square_size_;
    double board_height_;
    double x_offset_;

    std::vector<CalibBoard> calib_boards_;
    std::vector<std::vector<int>> code_dict_;
    std::vector<std::vector<int>> code_dict_rev_;

    float corner_score_threshold_;
    int hood_line_ = -1;
    double cam_to_left_;
    double cam_to_right_;
    double cam_to_front_;
    double car_width_;
    double car_length_;
    double mea_camera_height_;

    std::vector<double> translation_;
    CamDirection cam_direction_;

    std::vector<double> roi_valid_ = {-1., -1., -1., -1.};

};


#endif //CALIBRATIONTOOL_CALIBRATOR_H
