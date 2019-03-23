//
// Created by Hoping Tang on 8/7/17.
//

#include "Calibrator.h"
#include "FindCorners.h"
#include <fstream>
#include "json.hpp"
#include <ros/ros.h>
using json = nlohmann::json;
using namespace std;

template <typename Enumeration>
auto as_integer(Enumeration const value)
-> typename std::underlying_type<Enumeration>::type {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

// duplicated function from object_filter
void json_array_to_mat(const json &jarr, cv::Mat &mat) {
    int cnt = 0;
    for (int r = 0; r < mat.rows; r++) {
        for (int c = 0; c < mat.cols; ++c) {
            mat.at<double>(r, c) = jarr[cnt++];
        }
    }
}


bool Calibrator::readCamParam_json(const std::string &camera_file, int step) {

    json calib;
    std::ifstream fin(camera_file);
    fin >> calib;
    fin.close();

    K_ = cv::Mat(3, 3, CV_64F);
    distCoeffs_ = cv::Mat(5, 1, CV_64F);
    // intrinsic parameters
    json_array_to_mat(calib["camera_matrix"], K_);
    json_array_to_mat(calib["distortion_coefficients"], distCoeffs_);

    // other params
    int img_width_ = calib["image_width"];
    int img_height_ = calib["image_height"];
    img_size_ = cv::Size(img_width_, img_height_);

    if (calib.find("do_rot180") != calib.end())
        do_rot180_ = calib["do_rot180"];

    // step 1
    if (calib.find("hood_line") != calib.end()) {
        hood_line_ = calib["hood_line"];
    } else {
        hood_line_ = img_height_;
    }

    // step 2
    if (calib.find("rotation_matrix") != calib.end()) {
        R_ = cv::Mat(3, 3, CV_64F);
        json_array_to_mat(calib["rotation_matrix"], R_);
        cam_h_ = calib["camera_height"];
    }

    if(calib.find("cam_to_front") != calib.end() && calib.find("cam_to_left")!=calib.end() &&
       calib.find("cam_to_right")!=calib.end() &&calib.find("camera_height")!=calib.end()){
        cam_to_left_ = calib["cam_to_left"];
        cam_to_front_ = calib["cam_to_front"];
        cam_to_right_ = calib["cam_to_right"];
        cam_h_ = calib["camera_height"];
        translation_.push_back(cam_to_front_);
        translation_.push_back((cam_to_left_-cam_to_right_)/2);
        translation_.push_back(-cam_h_);


    }

    std::cout << "finished reading cam params" << std::endl;
    return true;
}


bool Calibrator::readCamParam(const std::string &camera_file, int step) {
    // read from yml file
    cv::FileStorage f_cam(camera_file, cv::FileStorage::READ);
    if(!f_cam.isOpened()) {
        std::cout<< "Failed to read camera file: " << camera_file << std::endl;
        return false;
    } else {
        std::cout << "Successfully opened camera file: " << camera_file << std::endl;
    }

    // read params
    int img_width, img_height;
    f_cam["image_width"] >> img_width;
    f_cam["image_height"] >> img_height;

    f_cam["camera_matrix"] >> K_;
    f_cam["distortion_coefficients"] >> distCoeffs_;
    f_cam["do_rot180"] >> do_rot180_;

    if (step > 0) {
        f_cam["hood_line"] >> hood_line_;

        if (step > 1){
            f_cam["rotation_matrix"] >> R_;
            f_cam["camera_height"] >> cam_h_;
        }
    }
    img_size_ = cv::Size(img_width, img_height);
    std::cout << "finished reading cam params" << std::endl;
    return true;
}

bool Calibrator::saveCameraParam_json(const std::string &save_file) {
    json cameraParam = json{
            {"image_width", img_size_.width},
            {"image_height", img_size_.height},
            {"do_rot180", do_rot180_},
            {"camera_height", cam_h_},
    };

    cameraParam["rotation_matrix"] = json::array();
    for (int i=0; i<R_.rows; i++)
        for (int j=0; j<R_.cols; j++)
            cameraParam["rotation_matrix"].push_back(R_.at<double>(i, j));

    cameraParam["camera_matrix"] = json::array();
    for (int i=0; i<K_.rows; i++)
        for (int j=0; j<K_.cols; j++)
            cameraParam["camera_matrix"].push_back(K_.at<double>(i, j));

    cameraParam["distortion_coefficients"] = json::array();
    for (int i=0; i<distCoeffs_.rows * distCoeffs_.cols; i++)
        cameraParam["distortion_coefficients"].push_back(distCoeffs_.at<double>(i));

    if (hood_line_ > 0) {
        cameraParam["hood_line"] = hood_line_;
    }

    cameraParam["foe_x"] = K_.at<double>(0, 2);
    cameraParam["foe_y"] = K_.at<double>(1, 2);

    if (cam_to_left_ >= 0 && cam_to_right_ >= 0 && cam_to_front_ >= 0) {
        cameraParam["cam_to_left"] = cam_to_left_;
        cameraParam["cam_to_right"] = cam_to_right_;
        cameraParam["cam_to_front"] = cam_to_front_;
        // cameraParam["car_left_corner"] = {cam_to_front_, cam_to_left_};
        // cameraParam["car_right_corner"] = {cam_to_front_, -cam_to_right_};
    }

    if (car_width_ >= 0 && car_length_ >= 0) {
        cameraParam["car_width"] = car_width_;
        cameraParam["car_length"] = car_length_;
    }

    if (roi_valid_.size() == 4 && roi_valid_[2] > 0) {
        cameraParam["roi_valid"] = roi_valid_;
    }

    switch (cam_direction_) {
        case CamDirection::Front:
            cameraParam["cam_direction"] = "Front";
            break;
        case CamDirection::Left:
            cameraParam["cam_direction"] = "Left";
            break;
        case CamDirection::Right:
            cameraParam["cam_direction"] = "Right";
            break;
        case CamDirection::Back:
            cameraParam["cam_direction"] = "Back";
            break;
    }

    // if (!translation_.empty()) {
    //     cameraParam["translation"] = translation_;
    // }
    cameraParam["use_vcam"] = true;

    ofstream cameraParamFile(save_file);
    if (!cameraParamFile.is_open()) {
        std::cout<< "Failed to open save file: " << save_file << std::endl;
        return false;
    } else {
        cameraParamFile << std::setw(4) << cameraParam << std::endl;
        std::cout << "Successfully saved camera parameter file: " << save_file << std::endl;
        return true;
    }
}

double Calibrator::computeReprojectionErrors( const std::vector<cv::Point3f>& objectPoints,
                                              const std::vector<cv::Point2f>& imagePoints,
                                              std::vector<cv::Point2f>& reprojPoints,
                                              const cv::Mat &rvec, const cv::Mat &tvec,
                                              const cv::Mat &cameraMatrix , const cv::Mat &distCoeffs) {
    int pointNum = (int)objectPoints.size();

    // project
    cv::projectPoints( cv::Mat(objectPoints), rvec, tvec, cameraMatrix,
                       distCoeffs, reprojPoints);

    // difference
    double err = cv::norm(cv::Mat(imagePoints), cv::Mat(reprojPoints), CV_L2);

    return err / pointNum;
}


void Calibrator::getIntrinsicParam(cv::Mat &K, cv::Mat &distCoeffs, cv::Size &img_size) {
    K = K_.clone();
    distCoeffs = distCoeffs_.clone();
    img_size = img_size_;
}

void Calibrator::getExtrinsicParam(cv::Mat &R, double &cam_h) {
    R = R_.clone();
    cam_h = cam_h_;
}

void retrive_3d_line_cam_coord(const std::vector<cv::Point2f> &line2d, const vector<int> &idx_code_dict,
                               const cv::Mat &K_, const float squareSize,
                               cv::Point3d &bottom_pt, cv::Point3d &direction_vec) {
    int point_num = int(line2d.size());

    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    // TODO: will consider skew be better?
    cv::Mat M(2*point_num, 6, CV_64F);
    for (int i=0; i<point_num; i++) {
        // the coeef of control point is
        // 1 1 1 1 1 1
        // 0 1 2 3 4 5
        M.at<double>(2*i, 0) = fx;
        M.at<double>(2*i, 1) = 0.0;
        M.at<double>(2*i, 2) = cx - (double)line2d[i].x;
        M.at<double>(2*i, 3) = idx_code_dict[i] * fx;
        M.at<double>(2*i, 4) = 0.0;
        M.at<double>(2*i, 5) = idx_code_dict[i] * (cx - (double)line2d[i].x);


        M.at<double>(2*i+1, 0) = 0.0;
        M.at<double>(2*i+1, 1) = fy;
        M.at<double>(2*i+1, 2) = cy - (double)line2d[i].y;
        M.at<double>(2*i+1, 3) = 0.0;
        M.at<double>(2*i+1, 4) = idx_code_dict[i] * fy;
        M.at<double>(2*i+1, 5) = idx_code_dict[i] * (cy - (double)line2d[i].y);
    }

    cv::Mat MtM = M.t() * M;
    cv::Mat W, U, Vt;
    cv::SVDecomp(MtM, W, U, Vt, CV_SVD_MODIFY_A);

    // TODO: N = 2, N = 3, N = 4
    // N = 1, only one null singlar value
    cv::Mat eigen = Vt.row(5);
    double * peigen = eigen.ptr<double>();
    // cout << "W: " << W << endl;

    bottom_pt.x = peigen[0];
    bottom_pt.y = peigen[1];
    bottom_pt.z = peigen[2];

    direction_vec.x = peigen[3];
    direction_vec.y = peigen[4];
    direction_vec.z = peigen[5];

    if (bottom_pt.z < 0) {
        bottom_pt = -bottom_pt;
        direction_vec = -direction_vec;
    }

    double b = squareSize / cv::norm(direction_vec);
    bottom_pt = b * bottom_pt;
    direction_vec = b * direction_vec;
}

void inline intersection_of_plane_and_line(const cv::Point3d &plane_normal, const double &plane_d,
                                           const cv::Point3d &line_pt, const cv::Point3d &line_v, cv::Point3d &intersect) {
    // (line_pt + lamda * line_v).dot(plane_normal) = plane_d
    double denominator = line_v.dot(plane_normal);
    // assert(n != 0);

    double lamda = (plane_d - line_pt.dot(plane_normal) ) / denominator;
    intersect = line_pt + lamda * line_v;
}

void retrive_xw_with_x_offset(const cv::Point3d &intersect, const cv::Point3d &plane_normal, const double x_offset,
                              cv::Point3d &Xw, const Calibrator::CamDirection &cam_direction) {

    // TODO: check if cos_theta > 1 then return false
    double cos_theta = x_offset / cv::norm(intersect);

    if (cam_direction == Calibrator::CamDirection::Back)
        cos_theta = -cos_theta;

    // sin theta is positive if the first point is the lowest point
    double sin_theta = std::sqrt(1 - cos_theta * cos_theta);

    // rotate vector p3 with rotation vector p2 and angle theta
    cv::Point3d origin_w = cos_theta * intersect + sin_theta * (plane_normal.cross(intersect))
                           + (1 - cos_theta) * (plane_normal.dot(intersect)) * plane_normal;
    origin_w = origin_w * float(std::abs(x_offset) / cv::norm(origin_w));

    Xw = intersect - origin_w;
    Xw = Xw * (1.0 / cv::norm(Xw));
    if (cam_direction == Calibrator::CamDirection::Back)
        Xw = - Xw;
}

// TODO: merge these two func
void retrive_xw_with_two_board(const vector<cv::Point3d> &direction_vecs, const vector<cv::Point3d> &bottom_pts,
                               cv::Point3d &Xw, Calibrator::CamDirection cam_direction) {
    cv::Point3d b = bottom_pts[0] - bottom_pts[1];
    const cv::Point3d &k1 = direction_vecs[0];
    const cv::Point3d &k2 = direction_vecs[1];
    // TODO: pt num is the number of mid line
    auto pt_num = (int)bottom_pts.size();

    if (b.z > 0) {
        Xw = (pt_num - 1) / 2.f * (k1 - k2) + b;
    } else {
        Xw = (pt_num - 1) / 2.f * (k2 - k1) - b;
    }

    if (cam_direction == Calibrator::CamDirection::Back)
        Xw = -Xw;

    Xw = Xw * (1.0 / cv::norm(Xw));
}

void retrive_yw_with_two_board(const vector<cv::Point3d> &direction_vecs, const vector<cv::Point3d> &bottom_pts,
                               cv::Point3d &Yw, Calibrator::CamDirection cam_direction) {
    cv::Point3d b = bottom_pts[0] - bottom_pts[1];
    const cv::Point3d &k1 = direction_vecs[0];
    const cv::Point3d &k2 = direction_vecs[1];
    // TODO: pt num is the number of mid line
    auto pt_num = (int)bottom_pts.size();

    if (b.z > 0) {
        Yw = (pt_num - 1) / 2.f * (k1 - k2) + b;
    } else {
        Yw = (pt_num - 1) / 2.f * (k2 - k1) - b;
    }

    if (cam_direction == Calibrator::CamDirection::Right)
        Yw = -Yw;

    Yw = Yw * (1.0 / cv::norm(Yw));
}

void hough_points(std::vector<cv::Point2f> &corners, std::vector<cv::Point2f> &undistort_corners) {
    double max_distance=0;
    for (int i=0; i<(int)undistort_corners.size(); i++) {
        double norm = cv::norm(undistort_corners[i]);
        if (norm > max_distance) max_distance = norm;
    }

    // std::cout << max_distance << std::endl;
    int r_ratio = 10;
    double theta_step = M_PI / 180.0;
    double theta_range = M_PI / theta_step;

    cv::Mat hough_space = cv::Mat::zeros(2 * int(max_distance / r_ratio),
                                         int(theta_range), CV_32F);

    for (int i =0; i<(int)undistort_corners.size(); i++) {
        auto &p = undistort_corners[i];

        for (int a=0; a<int(theta_range); a++) {
            double theta = theta_step * a;
            double r = ((p.x * cos(theta) + p.y * sin(theta)) + max_distance)/ r_ratio;

            int row = int(r);
            // std::cout << col << " ";
            if (row > hough_space.rows - 1) row = hough_space.rows - 1;
            if (row < 0) row = 0;
            hough_space.at<float>(row, a) += 1;
        }
        // std::cout << std::endl;
    }

    // cv::imshow("hough", hough_space);
    // cv::waitKey(0);
    double min_val, max_val;
    cv::Point min_loc, max_loc;

    cv::minMaxLoc(hough_space, &min_val, &max_val, &min_loc, &max_loc);
    // std::cout << min_val << std::endl;
    // std::cout << max_val << std::endl;

    double theta = max_loc.x * M_PI / 180.0;
    double r = max_loc.y * r_ratio - max_distance;
    for (auto p =undistort_corners.begin(); p != undistort_corners.end();){

        double residual = std::abs(p->x * cos(theta) + p->y * sin(theta) - r);
        // std::cout << residual << std::endl;
        if (residual > r_ratio * 10) {
            auto idx = std::distance(undistort_corners.begin(), p);
            corners.erase(corners.begin() + idx);
            p = undistort_corners.erase(p);
        } else
            p++;
    }

}

void fit_vertical_line(std::vector<cv::Point2f> &points, float &line_k, float &line_b) {
    int point_num = (int) points.size();
    cv::Mat b(point_num, 1, CV_32F), A(point_num, 2, CV_32F), param(2, 1, CV_32F);
    for (int i=0; i<point_num; i++) {
        b.at<float>(i, 0) = points[i].x;
        A.at<float>(i, 0) = points[i].y;
        A.at<float>(i, 1) = 1.f;
    }
    param = (A.t() * A).inv() * A.t() * b;
    line_k = param.at<float>(0, 0);
    line_b = param.at<float>(1, 0);
}

bool get_step(const std::vector<cv::Point2f> &undistort_line, float &step) {
    size_t point_num = undistort_line.size();
    std::vector<double> steps;
    for (size_t i=0; i<point_num-1; i++) {
        steps.push_back(cv::norm(undistort_line[i+1] - undistort_line[i]));
        // cout << "undisted distances: "<< cv::norm(undistort_line[i+1] - undistort_line[i])<< endl;
    }

    for (int i=0; i<(int)steps.size(); i++) {
        double step_mean = std::accumulate(steps.begin(), steps.end(), 0.0) / steps.size();
        double max_diff = -1;
        size_t max_idx = 0;
        for (size_t j=0; j< steps.size(); j++) {
            double diff = std::abs(steps[j] - step_mean);
            if (diff > max_diff) {
                max_idx = j;
                max_diff = diff;
            }
            
        }
        // TODO: adjust this threshold
        if (max_diff > 5) {
            cout << "max diff: " << max_diff << endl;
            steps.erase(steps.begin()+max_idx);
        } else {
            break;
        }
        if (steps.size() < 3) {
            return false;
        }
    }
    step = (float) std::accumulate(steps.begin(), steps.end(), 0.0) / steps.size();
    return true;
}

bool Calibrator::find_line_mod(const cv::Mat &img, std::vector<cv::Point2f> &det_line,
                               std::vector<cv::Point2f> &undistort_line,
                               std::vector<std::vector<int>> &code) {
    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    cv::Mat imgGray;
    cvtColor(img, imgGray, CV_BGR2GRAY);

    FindCorners corner_detector;
    corner_detector.detectCorners(imgGray, det_line, corner_score_threshold_);

    cv::Mat img_grey_undist;
    undistort(imgGray, img_grey_undist, K_, distCoeffs_);

    int point_num = (int) det_line.size();
    if (point_num <= 4) return false;

    std::sort(det_line.begin(), det_line.end(),
              [&](cv::Point2f p1,  cv::Point2f p2) { return p1.y > p2.y; });

    // std::cout << cv::Mat(det_line) << std::endl;
    cv::undistortPoints(det_line, undistort_line, K_, distCoeffs_);
    for (int i=0; i<point_num; i++) {
        undistort_line[i].x = (float) (fx * undistort_line[i].x + cx);
        undistort_line[i].y = (float) (fy * undistort_line[i].y + cy);
    }

    // remove outliers
    hough_points(det_line, undistort_line);
    point_num = (int) det_line.size();
    if (point_num <= 3) return false;

    // refine line and remove outliers
    float line_pk, line_pb;
    fit_vertical_line(undistort_line, line_pk, line_pb);

    for (auto p=undistort_line.begin(); p != undistort_line.end(); ){
        float delta = - p->x + line_pk * p->y + line_pb;
        if (std::abs(delta) > 10) {
            std::cout << "delta: " << delta << std::endl;
            auto idx = std::distance(undistort_line.begin(), p);
            det_line.erase(det_line.begin() + idx);
            p = undistort_line.erase(p);
        } else {
            p++;
        }
    }
    point_num = (int) det_line.size();
    if (point_num <= 3) return false;
    // std::cout << undistort_line.size() << std::endl;
    // std::cout << det_line.size() << std::endl;

    // for (int i=0; i<point_num; i++) {
    //     auto &p = undistort_line[i];
    //     cv::circle(img_grey_undist, p, 5, CV_RGB(0, 0, 255), 2);
    // }
    // cv::imshow("src", img_grey_undist);
    // cv::waitKey(0);

    // get code
    fit_vertical_line(undistort_line, line_pk, line_pb);
    float step = -1;
    bool get_step_ret = get_step(undistort_line, step);
    if (!get_step_ret) return false;

    // cout << "step: " << step << endl;

    det_line.erase(det_line.begin());
    det_line.erase(det_line.end()-1);
    undistort_line.erase(undistort_line.begin());
    undistort_line.erase(undistort_line.end()-1);

    point_num = (int) det_line.size();
    if (point_num <= 3) return false;

    float l_step_x = step * line_pk / std::sqrt(line_pk * line_pk + 1);
    float l_step_y = step / std::sqrt(line_pk * line_pk + 1);
    float v_step_x = l_step_y;
    float v_step_y = l_step_x;
    // cout << "l_step_x: " << l_step_x << endl;
    // cout << "l_step_y: " << l_step_y << endl;

    cv::Mat show = img_grey_undist.clone(), show_s;
    code.clear();
    code.resize((unsigned long) point_num);

    for (auto p=undistort_line.begin(); p != undistort_line.end(); ){
        auto i = std::distance(undistort_line.begin(), p);
        bool remove_p = false;
        std::vector<double> pixels;

        for (int n = 0; n < 4; n++) {
            for (int m = 0; m < 4; m++) {
                float x = p->x + (l_step_x * (n - 1.5f) + v_step_x * (m -1.5f)) * 1.f;
                float y = p->y + l_step_y * (n - 1.5f) + v_step_y * (m -1.5f);
                if (int(x) >= img_grey_undist.cols || int(y) >= img_grey_undist.rows || int(x) < 0 || int(y) < 0)
                    remove_p = true;

                if (remove_p) break;
                pixels.push_back((double) img_grey_undist.at<uchar>((int)y, (int)x));

                cv::Point2f show_p(x, y);
                // int show_text = pixels[pixels.size() - 1] > 100;
                if (n == 0)
                    cv::circle(show, show_p, 2, cv::Scalar(255, 255, 255), -1);
                // cv::putText(show, std::to_string(show_text),
                //             show_p, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            }
            if (remove_p) break;
        }
        if (remove_p) {
            p = undistort_line.erase(p);
            det_line.erase(det_line.begin() + i);
            code.erase(code.begin() + i);
            continue;
        } else {
            p++;
        }

        double mean_val = std::accumulate(pixels.begin(), pixels.end(), 0.0) / pixels.size();
        // std::cout << "mean: "<< mean_val << std::endl;

        for (int c=0; c<16; c++) {
            if (pixels[c] > mean_val)
                code[i].push_back(1);
            else
                code[i].push_back(0);
        }
    }
    cv::resize(show, show_s, cv::Size(1280, 720));
    cv::putText(show_s, std::string("enter anykey to continue"),
                cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::imshow("Calibration Tool", show_s);
    cv::waitKey(0);
    return true;
}

void fill_code_dict(const vector<vector<int>> &mob_code,
                    vector<vector<int>> &code_dict) {
    if (code_dict.empty()) {
        code_dict.resize(15);
        for (int i = 0; i < 15; i++) {
            code_dict[i].resize(16);
            for (int j = 0; j < 16; j++) {
                code_dict[i][j] = mob_code[i + 3 - j/4][j%4];
            }
        }
    }
}

bool code_equal(const std::vector<int> &code1, const std::vector<int> &code2) {
    bool rst = true;
    for (int c=0; c<(int)code1.size(); c++) {

        // std::cout << code1[c]<< " " << code2[c] << std::endl;
        rst = rst & (code1[c] == code2[c]);
    }
    // std::cout << rst << std::endl;
    return rst;
}


bool CalibBoard::erase(size_t idx) {
    if (idx >= mid_line_.size())
        return false;

    mid_line_.erase(mid_line_.begin() + idx);
    undist_mid_line_.erase(undist_mid_line_.begin() + idx);
    code_idx_.erase(code_idx_.begin() + idx);
    code_.erase(code_.begin() + idx);
    // code_coord_.erase(code_coord_.begin() + idx);
    return true;
}

bool CalibBoard::swap_idx(size_t i, size_t j) {
    if (i >= mid_line_.size() || j >= mid_line_.size())
        return false;

    std::iter_swap(mid_line_.begin()+i, mid_line_.begin()+j);
    std::iter_swap(undist_mid_line_.begin()+i, undist_mid_line_.begin()+j);
    std::iter_swap(code_idx_.begin()+i, code_idx_.begin()+j);
    std::iter_swap(code_.begin()+i, code_.begin()+j);
    // std::iter_swap(code_coord_.begin()+i, code_coord_.begin()+j);
    return true;
}


Calibrator::Calibrator(const std::string &config_file) {
    cam_to_left_ = -1.0;
    cam_to_right_ = -1.0;
    cam_to_front_ = -1.0;
    car_width_ = -1.0;
    car_length_ = -1.0;
    mea_camera_height_ = -1.0;
    R_ = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    // load config
    std::ifstream configIfstream(config_file);
    json config;
    configIfstream >> config;
    configIfstream.close();

    square_size_ = config["mob_square_size"];
    corner_score_threshold_ = config["corner_score_threshold"];

    vector<vector<int>> mob_code;
    bool use_new_code = config["use_new_code"];
    if (use_new_code) {
        vector<vector<int>> code = config["mob_code_dict"];
        mob_code = code;
    } else {
        vector<vector<int>> code = config["tac_code_dict"];
        mob_code = code;
    }

    fill_code_dict(mob_code, code_dict_);
    code_dict_rev_ = code_dict_;
    for (auto &cdr : code_dict_rev_) {
        std::reverse(cdr.begin(), cdr.end());
    }
    std::reverse(code_dict_rev_.begin(), code_dict_rev_.end());
    string cam_d = config["cam_direction"];
    for (auto &c:cam_d) {
        c = static_cast<char>(std::toupper(c));
    }
    cout << "camera direction: " << cam_d << endl;
    if (cam_d == "FRONT") {
        cam_direction_ = CamDirection::Front;
    } else if (cam_d == "LEFT") {
        cam_direction_ = CamDirection::Left;
    } else if (cam_d == "RIGHT") {
        cam_direction_ = CamDirection::Right;
    } else if (cam_d == "BACK") {
        cam_direction_ = CamDirection::Back;
    } else {
        cam_direction_ = CamDirection::Front;
        cerr << "Wrong camera direction, use front as default." << endl;
    }
    cout << "camera direction: " << as_integer(cam_direction_) << endl;
}

bool Calibrator::readMeasurement(const std::string &measure_file) {
    json mea;
    try {
        std::ifstream fin(measure_file);
        fin >> mea;
        fin.close();

        cam_to_left_ = mea["cam_to_left"];
        cam_to_right_ = mea["cam_to_right"];
        cam_to_front_ = mea["cam_to_front"];
        if (mea.find("car_width") != mea.end()) {
            car_width_ = mea["car_width"];
        }
        if (mea.find("car_length") != mea.end()) {
            car_length_ = mea["car_length"];
        }

        board_height_ = mea["mob_height"];
        mea_camera_height_ = mea["mea_cam_height"];
    } catch (exception& e) {
        cout << e.what() << endl;
        return false;
    }

    x_offset_ = (cam_to_left_ - cam_to_right_)/2.0;

    return true;
}


bool Calibrator::detect_board(const cv::Mat &img,
                              std::vector<cv::Point2f> &mid_line,
                              std::vector<cv::Point2f> &undist_mid_line,
                              std::vector<std::vector<int>> &code,
                              std::vector<cv::Point2f> &reproj_pts) {

    bool found = find_line_mod(img, mid_line, undist_mid_line, code);
    
    if (!found)
        return false;

    vector<int> idx_code_dict;
    int code_num = 0, code_rev_num = 0;
    for (size_t i = 0; i < code.size(); i++) {
        int idx = 0;
        bool found_code_idx = find_code_idx(code[i], idx, false);
        if (found_code_idx) {
            code_num++;
        }
        bool found_code_idx_rev = find_code_idx(code[i], idx, true);
        if (found_code_idx_rev) {
            code_rev_num++;
        }
        if (!(found_code_idx || found_code_idx_rev)) {
            mid_line.erase(mid_line.begin() + i);
            undist_mid_line.erase(undist_mid_line.begin() + i);
            code.erase(code.begin() + i);
            // code_coord.erase(code_coord.begin()+i);
            i--;
        }
    }

    if (code_num < 3 && code_rev_num < 3) {
        return false;
    }

    if (code_rev_num == code_num) {
        cerr << "could not judge code reverse or not." << endl;
        return false;
    }

    bool is_rev = (code_rev_num > code_num);
    for (size_t i = 0; i < code.size(); i++) {
        int idx = 0;
        find_code_idx(code[i], idx, is_rev);
        auto len = idx_code_dict.size();
        idx_code_dict.push_back(idx);
        if (len > 0) {
            if (idx <= idx_code_dict[len - 1]) {
                mid_line.erase(mid_line.begin() + i);
                undist_mid_line.erase(undist_mid_line.begin() + i);
                code.erase(code.begin() + i);
                // code_coord.erase(code_coord.begin()+i);
                idx_code_dict.erase(idx_code_dict.begin() + i);
                i--;
            }
        }
    }

    if (mid_line.size() < 2) {
        cerr << "find code failed." << endl;
        return false;
    }

    CalibBoard calib_board;
    calib_boards_.push_back(calib_board);
    size_t cur_idx = calib_boards_.size() - 1;

    calib_boards_[cur_idx].mid_line_ = mid_line;
    calib_boards_[cur_idx].undist_mid_line_ = undist_mid_line;
    calib_boards_[cur_idx].code_ = code;
    // calib_boards_[cur_idx].code_coord_ = code_coord;
    calib_boards_[cur_idx].code_idx_ = idx_code_dict;

    // std::cout << mid_line << std::endl;
    // std::cout << undist_mid_line << std::endl;
    int point_num = (int) undist_mid_line.size();

    cv::Point3d bottom_pt, direction_vec;
    // retrive mid line in 3d camera coord.
    retrive_3d_line_cam_coord(undist_mid_line, idx_code_dict, K_,
                              (float)square_size_, bottom_pt, direction_vec);

    cv::Point3d intersect;
    intersection_of_plane_and_line(direction_vec, 0.0, bottom_pt, direction_vec, intersect);
    std::cout << "bottom_pt: " << bottom_pt << std::endl;

    cv::Point3d Xw;
    retrive_xw_with_x_offset(intersect, direction_vec, x_offset_, Xw, CamDirection::Front);

    cv::Point3d Zw = direction_vec * (1.0 / cv::norm(direction_vec));
    cv::Point3d Yw = Zw.cross(Xw);
    cv::Mat Xwmat = cv::Mat(Xw);
    cv::Mat Ywmat = cv::Mat(Yw);
    cv::Mat Zwmat = cv::Mat(Zw);
    cv::Mat R;
    cv::hconcat(Xwmat, Ywmat, R);
    cv::hconcat(R, Zwmat, R);

    // std::cout << "R = " << R << std::endl;

    std::vector<cv::Point3f> object_points;
    for (int i=0; i<point_num; i++) {
        object_points.emplace_back(cv::Point3f(0.0, 0.0, idx_code_dict[i] * (float)square_size_));
        // std::cout << object_points[i] << std::endl;
    }

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cv::Mat tvec;
    tvec = (cv::Mat_<double>(3, 1) << bottom_pt.x, bottom_pt.y, bottom_pt.z);
    double reprojErr = computeReprojectionErrors(object_points, mid_line, reproj_pts, rvec, tvec, K_, distCoeffs_);
    cout << "reproj Err: " << reprojErr << endl;

    return true;
}


void Calibrator::match_code() {
    int img_num = (int) calib_boards_.size();

    auto &f_uml = calib_boards_[0].undist_mid_line_;
    auto &f_c= calib_boards_[0].code_;

    for (auto p=f_uml.begin(); p!=f_uml.end();) {
        bool have_match = true;
        size_t idx = (size_t) std::distance(f_uml.begin(), p);

        for (int lidx=1; lidx < img_num; lidx++) {
            bool matched = false;
            auto &b = calib_boards_[lidx];

            for (size_t match_idx = idx; match_idx < b.undist_mid_line_.size(); match_idx++) {
                // std::cout << "match_idx:" << match_idx << std::endl;
                if (code_equal(b.code_[match_idx], f_c[idx])) {
                    b.swap_idx((size_t) idx, (size_t) match_idx);
                    matched = true;
                    break;
                }
            }

            if (!matched) {
                have_match = false;
                break;
            }
        }
        if (have_match) {
            p++;
        } else {
            p--;
            calib_boards_[0].erase((size_t) idx);
            p++;
        }
    }

    for (int lidx=1; lidx < img_num; lidx++) {
        while(calib_boards_[lidx].mid_line_.size() > f_uml.size()) {
            size_t end_idx = calib_boards_[lidx].mid_line_.size() - 1;
            calib_boards_[lidx].erase(end_idx);
        }
    }

}


bool Calibrator::find_code_idx(const std::vector<int> &code, int &code_idx, bool reverse) {

    bool found = false;
    auto * code_dict = &code_dict_;
    if (reverse)
        code_dict = &code_dict_rev_;

    for (int ci=0; ci < (int)code_dict->size(); ci++) {
        if (code_equal(code, (*code_dict)[ci])) {
            code_idx = ci;
            found = true;
            break;
        }
    }

    return found;
}

bool Calibrator::estimateExtrinsicParam_mob(cv::Mat &R, double &cam_h,
                                            std::vector<CalibBoard> &calib_boards,
                                            std::vector<cv::Mat> &tvecs) {
    // TODO: Logging system

    // movable board is in the middle of car in first image
    std::vector<cv::Point3d> bottom_pts(3), direction_vecs(3);

    // get road plane by connecting points from mid_lines of three images respectively
    match_code();

    if (calib_boards_[0].code_.empty()) {
        cerr << "find code failed." << endl;
        return false;
    }

    // TODO: check and avoid two of the lines too close
    // and the intersection points will Severely affected by noise

    for (int idx = 0; idx < 3; idx++) {
        std::vector<cv::Point2f> &undist_mid_line = calib_boards_[idx].undist_mid_line_;
        cv::Point3d &bottom_pt = bottom_pts[idx];
        cv::Point3d &direction_vec = direction_vecs[idx];
        // retrive mid line in 3d camera coord.
        retrive_3d_line_cam_coord(undist_mid_line, calib_boards_[idx].code_idx_,
                                  K_, (float) square_size_, bottom_pt, direction_vec);
    }


    // check and avoid two of the lines too close
    // and the intersection points will Severely affected by noise
    if (cv::norm(bottom_pts[1] - bottom_pts[2]) < 0.2) {
        cerr << "last two mob place too close." << endl;
        return false;
    }

    // check the place order
    // if (cam_direction_ == CamDirection::Front) {
    //     if (bottom_pts[0].z >= bottom_pts[1].z || bottom_pts[0].z >= bottom_pts[2].z) {
    //         cerr << "first place should be closest." << endl;
    //         return false;
    //     }
    // }

    // TODO: direct get plane normal without obtain line 3d each imgs
    int img_num = 3;
    auto point_num = calib_boards_[0].undist_mid_line_.size();
    std::vector<cv::Point3d> plane_normals(point_num);
    for (int idx = 0; idx < (int) point_num; idx++) {
        auto &plane_normal = plane_normals[idx];
        // plane equation: A * X + B * Y + C * Z = D
        // => A/B * X + Y + C/B * Z - D/B = 0
        // => A/B * X + C/B * Z - D/B = -Y
        cv::Mat A(img_num, 3, CV_64F), b(img_num, 1, CV_64F);
        for (int i = 0; i < img_num; i++) {
            auto p = bottom_pts[i] + calib_boards_[i].code_idx_[idx] * direction_vecs[i];
            A.at<double>(i, 0) = p.x;
            A.at<double>(i, 1) = p.z;
            A.at<double>(i, 2) = 1.0;
            b.at<double>(i, 0) = p.y;
        }
        cv::Mat solve = (A.t() * A).inv() * A.t() * b;
        plane_normal.x = solve.at<double>(0, 0);
        plane_normal.y = -1.0;
        plane_normal.z = solve.at<double>(1, 0);
        plane_normal = plane_normal * (1.0 / cv::norm(plane_normal));
        // std::cout << plane_normal << std::endl;
        double angle = acos(plane_normal.dot(cv::Point3d(0., -1., 0.))) / M_PI * 180.0;
        cout << "plane angle: " << angle << " degree" << endl;
    }

    cv::Point3d plane_normal = std::accumulate(plane_normals.begin(), plane_normals.end(), cv::Point3d(0.0, 0.0, 0.0))
                               * (1.0 / double(plane_normals.size()));

    // std::cout << plane_normal << std::endl;

    cv::Point3d Xw, Yw, Zw;
    Zw = plane_normal * (1.0 / cv::norm(plane_normal));

    if (cam_direction_ == CamDirection::Front || cam_direction_ == CamDirection::Back) {
        // cv::Point3d intersect;
        // intersection_of_plane_and_line(plane_normal, 0.0, bottom_pts[0], direction_vecs[0], intersect);
        // retrive_xw_with_x_offset(intersect, plane_normal, x_offset_, Xw, cam_direction_);
        // Yw = Zw.cross(Xw);
        retrive_xw_with_two_board(direction_vecs, bottom_pts, Xw, cam_direction_);
        Xw = Xw - Xw.dot(Zw) * Zw;
        Xw = Xw * (1.0 / cv::norm(Xw));
        Yw = Zw.cross(Xw);
        cout << "check x norm: " << Xw.dot(Xw) << endl;
    } else {
        // if (cam_direction == CamDirection::Left || cam_direction == CamDirection::Right)
        retrive_yw_with_two_board(direction_vecs, bottom_pts, Yw, cam_direction_);
        Yw = Yw - Yw.dot(Zw) * Zw;
        Yw = Yw * (1.0 / cv::norm(Yw));
        Xw = Yw.cross(Zw);
        cout << "check x norm: " << Xw.dot(Xw) << endl;
    }

    cv::Mat Xwmat = cv::Mat(Xw);
    cv::Mat Ywmat = cv::Mat(Yw);
    cv::Mat Zwmat = cv::Mat(Zw);
    R = cv::Mat();
    cv::hconcat(Xwmat, Ywmat, R);
    cv::hconcat(R, Zwmat, R);

    cv::Mat tvec;
    tvec = (cv::Mat_<double>(3, 1) << bottom_pts[0].x, bottom_pts[0].y, bottom_pts[0].z);
    tvec = R.t() * tvec;

    // plus 1 because the bottom point is removed
    auto code_height = square_size_;
    cam_h = -tvec.at<double>(2, 0) + board_height_ + code_height;
    cam_h_ = cam_h;
    R_ = R.clone();

    // if (cam_direction_ == CamDirection::Front) {
    //     cam_to_front_ = tvec.at<double>(0, 0);
    // }

    std::cout << "R = " << R << std::endl;

    calib_boards = calib_boards_;

    for (auto &bt_pt: bottom_pts) {
        cv::Mat t;
        t = (cv::Mat_<double>(3, 1) << bt_pt.x, bt_pt.y, bt_pt.z);
        t = R.t() * t;
        tvecs.push_back(t);
    }

    if (cam_direction_ == CamDirection::Front) {
        cam_to_left_ = std::abs(tvecs[0].at<double>(1, 0));
        cam_to_right_ = std::abs(tvecs[2].at<double>(1, 0));
    }

    if (abs(cam_h - mea_camera_height_) > 0.15) {
        cerr << "camera height err, measurement: "
             << mea_camera_height_
             << ", calibrated: "
             << cam_h << endl;
        return false;
    }

    if (cam_direction_ == CamDirection::Front) {
        if (abs(cam_to_front_ - tvec.at<double>(0, 0)) > 0.50) {
            cerr << "camera to front warning, measurement: "
                 << cam_to_front_
                 << ", calibrated: "
                 << tvec.at<double>(0, 0) << endl;
        }
    }

    return true;
}

bool Calibrator::set_hood_line(int yline) {
    hood_line_ = yline;
    return true;
}

bool Calibrator::get_roi_valid() {
    if (hood_line_ < img_size_.height/2.0) {
        cerr << "please set hood line first" << endl;
        return false;
    }
    if (K_.rows == 0 || distCoeffs_.cols == 0) {
        cerr << "camera intrinsic param is not set" << endl;
        return false;
    }

//    vector<Point2d> ori_pts;
//    ori_pts.emplace_back(0., 0.);
//    ori_pts.emplace_back(Point2d(0., hood_line_));
//    ori_pts.emplace_back(Point2d(img_size_.width, 0.));
//    ori_pts.emplace_back(Point2d(img_size_.width, hood_line_));
//
//    vector<Point2d> dst_pts(4);
//    undistortPoints(ori_pts, dst_pts, K_, distCoeffs_, Mat::eye(3, 3, CV_64F), K_);
//
//    double left = (dst_pts[0].x + dst_pts[1].x) / 2.0;
//    left = max(0.0, left);
//
//    double right = (dst_pts[2].x + dst_pts[3].x) / 2.0;
//    right = min(img_size_.width, right);
//
//    double top = (dst_pts[0].y + dst_pts[2].y) / 2.0;
//    top = max(0.0, top);
//
//    double bottom = (dst_pts[1].y + dst_pts[3].y) / 2.0;
//    bottom = min(img_size_.height, bottom);
//
//    roi_valid_ = {left, top, right - left, bottom - top};

    double left = 50.0;
    double right = img_size_.width - 50.0;
    double top = 50.0;
    double bottom = hood_line_ - 50.0;
    roi_valid_ = {left, top, right - left, bottom - top};
    return true;
}

bool Calibrator::get_translation() {
    translation_.resize(3);
    translation_[0] = -cam_to_front_;
    translation_[2] = cam_h_;

    switch (cam_direction_) {
        case CamDirection::Front:
            translation_[1] = x_offset_;
            break;
        case CamDirection::Left:
            translation_[1] = car_width_ / 2;
            break;
        case CamDirection::Right:
            translation_[1] = -car_width_ / 2;
            break;
        case CamDirection::Back:
            translation_[1] = x_offset_;
            break;
    }

    return true;
}

//added by zhangdanfeng
void Calibrator::compute_3D(cv::Point2i &image_point,cv::Point2d &_3D_point)
{

    Mat image_dist_point = Mat::zeros(1,1,CV_64FC2);
    image_dist_point.at<Vec2d>(0,0)[0] = image_point.x;
    image_dist_point.at<Vec2d>(0,0)[1] = image_point.y;
    Mat image_undist_point = Mat::zeros(1,1,CV_64FC2);
    Mat R_c2w;
    R_.copyTo(R_c2w);
    R_c2w = R_c2w.t();


    cout << image_dist_point.at<double>(0)<<" "<<image_dist_point.at<double>(1)<<endl;
    undistortPoints(image_dist_point, image_undist_point, K_,distCoeffs_, Mat::eye(3,3,CV_32FC1), K_);
    Mat image_undist_3_point = Mat::ones(3,1,CV_64FC1);
    image_undist_3_point.at<double>(0,0) = image_undist_point.at<Vec2d>(0,0)[0];
    image_undist_3_point.at<double>(1,0) = image_undist_point.at<Vec2d>(0,0)[1];
    Mat x_w = Mat::zeros(3,1,CV_64FC1);
    x_w = R_.t()*K_.inv()*image_undist_3_point;

    _3D_point.x = -x_w.at<double>(0,0)*cam_h_/x_w.at<double>(2,0) - translation_[0];
    _3D_point.y = -x_w.at<double>(1,0)*cam_h_/x_w.at<double>(2,0) - translation_[1];

}
void Calibrator::boards_pop_last_data()
{
    calib_boards_.pop_back();
}

