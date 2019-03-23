#ifndef DRAW_CALIB_BOARD_HPP
#define DRAW_CALIB_BOARD_HPP

#include <opencv2/opencv.hpp>
#include <vector>

void draw_reprojection(cv::Mat &img_roi,
                       cv::Mat &img_show,
                       std::vector<cv::Point2f> &mid_line,
                       std::vector<cv::Point2f> &reproj_pts) {
    // show result
    cv::Size img_show_size(1280, 720);
    cv::resize(img_roi, img_show, img_show_size);

    float hscale = float(img_roi.size().height) / float(img_show_size.height);
    float wscale = float(img_roi.size().width) / float(img_show_size.width);

    // Draw the corners.
    cv::Scalar imagePointColor(0, 0, 255);
    cv::Scalar reprojPointColor(0, 255, 0);

    for (int i=0; i < int(mid_line.size()); i++) {
        cv::Point2f p = cv::Point2f(mid_line[i].x / wscale, mid_line[i].y / hscale);
        cv::circle(img_show, p, 4, imagePointColor, 2);
        // check imagePoints index
        cv::putText(img_show, std::string(" ") + std::to_string(i),
                    p, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255));
    }

    for (auto &reproj_pt : reproj_pts) {
        cv::circle(img_show, cv::Point2f(reproj_pt.x / wscale, reproj_pt.y / hscale),
                   3, reprojPointColor, -1);
    }
}


void draw_match_mid_pts(std::vector<cv::Mat> &imgs,
                        cv::Mat &img_show,
                        std::vector<CalibBoard> &calib_boards) {
    cv::Size img_show_size(1280, 720);
    std::vector<cv::Mat> img_shows(3);
    for (int i=0; i<3; i++) {
        cv::Mat grey;
        cvtColor(imgs[i], grey, CV_BGR2GRAY);
        switch (i) {
            case 0:
                grey.convertTo(grey, -1, 1.8, -80);
                break;
            case 1:
                grey.convertTo(grey, -1, 1.0, -40);
                break;
            case 2:
                grey.convertTo(grey, -1, 1.4, -60);
                break;

            default:break;
        }
        cv::resize(grey, img_shows[i], img_show_size);
    }

    cv::merge(img_shows, img_show);

    float hscale = float(imgs[0].size().height) / float(img_show_size.height);
    float wscale = float(imgs[0].size().width) / float(img_show_size.width);

    // Draw the corners.
    // drawChessboardCorners(img_show, boardSize, cv::Mat(imagePoints), found );
    cv::Scalar imagePointColor(255, 255, 255);

    for (int mi = 0; mi < int(calib_boards.size()); mi++) {
        auto & mid_line = calib_boards[mi].mid_line_;
        auto & code_idx = calib_boards[mi].code_idx_;
        for (int i=0; i < int(calib_boards[0].mid_line_.size()); i++) {
            cv::Point2f p = cv::Point2f(mid_line[i].x / wscale, //+ mi * img_show_size.width,
                                        mid_line[i].y / hscale);

            cv::circle(img_show, p, 2, imagePointColor, -1);
            // // check imagePoints index
            cv::putText(img_show, std::string(" ") + std::to_string(code_idx[i]),
                        p, cv::FONT_HERSHEY_SIMPLEX, 0.4, imagePointColor);
        }
    }
}

#endif // DRAW_CALIB_BOARD_HPP
