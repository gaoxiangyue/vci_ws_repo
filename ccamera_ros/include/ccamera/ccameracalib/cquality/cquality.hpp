#ifndef CQUALITY_H
#define CQUALITY_H

#include <opencv2/core.hpp>

namespace ce
{
    void estimateLine(const std::vector<cv::Point2f>& points, std::vector<double>& linecoeffs);

    void intersectLines(const std::vector<std::vector<double>>& lines, cv::Point3f& result);

    bool estimateCameraMatrix(
            const std::vector<cv::Mat>& homographies,
            cv::Mat& estimatedK,
            double const * fx = nullptr,
            double const * fy = nullptr,
            double const * cx = nullptr,
            double const * cy = nullptr
            );
}

#endif //CQUALITY_H
