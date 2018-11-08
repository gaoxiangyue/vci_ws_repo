#ifndef RECTILINEAR_CALIBRATION_ALGORITHM_HPP
#define RECTILINEAR_CALIBRATION_ALGORITHM_HPP

#include "ca/ca.hpp"

namespace ce {


class FisheyeOpenCvCalibrationAlgorithm : public OpenCvCalibrationAlgorithm
{
public:

    FisheyeOpenCvCalibrationAlgorithm(std::shared_ptr<Logger> logger);

    virtual std::string cameraTypeString() override;
    virtual ECameraType cameraType() override;
    virtual bool validateIntrinsics(const Intrinsics &intrinsics);

    //for correct FOV, intrinsics should be computed with Cogent OpenCV version (fisheye::calibrate() >180 degree fov support)

    //TODO: reimplement this with shading-aware code
    virtual double computeDFOV(const Intrinsics &intrinsics, const cv::Size &imageSize) override;

protected:
    virtual double estimateTheta(const Intrinsics &intrinsics, cv::Point2f direction, bool& wasExtrapolated) override;

    virtual double openCvCalibrate(const std::vector<std::vector<cv::Point3f>> &objectsFeatures,
                                   const std::vector<std::vector<cv::Point2f>> &imagesFeatures,
                                   const cv::Size &imageSize,
                                   cv::Mat &K,
                                   cv::Mat &D,
                                   cv::OutputArrayOfArrays &rvecs,
                                   cv::OutputArrayOfArrays &tvecs);
    virtual void openCvProjectPoints(const std::vector<cv::Point3f> &objectFeatures,
                                     const cv::Mat &rvec, const cv::Mat &tvec,
                                     const cv::Mat &K, const cv::Mat &D,
                                     std::vector<cv::Point2f> &featuresRep);
    virtual void openCvUndistortPoints(const std::vector<cv::Point2f> &features,
                                       const cv::Mat &K, const cv::Mat &D,
                                       std::vector<cv::Point2f> &featuresUn);
    virtual void openCvUndistortSpheric(const std::vector<cv::Point2f> &features,
                                        const cv::Mat &K, const cv::Mat &D,
                                        std::vector<cv::Point2f> &featuresUn) override;
    virtual void openCvDistortSpheric(const std::vector<cv::Point2f> &featuresUn,
                                      const cv::Mat &K, const cv::Mat &D,
                                      std::vector<cv::Point2f> &features) override;
    virtual void openCvInitUndistortRectifyMap(const cv::Mat &K, const cv::Mat &D,
                                               const cv::Size &imageSize,
                                               cv::Mat &mapX, cv::Mat &mapY);
};


}

#endif //RECTILINEAR_CALIBRATION_ALGORITHM_HPP
