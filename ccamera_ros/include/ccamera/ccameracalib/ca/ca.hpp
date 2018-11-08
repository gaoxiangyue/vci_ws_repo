#ifndef CALIBRATION_ALGORITHM_HPP
#define CALIBRATION_ALGORITHM_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "image/image.hpp"
#include "cc/ccm.hpp"
#include "logger/logger.hpp"
#include "ca/caparams.hpp"
#include "camera/intrinsics.hpp"

namespace ce {

class OpenCvCalibrationAlgorithm : public CalibrationControllerModule
{
public:

    OpenCvCalibrationAlgorithm();
    OpenCvCalibrationAlgorithm(std::shared_ptr<Logger> logger);
    ~OpenCvCalibrationAlgorithm() {}

    int extractFeatures(const ImageList &imageList, const CalibrationAlgorithmParams &params, ImageSet &validImages);
    int calibrateIntrinsics(
            const ImageSet &calibImageSet,
            const CalibrationAlgorithmParams &params,
            Intrinsics &intrinsics,
            double &reprojectionError,
            double &calculatedReprojectionError,
            double &maxReprojectionError,
            std::string &calibrationTimestamp,
            std::string &calibrationQualityEst);
    int computeReprojectionError(
            const ImageSet &erchkImageSet,
            const CalibrationAlgorithmParams &params,
            const Intrinsics &intrinsics,
            double &reprojectionError,
            double &maxReprojectionError
            );

    int computeReconstructionError(
            const ImageSet &erchkImageSet,
            const CalibrationAlgorithmParams &params,
            const Intrinsics &intrinsics,
            double &reprojectionError,
            double &maxReprojectionError
            );

    int checkDistortionQuality(
            const ImageSet &erchkImageSet,
            const CalibrationAlgorithmParams &params,
            const Intrinsics &intrinsics,
            double &reprojectionError,
            double &maxReprojectionError
            );
    bool createUndistortMap(const Intrinsics &intrinsics, const cv::Size &imageSize, cv::Mat &undistortMap);

    int reprojectFeatures(const std::vector<cv::Point2f> &features,
                          const CalibrationAlgorithmParams &params,
                          const Intrinsics &intrinsics,
                          std::vector<cv::Point2f> &featuresRep);
    bool undistortFeatures(const std::vector<cv::Point2f> &features,
                           const Intrinsics &intrinsics,
                           std::vector<cv::Point2f> &featuresUn);
    bool undistortSpheric(const std::vector<cv::Point2f> &features,
                          const Intrinsics &intrinsics,
                          std::vector<cv::Point2f> &featuresUn);
    bool distortSpheric(const std::vector<cv::Point2f> &features,
                          const Intrinsics &intrinsics,
                          std::vector<cv::Point2f> &featuresUn);
    bool validateParams(const CalibrationAlgorithmParams &caParams);

    int checkCellSize(const ImageSet &qchkImageSet, ImageSet &newImageSet, const CalibrationAlgorithmParams &params);
    int checkFeatureContrast(const ImageSet &qchkImageSet, ImageSet &newImageSet, const CalibrationAlgorithmParams &params);
    int evaluateCoverage(const ImageList &qchkImageList, const Intrinsics &intrinsics, const CalibrationAlgorithmParams &params, double& coverage);
    int evaluateCoverageRectangle(const ImageList &qchkImageList, const Intrinsics &intrinsics, double& coverage);
    int evaluateCoverageSpherical(const ImageList &qchkImageList, const Intrinsics &intrinsics, double& coverage);
    int rateCalibrationDataset(const ImageList &qchkImageList, const Intrinsics &intrinsics, const CalibrationAlgorithmParams &params, double calibrationError, double &quality);
    void initHistogramPointGrid(std::shared_ptr<CoverageReport> report, const Intrinsics &intrinsics, cv::Size imgSize);

    virtual ECameraType cameraType();
    virtual std::string cameraTypeString();
    virtual bool validateIntrinsics(const Intrinsics &intrinsics);

    virtual double computeVFOV(const Intrinsics &intrinsics, const cv::Size &imageSize);
    virtual double computeHFOV(const Intrinsics &intrinsics, const cv::Size &imageSize);
    virtual double computeDFOV(const Intrinsics &intrinsics, const cv::Size &imageSize);

protected:
    void undistortedToSpherical(const std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& angles);
    void undistortedToSpherical(const std::vector<cv::Point3f>& points3d, std::vector<cv::Point2f>& angles);
    void sphericalToUndistorted(const std::vector<cv::Point2f>& angles, std::vector<cv::Point3f>& points);

    int compute3dError(const std::vector<cv::Point2f> &features,
                             const CalibrationAlgorithmParams &params,
                             const Intrinsics &intrinsics,
                             double& error);

    bool validateCameraMatrix(
            const ImageSet &calibImageSet,
            const Intrinsics &intrinsics,
            const CalibrationAlgorithmParams &params
            );

    double distance(cv::Point2f p1, cv::Point2f p2);
    virtual int computeValidFOV(const cv::Point2f boundaryPoint, const Intrinsics &intrinsics, double& fov);
    virtual int computeValidFOV(const ImageList &qchkImageList, const Intrinsics &intrinsics, double& fov);
    virtual double estimateTheta(const Intrinsics &intrinsics, cv::Point2f direction, bool& wasExtrapolated);
    virtual double computeTheta(const Intrinsics &intrinsics, cv::Point2f point);
    virtual double computeFOV(const Intrinsics &intrinsics, std::vector<cv::Point2f> boundaries, const cv::Size &imageSize);

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
                                        std::vector<cv::Point2f> &featuresUn);
    virtual void openCvDistortSpheric(const std::vector<cv::Point2f> &featuresUn,
                                      const cv::Mat &K, const cv::Mat &D,
                                      std::vector<cv::Point2f> &features);

    virtual void openCvInitUndistortRectifyMap(const cv::Mat &K, const cv::Mat &D,
                                               const cv::Size &imageSize,
                                               cv::Mat &mapX, cv::Mat &mapY);

    virtual bool solvePnP( const std::vector<cv::Point3f>& objectPoints,
                           const std::vector<cv::Point2f>& imagePointsDist,
                           const Intrinsics& intrinsics,
                           cv::Mat& rvec, cv::Mat& tvec);

    std::shared_ptr<Logger> mLogger;

    std::vector<std::vector<cv::Point3f>> generateObjectsFeatures(size_t size, uint width, uint height, double cellSize);
    std::vector<cv::Point3f> generatePatternObjectFeatures(uint width, uint height, double cellSize);

    /**
     * @brief Determine the appropriate corner refinement area.
     */
    cv::Size optimalRefinementArea(const std::shared_ptr<Image> image);
};


}

#endif // CALIBRATION_ALGORITHM_HPP
