#ifndef INTRINSICS
#define INTRINSICS

#include <sstream>
#include <string>

#include <cereal/cereal.hpp>

#include "io/cvmat.hpp"

namespace ce {

struct CameraInfo
{
    CameraInfo() :
        sensorWidthPx(-1),
        sensorHeightPx(-1),
        pixelSizeMM(-1)
    {}

    std::string cameraModel;
    std::string serialNumber;
    std::string lensInfo;
    std::string firmwareVersion;
    std::string OtpId;

    int sensorWidthPx;
    int sensorHeightPx;
    double pixelSizeMM;
};

struct CalibrationInfo
{
    CalibrationInfo()
    {
        reprojectionError = -1;
        calculatedReprojectionError = -1;
        maxReprojectionError = -1;

        reconstructionError = -1;
        maxReconstructionError = -1;

        distortionError = -1;
        maxDistortionError = -1;
    }

    std::string quality;
    std::string timestamp;
    std::string id;

    double reprojectionError;
    double calculatedReprojectionError;
    double maxReprojectionError;

    double reconstructionError;
    double maxReconstructionError;

    double distortionError;
    double maxDistortionError;
};

struct Intrinsics
{
    Intrinsics() :
        K(0, 0, CV_64F),
        D(0, 0, CV_64F)
    {}

    cv::Mat K;
    cv::Mat D;

    //for debug
    std::string print() const
    {
        std::ostringstream oss;

        oss << "K: " << K << std::endl;

        oss << "D: " << D << std::endl;

        return oss.str();
    }

private:
    friend class cereal::access;

    template<class Archive>
    void serialize(Archive &ar)
    {
        ar( cereal::make_nvp("CameraMatrix", K),
            cereal::make_nvp("DistortionCoeffs", D));
    }
};

}

#endif //INTRINSICS
