#ifndef MODEL_HPP
#define MODEL_HPP

#include <opencv2/core/core.hpp>

#include "image/image.hpp"
#include "ca/caparams.hpp"
#include "camera/intrinsics.hpp"

namespace ce {


class Model
{
    friend class CalibrationController;

protected:
    ImageList imageList;

    cv::Size imageSize() {
        if(imageList.empty())
        {
            if ((cameraInfo.sensorWidthPx < 0) || (cameraInfo.sensorHeightPx < 0))
                return cv::Size(-1, -1);
            else
                return cv::Size(cameraInfo.sensorWidthPx, cameraInfo.sensorHeightPx);
        }

        return cv::Size(imageList.front()->width(), imageList.front()->height());
    }

    ImageSet  validImageSet;
    ImageSet  calibImageSet;
    ImageSet  erchkImageSet;

    CalibrationAlgorithmParams caParams;

    Intrinsics intrinsics;
    CameraInfo cameraInfo;
    CalibrationInfo calibrationInfo;
    cv::Mat undistortMap;
};


}

#endif // MODEL_HPP
