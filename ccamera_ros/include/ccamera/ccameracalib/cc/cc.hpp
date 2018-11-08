#ifndef CALIBRATIONCONTROLLER_H
#define CALIBRATIONCONTROLLER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <functional>

#include "ccm.hpp"
#include "logger/logger.hpp"

#include "fg/fg.hpp"
#include "fg/vfg.hpp"

#include "ca/ca.hpp"
#include "ca/fisheyeca.hpp"

#include "model/model.hpp"

namespace ce {

//http://stackoverflow.com/questions/11711034/stdshared-ptr-of-this
class CalibrationController
{
public:
    CalibrationController();
    CalibrationController(std::shared_ptr<Logger> logger, std::shared_ptr<FrameGrabber> fg, std::shared_ptr<OpenCvCalibrationAlgorithm> ca);
    CalibrationController(std::shared_ptr<Logger> logger, std::shared_ptr<FrameGrabber> fg, std::shared_ptr<OpenCvCalibrationAlgorithm> ca, std::shared_ptr<VideoFrameGrabber> vfg);
    ~CalibrationController() { join(); }

    std::function<bool()> running;
    std::function<void()> interrupt;

    void setLogger(std::shared_ptr<Logger> logger);
    void setFrameGrabber(std::shared_ptr<FrameGrabber> fg);
    void setVideoFrameGrabber(std::shared_ptr<VideoFrameGrabber> vfg);
    void setCalibrationAlgorithm(std::shared_ptr<OpenCvCalibrationAlgorithm> ca);

    int startVideoStreaming(const std::string &dirPath, int width, int height, int bitDepth, std::function<void()> callback = []{}, int* const code = nullptr); //procedure

    int startImagesFromListLoading(const std::vector<std::string> &filepaths, std::function<void()> callback = []{}, int* const code = nullptr);
    int startImagesLoading(const std::string &dirPath, std::function<void()> callback = []{}, int* const code = nullptr); //procedure
    void saveImages(const std::string &path);
    ImageList getImages();

    int startFeaturesExtraction(std::function<void()> callback = []{}, int* const code = nullptr); //procedure
    int calibrateIntrinsics(std::function<void()> callback = []{}, int* const code = nullptr); //procedure, updates model intrinsics
    int computeReprojectionError();
    int computeReconstructionError();
    int checkDistortionQuality();

    void setUseQualityFilter(bool useQF);

    bool updateCAParams(const CalibrationAlgorithmParams &caParams);
    CalibrationAlgorithmParams getCAParams();

    bool updateIntrinsics(const Intrinsics &intrinsics);
    Intrinsics getIntrinsics();
    bool loadIntrinsics(std::string path);
    bool saveIntrinsics(std::string path, std::string uuid);

    std::vector<std::string> getModelInfoFieldNames();
    void setModelInfo(std::map<std::string, std::string> infoMap);
    std::map<std::string, std::string> getModelInfo();

    double getReprojectionError();
    double getCalculatedReprojectionError();
    double getMaxReprojectionError();

    double getReconstructionError();
    double getMaxReconstructionError();

    void selectAllImagesForCalibration();
    void selectAllImagesForErrorChecking();

    void addImage(std::shared_ptr<Image> image);
    void rmImage(int id);

    void addCalibImage(int id);
    void rmCalibImage(int id);
    bool isCalibImage(int id);

    void addErchkImage(int id);
    void rmErchkImage(int id);
    bool isErchkImage(int id);

    bool createUndistortMap();
    cv::Mat getUndistortMap();

    void reprojectFeatures(std::shared_ptr<Image> image);
    void undistortFeatures(std::shared_ptr<Image> image);

    double getHFOV();
    double getVFOV();
    double getDFOV();

    cv::Size imageSize();
    std::string getTimestamp();
    std::string getCalibrationQualityEst();

    void join(); //wait for procedures finished

protected:
    std::shared_ptr<Logger> mLogger;

    std::shared_ptr<FrameGrabber> mFG;
    std::shared_ptr<OpenCvCalibrationAlgorithm> mCA;
    std::shared_ptr<VideoFrameGrabber> mVFG;
    std::shared_ptr<Model> mModel;

private:
    bool bind(std::shared_ptr<CalibrationControllerModule> ccm, std::function<void()> callback = []{}, int* const code = nullptr);

    std::thread mThread;
    std::mutex mMutex;
};


}

#endif // CALIBRATIONCONTROLLER_H
