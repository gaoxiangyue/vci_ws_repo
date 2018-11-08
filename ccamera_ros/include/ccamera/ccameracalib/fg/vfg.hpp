#ifndef VIDEO_FRAME_GRABBER_HPP
#define VIDEO_FRAME_GRABBER_HPP

#include "cc/ccm.hpp"
#include "logger/logger.hpp"
#include "image/image.hpp"

#ifndef WITH_CCAMERA
    struct camera_buf_t {
        void *data;
    };
#else
#include <ccamera.h>
#endif

namespace ce {

class VideoFrameHandler{
public:
    virtual void onFrame(std::shared_ptr<camera_buf_t> buffer) = 0;
};

//CameraFrameGrabber
class VideoFrameGrabber : public CalibrationControllerModule
{
public:
    VideoFrameGrabber();

#ifdef WITH_CCAMERA
    VideoFrameGrabber(std::shared_ptr<Logger> logger);

    bool validateMAC(const std::string &mac);

    int startVideoStreaming(const std::string &mac, int width, int height, int bitDepth);
#else //mock functions implementation
    VideoFrameGrabber(std::shared_ptr<Logger> logger) {}

    bool validateMAC(const std::string &mac) { return false; }

    int startVideoStreaming(const std::string &mac, int width, int height, int bitDepth) { return  -1; }
#endif

    void addVideoFrameHandler(std::shared_ptr<VideoFrameHandler> h)
    {
        mVFHs.push_back(h);
    }

protected:
    std::shared_ptr<Logger> mLogger;

    std::vector<std::shared_ptr<VideoFrameHandler>> mVFHs;
};


}
#endif // VIDEO_FRAME_GRABBER_HPP
