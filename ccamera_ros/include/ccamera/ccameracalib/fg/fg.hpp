#ifndef FRAME_GRABBER_HPP
#define FRAME_GRABBER_HPP

#include "cc/ccm.hpp"
#include "logger/logger.hpp"
#include "image/image.hpp"

namespace ce {

//DirectoryFrameGrabber
class FrameGrabber : public CalibrationControllerModule
{
public:
    FrameGrabber();
    FrameGrabber(std::shared_ptr<Logger> logger);
    ~FrameGrabber() {}

    virtual int loadImageList(const std::vector<std::string>& fileList, ImageList &imageList);
    virtual int loadImages(const std::string &dirPath, ImageList &imageList);
    virtual int loadImagesFromList(const std::vector<std::string>& fileList, ImageList &imageList);

protected:
    std::shared_ptr<Logger> mLogger;
};


}

#endif // FRAME_GRABBER_HPP
