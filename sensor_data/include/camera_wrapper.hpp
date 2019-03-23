#ifndef CALIBRATIONTOOL_CAMERA_WRAPPER_HPP
#define CALIBRATIONTOOL_CAMERA_WRAPPER_HPP

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <unistd.h>

class CameraWrapper {
public:
    virtual bool Init(int index=0) = 0;
    virtual bool GetFrame(cv::Mat &frame) = 0;
    virtual void Stop() = 0;
    bool rot180_=false;
};

class USBCamera: public CameraWrapper {
public:
    bool Init(int index=0) {
        cam_.open(index);
        if (!cam_.isOpened()) {
            std::cerr << "Camera " << index << " open failed"<< std::endl;
            return false;
        }
        return true;
    }

    bool GetFrame(cv::Mat &frame) {
        bool rst = cam_.read(frame);

        if (rst && (!frame.empty()))
            if (rot180_)
                cv::flip(frame, frame, -1);

        return rst;
    }

    void Stop() {
        cam_.release();
    }
private:
    cv::VideoCapture cam_;
};

#ifdef WITH_FLYCAPTURE

/***
 *Author:       LiangChen
 *E-Mail:       chinandy@foxmail.com
 *Web:          www.chenliang.me
 *Version:      v1.0
 *Last Modified:2017-4-21 19:20:00
 *
 *Log:
 *      1.This file has been modified to adapt the hardware setup for momenta hdmap vehicle.
 */

#include <FlyCapture2.h>

#define SN_TEST_FOV50 16434663

using namespace FlyCapture2;

static inline void PrintError( Error error )
{
    error.PrintErrorTrace();
}

class PGCamera: public CameraWrapper {
public:
    PGCamera(): debug_(false) {};
    ~PGCamera() {
        Stop();
    }

    bool GetFrame(cv::Mat& frame) {
        long tick;
        return GetFrame(frame, tick);
    }

    bool GetFrame(cv::Mat& frame, long& tick) {
        Image rawImage, copyImage;
        TimeStamp timestamp;
        Error error;

        // Retrieve an image
        error = cam_.RetrieveBuffer( &rawImage );
        timestamp = rawImage.GetTimeStamp();
        tick = timestamp.seconds*1000 + timestamp.microSeconds/1000;

        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }
        rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &copyImage);
        cv::Mat img(copyImage.GetRows(), copyImage.GetCols(), CV_8UC3, copyImage.GetData());
        img.copyTo(frame);
        if (!frame.empty())
            if (rot180_)
             cv::flip(frame, frame, -1);
        return true;
    }

    void Stop() {
        Error error;
        error = cam_.StopCapture();
        // if (error != PGRERROR_OK)
        // {
        //     PrintError( error );
        //     return;
        // }
        error = cam_.Disconnect();
        // if (error != PGRERROR_OK)
        // {
        //     PrintError( error );
        //     return;
        // }
    }

    bool Init(int index = 0) {

        PGRGuid guid;
        Error error;
        BusManager busMgr;
        CameraInfo camInfo[10];
        unsigned int numCamInfo = 10;
        error = BusManager::DiscoverGigECameras( camInfo, &numCamInfo );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }
        if(debug_)
            std::cerr << "Number of cameras discovered: " << numCamInfo << std::endl;

        unsigned int numCameras;
        error = busMgr.GetNumOfCameras(&numCameras);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }
        if(debug_)
            std::cerr << "Number of cameras enumerated: " << numCameras << std::endl;

        if (use_serial_num) {
            error = busMgr.GetCameraFromSerialNumber(index, &guid);
        } else {
            error = busMgr.GetCameraFromIndex(index, &guid);
        }
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }
        InterfaceType interfaceType;
        error = busMgr.GetInterfaceTypeFromGuid( &guid, &interfaceType );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return false;
        }
        if ( interfaceType == INTERFACE_GIGE ) {
            error = cam_.Connect(&guid);
            if (error != PGRERROR_OK)
            {
                PrintError( error );
                return false;
            }

            CameraInfo camInfoAgain;
            error = cam_.GetCameraInfo(&camInfoAgain);
            if (error != PGRERROR_OK)
            {
                PrintError( error );
                return false;
            }

            std::cout << camInfoAgain.serialNumber << std::endl;
        }
        if(debug_)
            std::cerr << "Init finish." << std::endl;
        unsigned int numStreamChannels = 0;
        error = cam_.GetNumStreamChannels( &numStreamChannels );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }
        for (unsigned int i=0; i < numStreamChannels; i++)
        {
            GigEStreamChannel streamChannel;
            error = cam_.GetGigEStreamChannelInfo( i, &streamChannel );
            if (error != PGRERROR_OK)
            {
                PrintError( error );
                return false;
            }

            streamChannel.destinationIpAddress.octets[0] = 224;
            streamChannel.destinationIpAddress.octets[1] = 0;
            streamChannel.destinationIpAddress.octets[2] = 0;
            streamChannel.destinationIpAddress.octets[3] = 1;

            error = cam_.SetGigEStreamChannelInfo( i, &streamChannel );
            if (error != PGRERROR_OK)
            {
                PrintError( error );
                return false;
            }
        }

        // if (debug_)
            // std::cerr << "Querying GigE image setting information..." << std::endl;
        // GigEImageSettingsInfo imageSettingsInfo;
        // error = cam_.GetGigEImageSettingsInfo( &imageSettingsInfo );
        // if (error != PGRERROR_OK)
        // {
        //     PrintError( error );
        //     return false;
        // }
        //
        // GigEImageSettings imageSettings;
        // imageSettings.offsetX = 0;
        // imageSettings.offsetY = 0;
        // imageSettings.height = imageSettingsInfo.maxHeight;
        // imageSettings.width = imageSettingsInfo.maxWidth;
        // imageSettings.pixelFormat = PIXEL_FORMAT_RAW8;
        //
        // if (debug_)
        //     std::cerr << "Setting GigE image settings..." << std::endl;
        //
        // error = cam_.SetGigEImageSettings( &imageSettings );
        // if (error != PGRERROR_OK)
        // {
        //     PrintError( error );
        //     return false;
        // }
        if (debug_)
            std::cerr << "Starting image capture..." << std::endl;
        error = cam_.StartCapture();
        if (error != PGRERROR_OK) {
            PrintError( error );
            std::cout << "Cam" << index << " Init failed." << std::endl;
            return false;
        }
        return true;
    }

    bool debug_;
    bool use_serial_num = false;

private:
    FlyCapture2::GigECamera cam_;
};

#endif

class FakeCamera: public CameraWrapper {
public:
    bool GetFrame(cv::Mat& frame) {
        long tick;
        return GetFrame(frame, tick);
    }
    bool Init(int index = 0) {return true;};
    bool Init(const std::string prefix) {
        prefix_ = prefix;
        return true;
    }
    bool GetFrame(cv::Mat& frame, long& tick) {
        FILE *f = fopen((prefix_ + "/flag").c_str(), "r");
        //std::cout <<"err5" <<std::endl;
        if (f != NULL) {
            int flag = -1;
            int ret = fscanf(f, "%d", &flag);
            fclose(f);
            if (ret > 0) {
                sprintf(buf_, "%s/%d.bmp", prefix_.c_str(), flag);
                frame = cv::imread(buf_);
                return true;
            }
        }
        usleep(1000);
        return false;
    }
    void Stop() {};
private:
    std::string prefix_;
    char buf_[100];
};


#endif
