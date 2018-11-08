#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>			// C++
#include "opencv2/imgproc/imgproc_c.h"	// C
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

#include <gflags/gflags.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <fstream>
#include <thread>
#include <csignal>

#include "ccamera/ccamera.h"

using namespace std;
using namespace cv;


// ... live camera calibration
/* exit codes */
#define STATUS_OK       0  /* successful completion */
#define STATUS_NODEV    1  /* VGA2USB device not found */
#define STATUS_VMERR    2  /* Video mode detection failure */
#define STATUS_NOSIGNAL 3  /* No signal detected */
#define STATUS_GRABERR  4  /* Capture error */
#define STATUS_IOERR    5  /* File save error */
#define STATUS_CMDLINE  6  /* Command line syntax error */
#define STATUS_TERM     7  /* Program terminated */


/// ... google flags
//DEFINE_string(input_type, "video", "The type of calibration input. Currently support 'images', 'video', 'usbCam', and 'cogent'.");
//DEFINE_string(mac_Cam, "00:04:9f:00:4a:67", "the MAC address of camera.");
//DEFINE_int32(usbCam_idx, 0, "the camera index for usbCamera.");
//DEFINE_int32(log_idx, 0, "the log sequence index for usbCamera.");
//DEFINE_string(vid_fname, "./", "Calibration video filename.");
//DEFINE_string(imgs_path, "./", "Calibration images folder.");
//DEFINE_string(calib_fname, "../models/calib_02.yaml", "Calibration filename.");
//DEFINE_int32(calib_model, 1, "The calibration camera model:  =0: wide-angle; =1: fisheye; =-1: wide-angle but convert to fisheye");

const string input_type = "cogent";
const string mac_Cam = "00:04:9f:00:4c:20";
const int usbCam_idx = 0;
const int log_idx = 0;

const string& vid_fname = "/home/vci/Videos/car_9.avi";
const string& imgs_path = "./";
const string& calib_fname = "../models/calib_02.yaml";
const int calib_model = 1;

//save video params
bool if_record = true;
cv::VideoWriter w;
string outfile = "/home/vci/Videos/car1016.avi";


//camera ID
int idCam = -1;
//camera options - fixed for the moment
const char *caps = "";
const char *bufferType = "BGR";
int flags = 0;
int width = 1280;
int height = 800;	//720;	//1080;	//
float frameRate = 30;	//15;
float confid_thresh = 0.3f;	//0.6f;


const char *windowResultView = "Processed View";

void printUsage(const char *nameProgram) {
	printf("usage: %s \n"
			"          macAddressCamera \n"
			"          calibPattern_width_N \n"
			"          calibPattern_height_N \n"
			"          calibPattern_squareSize(mm) \n"
			"          calibCameraModel(0: regular; 1: fisheye; -1: regular but conv-to fisheye) \n", nameProgram);
	printf("\n");
}

void signalHandler(int signal) {
	switch (signal) {
	case SIGTERM:
	case SIGINT:
		if (idCam)
			camera_close(idCam);
		exit(STATUS_TERM);
		break;
	default:
		if (idCam)
			camera_close(idCam);
		exit(STATUS_TERM);
		break;
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/usb_cam/image_raw", 1);

	gflags::SetUsageMessage("Do camera intrinsic calibration.\n"
			"Usage:\n"
			"    calib_intrinsic [FLAGS  --input_type=camera --mac_Cam=00:04:9f:00:4a:67 --calib_model=1] \n");
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	

	cv::VideoCapture cap;	//, cap1;

	cv::String filename;	//(mac_Cam);
	std::size_t found;
	if (input_type == "cogent") {
		// --- Camera I/O setting
		//... set up signal handler
		signal(SIGINT, signalHandler);
		signal(SIGILL, signalHandler);
		signal(SIGSEGV, signalHandler);
		signal(SIGTERM, signalHandler);
		//... open camera
		idCam = camera_open_with_args(mac_Cam.c_str(), caps, flags, bufferType,
				width, height, frameRate);
		//... error in opening camera
		if (idCam < 0 ) {
			printf("error in opening camera\n");
			exit(STATUS_NOSIGNAL);
		}

		/// log path
		filename += mac_Cam;
		found = filename.find_first_of(':');
		while (found!=std::string::npos)
		{
			found=filename.find_first_of(':',found+1);
		}
	}
	else if(input_type == "usbCam"){
		cap.open(usbCam_idx);
		if (!cap.isOpened()) {
			//			LOG(FATAL)<< "Failed to open video: " << vid_fname;
		}
		/// log path
		filename += "../video/usbCamera";
	}
	else if (input_type == "video") {
		cap.open(vid_fname);
		if (!cap.isOpened()) {
		}

		/// log path
		filename += vid_fname;
		found = filename.find(".avi");	//.find_first_of('.');
		filename = filename.substr(0, found);
	}
	else if (input_type == "images") {
		//		imgs_path >> file;
	}

	cout << "pass open camera or video! " << endl;
	/// log path
	const char* log_path = filename.c_str();

	// --- Snapshot images for logging
	int frame_cnt = 0;

	// ... images Mats
	cv::Mat img_in;

 // cv::Mat image = cv::imread("/home/ros/catkin_ws/src/darknet_ros/doc/quadruped_anymal_and_person.JPG", CV_LOAD_IMAGE_COLOR);
 // if(image.empty()){
 //  printf("open error\n");
 //  }
  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(30);



 if(if_record)
     w.open(outfile,CV_FOURCC('D', 'I', 'V', 'X'),30,Size(1280,800),true);





  while (nh.ok()) {
    if (input_type == "cogent") {
	camera_buf_t *bufferCam = NULL;
	bufferCam = camera_get_buffer(::idCam, 14000);
	if (bufferCam && bufferCam->data) {
		cv::Mat frameCam(::height, ::width, CV_8UC3, (void*) (bufferCam->data));
		img_in = frameCam.clone();	
		camera_release_buffer(bufferCam);
		frame_cnt ++;
        if(if_record)
           {
		w.write(img_in);
		waitKey(40);
           }
	}
	else{
		cout << "extract frame failed! " <<endl;
		waitKey(150);
		continue;
	}
    }else if ( (input_type == "video")||(input_type == "usbCam") ){
	Mat frame;
	bool success = cap.read(frame);
	if (!success) {
                cout << "open err! " << endl;
		break;
	}
	img_in = frame.clone();
	frame_cnt ++;
    }else if (input_type == "images") {
	}

	//		if(frame_cnt % 3 != 0){
	//			continue;
	//		}

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_in).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (input_type == "cogent") {
	// --- close camera
	camera_close(idCam);
     if(if_record)
        w.release();
  }else if ( (input_type == "video") || (input_type == "usbCam") ){
    if (cap.isOpened()) {
	  cap.release();
    }
  }
}
