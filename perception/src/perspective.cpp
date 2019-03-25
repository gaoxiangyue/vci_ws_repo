#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
//
#include <thread>
#include "Common.hpp"
#include "perception_node.hpp"
#include "cvui.h"

using namespace cv;
using namespace std;

int main( )
{
	Mat img=imread("./KITTI/2011_09_26/2011_09_26_drive_0009_sync/image_00/data/0000000131.png");
    if( !img.data )
	{ std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    imshow("img",img);

	int img_height = img.rows;
	int img_width = img.cols;
	vector<Point2f> corners(4);
	corners[0] = Point2f(530,200);
	corners[1] = Point2f(650,200);
	corners[2] = Point2f(100,img_height-1);
	corners[3] = Point2f(900,img_height-1);
	vector<Point2f> corners_trans(4);
	corners_trans[0] = Point2f(530,0);
	corners_trans[1] = Point2f(650,0);
	corners_trans[2] = Point2f(530,img_height-1);
	corners_trans[3] = Point2f(650,img_height-1);
 
	Mat transform = getPerspectiveTransform(corners_trans,corners);
	cout<<transform<<endl;
	vector<Point2f> points, points_trans;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			points_trans.push_back(Point2f(j,i));
		}
	}
    
	perspectiveTransform( points_trans, points, transform);
    
	Mat img_trans = Mat::zeros(img_height,img_width,CV_8UC3);

	int count = 0;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			int y = points[count].y;
			int x = points[count].x;
            int ty = points_trans[count].y;
			int tx = points_trans[count].x;
            if(y <0 || y >= img_height || x <0 || x >= img_width){}
            else{
                uchar* t = img_trans.ptr<uchar>(ty);
                uchar* p = img.ptr<uchar>(y);
                t[tx*3]  = p[x*3];
                t[tx*3+1]  = p[x*3+1];
                t[tx*3+2]  = p[x*3+2];
            }
			count++;
		}
	}
	//imwrite("boy_trans.png",img_trans);
    imshow("img_trans",img_trans);
    waitKey();
	return 0;
}