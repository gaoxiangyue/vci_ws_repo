#ifndef FIND_CORNERS_H
#define FIND_CORNERS_H

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using std::vector;
using namespace std;
using namespace cv;

class FindCorners
{
public:
	FindCorners();
	~FindCorners();

public:
	void detectCorners(Mat &gray, vector<Point2f> &resultCornors,float scoreThreshold);

private:
	float normpdf(float dist, float mu, float sigma);

	void getMin(Mat src1, Mat src2, Mat &dst);

	void getMax(Mat src1, Mat src2, Mat &dst);

	void getImageAngleAndWeight(Mat img, Mat &imgDu, Mat &imgDv, Mat &imgAngle, Mat &imgWeight);

	void edgeOrientations(Mat imgAngle, Mat imgWeight,int index);

	void findModesMeanShift(vector<float> hist, vector<float> &hist_smoothed, vector<pair<float, int>> &modes, float sigma);

	//score corners
	void scoreCorners(Mat img, Mat imgAngle, Mat imgWeight, vector<Point2f> &cornors, vector<int> radius, vector<float> &score);

	//compute corner statistics
	void cornerCorrelationScore(Mat img, Mat imgWeight, vector<Point2f> cornersEdge, float &score);

	void refineCorners(vector<Point2f> &cornors, Mat imgDu, Mat imgDv, Mat imgAngle, Mat imgWeight, float radius);

	void createkernel(float angle1, float angle2, int kernelSize, Mat &kernelA, Mat &kernelB, Mat &kernelC, Mat &kernelD);

	void nonMaximumSuppression(Mat& inputCorners, vector<Point2f>& outputCorners, float threshold, int margin, int patchSize);

private:
	vector<Point2f> templateProps;
	vector<int> radius;
	vector<Point2f> cornerPoints;
	std::vector<std::vector<float> > cornersEdge1;
	std::vector<std::vector<float> > cornersEdge2;

};



#endif // FIND_CORNERS_H
