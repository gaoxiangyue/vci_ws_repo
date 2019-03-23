#include "FindCorners.h"

FindCorners::FindCorners()
{
	radius.push_back(4);
	radius.push_back(8);
	radius.push_back(12);
	templateProps.push_back(Point2f((float)0, (float)CV_PI / 2));
	templateProps.push_back(Point2f((float)CV_PI / 4, (float)-CV_PI / 4));
	templateProps.push_back(Point2f((float)0, (float)CV_PI / 2));
	templateProps.push_back(Point2f((float)CV_PI / 4, (float)-CV_PI / 4));
	templateProps.push_back(Point2f((float)0, (float)CV_PI / 2));
	templateProps.push_back(Point2f((float)CV_PI / 4, (float)-CV_PI / 4));
}
FindCorners::~FindCorners()
{}


//正态分布
float FindCorners::normpdf(float dist, float mu, float sigma){
	return exp(-0.5*(dist - mu)*(dist - mu) / (sigma*sigma)) / (std::sqrt(2 * CV_PI)*sigma);
}

//**************************生成核*****************************//
//angle代表核类型：45度核和90度核
//kernelSize代表核大小（最终生成的核的大小为kernelSize*2+1）
//kernelA...kernelD是生成的核
//*************************************************************************//
void FindCorners::createkernel(float angle1, float angle2, int kernelSize, Mat &kernelA, Mat &kernelB, Mat &kernelC, Mat &kernelD){

	int width = (int)kernelSize * 2 + 1;
	int height = (int)kernelSize * 2 + 1;
	kernelA = cv::Mat::zeros(height, width, CV_32F);
	kernelB = cv::Mat::zeros(height, width, CV_32F);
	kernelC = cv::Mat::zeros(height, width, CV_32F);
	kernelD = cv::Mat::zeros(height, width, CV_32F);

	for (int u = 0; u<width; ++u){
		for (int v = 0; v<height; ++v){
			float vec[2];
            vec[0] = u - kernelSize;
            vec[1] = v - kernelSize;//相当于将坐标原点移动到核中心
			float dis = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);//相当于计算到中心的距离
			float side1 = vec[0] * (-sin(angle1)) + vec[1] * cos(angle1);//相当于将坐标原点移动后的核进行旋转，以此产生四种核
			float side2 = vec[0] * (-sin(angle2)) + vec[1] * cos(angle2);//X=X0*cos+Y0*sin;Y=Y0*cos-X0*sin
			if (side1 <= -0.1&&side2 <= -0.1){
				kernelA.ptr<float>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
			if (side1 >= 0.1&&side2 >= 0.1){
				kernelB.ptr<float>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
			if (side1 <= -0.1&&side2 >= 0.1){
				kernelC.ptr<float>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
			if (side1 >= 0.1&&side2 <= -0.1){
				kernelD.ptr<float>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
		}
	}
	//std::cout << "kernelA:" << kernelA << endl << "kernelB:" << kernelB << endl
	//	<< "kernelC:" << kernelC<< endl << "kernelD:" << kernelD << endl;

	kernelA = kernelA / cv::sum(kernelA)[0];
	kernelB = kernelB / cv::sum(kernelB)[0];
	kernelC = kernelC / cv::sum(kernelC)[0];
	kernelD = kernelD / cv::sum(kernelD)[0];

}

void FindCorners::getMin(Mat src1, Mat src2, Mat &dst){
	int rowsLeft = src1.rows;
	int colsLeft = src1.cols;
	int rowsRight = src2.rows;
	int colsRight = src2.cols;
	if (rowsLeft != rowsRight || colsLeft != colsRight)return;

	int channels = src1.channels();

	int nr = rowsLeft;
	int nc = colsLeft;
	if (src1.isContinuous()){
		nc = nc*nr;
		nr = 1;
		//std::cout<<"continue"<<std::endl;
	}
	for (int i = 0; i<nr; i++){
		const float* dataLeft = src1.ptr<float>(i);
		const float* dataRight = src2.ptr<float>(i);
		float* dataResult = dst.ptr<float>(i);
		for (int j = 0; j<nc*channels; ++j){
			dataResult[j] = (dataLeft[j]<dataRight[j]) ? dataLeft[j] : dataRight[j];
		}
	}
}
//**************************//获取最大值*****************************//
//*************************************************************************//
void FindCorners::getMax(Mat src1, Mat src2, Mat &dst){

	int rowsLeft = src1.rows;
	int colsLeft = src1.cols;
	int rowsRight = src2.rows;
	int colsRight = src2.cols;
	if (rowsLeft != rowsRight || colsLeft != colsRight)return;

	int channels = src1.channels();

	int nr = rowsLeft;
	int nc = colsLeft;
	if (src1.isContinuous()){
		nc = nc*nr;
		nr = 1;
		//std::cout<<"continue"<<std::endl;
	}
	for (int i = 0; i<nr; i++){
		const float* dataLeft = src1.ptr<float>(i);
		const float* dataRight = src2.ptr<float>(i);
		float* dataResult = dst.ptr<float>(i);
		for (int j = 0; j<nc*channels; ++j){
			dataResult[j] = (dataLeft[j] >= dataRight[j]) ? dataLeft[j] : dataRight[j];
		}
	}
}
//获取梯度角度和权重
void FindCorners::getImageAngleAndWeight(Mat img, Mat &imgDu, Mat &imgDv, Mat &imgAngle, Mat &imgWeight){
	Mat sobelKernel(3, 3, CV_32F);
	Mat sobelKernelTrs(3, 3, CV_32F);
	//soble滤波器算子核
	sobelKernel.col(0).setTo(cv::Scalar(-1));
	sobelKernel.col(1).setTo(cv::Scalar(0));
	sobelKernel.col(2).setTo(cv::Scalar(1));

	sobelKernelTrs = sobelKernel.t();

	filter2D(img, imgDu, CV_32F, sobelKernel);
	filter2D(img, imgDv, CV_32F, sobelKernelTrs);

	if (imgDu.size() != imgDv.size())return;

	for (int i = 0; i < imgDu.rows; i++)
	{
		float* dataDv = imgDv.ptr<float>(i);
		float* dataDu = imgDu.ptr<float>(i);
		float* dataAngle = imgAngle.ptr<float>(i);
		float* dataWeight = imgWeight.ptr<float>(i);
		for (int j = 0; j < imgDu.cols; j++)
		{
			if (dataDu[j]>0.000001)
			{
				dataAngle[j] = atan2((float)dataDv[j], (float)dataDu[j]);
				if (dataAngle[j] < 0)dataAngle[j] = dataAngle[j] + CV_PI;
				else if (dataAngle[j] > CV_PI)dataAngle[j] = dataAngle[j] - CV_PI;
			}
			dataWeight[j] = std::sqrt((float)dataDv[j] * (float)dataDv[j] + (float)dataDu[j] * (float)dataDu[j]);
		}
	}
}
//**************************非极大值抑制*****************************//
//inputCorners是输入角点，outputCorners是非极大值抑制后的角点
//threshold是设定的阈值
//margin是进行非极大值抑制时检查方块与输入矩阵边界的距离，patchSize是该方块的大小
//*************************************************************************//
void FindCorners::nonMaximumSuppression(Mat& inputCorners, vector<Point2f>& outputCorners,
                                        float threshold, int margin, int patchSize)
{
	if (inputCorners.empty())
	{
		cout << "The imput mat is empty!" << endl; return;
	}
	for (int i = margin + patchSize; i < inputCorners.cols - (margin + patchSize); i = i + patchSize + 1)//移动检查方块，每次移动一个方块的大小
	{
		for (int j = margin + patchSize; j < inputCorners.rows - (margin + patchSize); j = j + patchSize + 1)
		{
			float maxVal = inputCorners.ptr<float>(j)[i];
			int maxX = i; int maxY = j;
			for (int m = i; m < i + patchSize +1; m++)//找出该检查方块中的局部最大值
			{
				for (int n = j; n < j + patchSize +1; n++)
				{
					float temp = inputCorners.ptr<float>(n)[m];
					if (temp>maxVal)
					{
						maxVal = temp; maxX = m; maxY = n;
					}
				}
			}
			if (maxVal < threshold)continue;//若该局部最大值小于阈值则不满足要求
			int flag = 0;
			for (int m = maxX - patchSize; m < min(maxX + patchSize, inputCorners.cols-margin); m++)//二次检查
			{
				for (int n = maxY - patchSize; n < min(maxY + patchSize, inputCorners.rows - margin); n++)
				{
					if (inputCorners.ptr<float>(n)[m]>maxVal && (m<i || m>i + patchSize || n<j || n>j + patchSize))
					{
						flag = 1; break;
					}
				}
				if (flag)break;
			}
			if (flag)continue;
			outputCorners.push_back(Point2f(maxX, maxY));
			std::vector<float> e1(2, 0.0);
			std::vector<float> e2(2, 0.0);
			cornersEdge1.push_back(e1);
			cornersEdge2.push_back(e2);
		}
	}
}
//find modes of smoothed histogram
void FindCorners::findModesMeanShift(vector<float> hist, vector<float> &hist_smoothed, vector<pair<float, int>> &modes, float sigma){
	//efficient mean - shift approximation by histogram smoothing
	//compute smoothed histogram
	bool allZeros = true;
	for (int i = 0; i <(int) hist.size(); i++)
	{
		float sum = 0;
		for (int j = -(int)round(2 * sigma); j <= (int)round(2 * sigma); j++)
		{
			int idx = 0;
			if ((i + j) < 0)idx = i + j + hist.size();
			else if ((i + j) >= 32)idx = i + j - hist.size();
			else idx = (i + j);
			sum = sum + hist[idx] * normpdf(j, 0, sigma);
		}
		hist_smoothed[i]=sum;
		if (abs(hist_smoothed[i] - hist_smoothed[0])>0.0001)allZeros = false;// check if at least one entry is non - zero
																			//(otherwise mode finding may run infinitly)
	}
	if (allZeros)return;

	//mode finding
	//for (int i = 0; i < hist.size(); i++)
	//{
	//	int j = i;
	//	while (true)
	//	{
	//		float h0 = hist_smoothed[j];
	//		int j1 = (j - 1)<0 ? j - 1 + hist.size() : j - 1;
	//		j1 = j>hist.size() ? j - 1 - hist.size() : j - 1;
	//		int j2 = (j + 1)>hist.size() - 1 ? j + 1 - hist.size() : j + 1;
	//		j2 = (j + 1)<0 ? j + 1 + hist.size() : j + 1;
	//		float h1 = hist_smoothed[j1];
	//		float h2 = hist_smoothed[j2];
	//		if (h1 >= h0&&h1 >= h2)j = j1;
	//		else if (h2 >= h0&&h2 >= h1)j = j2;
	//		else break;
	//	}
	//	if (modes.size() == 0 || modes[i].x!=(float)j)
	//	{

	//	}
	//}
	for (int i = 0; i<(int)hist.size(); ++i){
		int j = i;
		int curLeft = (j - 1)<0 ? j - 1 + (int)hist.size() : j - 1;
		int curRight = (j + 1)>(int)hist.size() - 1 ? j + 1 - (int)hist.size() : j + 1;
		if (hist_smoothed[curLeft]<hist_smoothed[i] && hist_smoothed[curRight]<hist_smoothed[i]){
			modes.push_back(std::make_pair(hist_smoothed[i], i));
		}
	}
	std::sort(modes.begin(), modes.end());
}
//estimate edge orientations
void FindCorners::edgeOrientations(Mat imgAngle, Mat imgWeight, int index){
	//number of bins (histogram parameter)
	int binNum = 32;

	//convert images to vectors
	if (imgAngle.size() != imgWeight.size())return;
	vector<float> vec_angle, vec_weight;
	for (int i = 0; i < imgAngle.cols; i++) {
		for (int j = 0; j < imgAngle.rows; j++) {
			// convert angles from normals to directions
			float angle = imgAngle.at<float>(i, j) + (float)M_PI / 2;
			if (angle > M_PI)
				angle -= M_PI;
			vec_angle.push_back(angle);

			float w = imgWeight.at<float>(i, j);
			vec_weight.push_back(w);
		}
	}

	//create histogram
	vector<float> angleHist(binNum, 0);
	for (int i = 0; i < (int)vec_angle.size(); i++)
	{
		int bin = max(min((int)floor(vec_angle[i] / (CV_PI / binNum)), binNum - 1), 0);
		angleHist[bin] = angleHist[bin] + vec_weight[i];
	}

	// find modes of smoothed histogram
	vector<float> hist_smoothed(angleHist);
	vector<std::pair<float, int> > modes;
	findModesMeanShift(angleHist, hist_smoothed, modes,1);

	// if only one or no mode = > return invalid corner
	if (modes.size() <= 1)return;

	//extract 2 strongest modes and compute orientation at modes
	std::pair<float, int> most1 = modes[modes.size() - 1];
	std::pair<float, int> most2 = modes[modes.size() - 2];
	float most1Angle = most1.second*CV_PI / binNum;
	float most2Angle = most2.second*CV_PI / binNum;
	float tmp = most1Angle;
	most1Angle = (most1Angle>most2Angle) ? most1Angle : most2Angle;
	most2Angle = (tmp>most2Angle) ? most2Angle : tmp;

	// compute angle between modes
	float deltaAngle = min(most1Angle - most2Angle, most2Angle + (float)CV_PI - most1Angle);

	// if angle too small => return invalid corner
	if (deltaAngle <= 0.3)return;

	//set statistics: orientations
	cornersEdge1[index][0] = cos(most1Angle);
	cornersEdge1[index][1] = sin(most1Angle);
	cornersEdge2[index][0] = cos(most2Angle);
	cornersEdge2[index][1] = sin(most2Angle);
}

void FindCorners::refineCorners(vector<Point2f> &cornors, Mat imgDu, Mat imgDv, Mat imgAngle, Mat imgWeight, float radius){
	// image dimensions
	int width = imgDu.cols;
	int height = imgDu.rows;
	// for all corners do
	for (int i = 0; i < (int)cornors.size(); i++)
	{
		//extract current corner location
		float cu = cornors[i].x;
		float cv = cornors[i].y;
		// estimate edge orientations
		int startX, startY, ROIwidth, ROIheight;
		startX = (int) std::max(cu - radius, (float)0);
		startY = (int) std::max(cv - radius, (float)0);
		ROIwidth = (int) min(cu + radius, (float)width-1) - startX ;
		ROIheight = (int) min(cv + radius, (float)height-1) - startY ;

		Mat roiAngle, roiWeight;
		roiAngle = imgAngle(Rect(startX, startY, ROIwidth, ROIheight));
		roiWeight = imgWeight(Rect(startX, startY, ROIwidth, ROIheight));
		edgeOrientations(roiAngle, roiWeight,i);

		// continue, if invalid edge orientations
		if ((cornersEdge1[i][0] == 0 && cornersEdge1[i][1] == 0)
			|| (cornersEdge2[i][0] == 0 && cornersEdge2[i][1] == 0))
			continue;

        cv::Mat A1 = cv::Mat::zeros(2,2,CV_32F);
        cv::Mat A2 = cv::Mat::zeros(2,2,CV_32F);

        cv::Mat G = cv::Mat::zeros(2,2,CV_32F);
        cv::Mat b = cv::Mat::zeros(2,1,CV_32F);

        std::vector<float> e1(cornersEdge1[i]);
        std::vector<float> e2(cornersEdge2[i]);
        //std::cout<<"e1:"<<e1[0]<<' '<<e1[1]<<" e2:"<<e2[0]<<' '<<e2[1]<<std::endl;

		// TODO: split (with svd decom) into two for loop
        for(int u=startX; u<=startX+ROIwidth; ++u){
            for(int v=startY; v<=startY+ROIheight; v++){
                std::vector<float> gp;
                gp.push_back(imgDu.at<float>(v,u));
                gp.push_back(imgDv.at<float>(v,u));
                float normLen=std::sqrt(gp[0]*gp[0]+gp[1]*gp[1]);
                //std::cout<<"normLen:"<<normLen<<std::endl;
                if(normLen<0.1)continue;
                gp[0]=gp[0]/normLen;
                gp[1]=gp[1]/normLen;


//                float dot_gp_e1=gp[0]*e1[0]+gp[1]*e1[1];
//                dot_gp_e1=(dot_gp_e1>0)?dot_gp_e1:-dot_gp_e1;
                float dot_gp_e1=std::abs(gp[0]*e1[0]+gp[1]*e1[1]);
                if(dot_gp_e1<0.25){
                    A1.at<float>(0,0)+=gp[0]*gp[0];
                    A1.at<float>(0,1)+=gp[0]*gp[1];
                    A1.at<float>(1,0)+=gp[1]*gp[0];
                    A1.at<float>(1,1)+=gp[1]*gp[1];
                }

                float dot_gp_e2=std::abs(gp[0]*e2[0]+gp[1]*e2[1]);
                if(dot_gp_e2<0.25){
                    A2.at<float>(0,0)+=gp[0]*gp[0];
                    A2.at<float>(0,1)+=gp[0]*gp[1];
                    A2.at<float>(1,0)+=gp[1]*gp[0];
                    A2.at<float>(1,1)+=gp[1]*gp[1];
                }
                // std::cout<<"A1:"<<A1<<" A2:"<<A2<<std::endl;
                //std::cout<<"dot_gp_e1:"<<dot_gp_e1<<" dot_gp_e2:"<<dot_gp_e2<<std::endl;
                std::vector<float> vec_cp;
                vec_cp.push_back(u-cu);
                vec_cp.push_back(v-cv);
                if(u!=cu||v!=cv){
                    float dot=vec_cp[0]*e1[0]+vec_cp[1]*e1[1];
                    float d1u=vec_cp[0]-dot*e1[0];
                    float d1v=vec_cp[1]-dot*e1[1];
                    float d1=std::sqrt(d1u*d1u+d1v*d1v);

                    float dot2=vec_cp[0]*e2[0]+vec_cp[1]*e2[1];
                    float d2u=vec_cp[0]-dot2*e2[0];
                    float d2v=vec_cp[1]-dot2*e2[1];
                    float d2=std::sqrt(d2u*d2u+d2v*d2v);

                    if((d1<3&&dot_gp_e1<0.25) || (d2<3&&dot_gp_e2<0.25)){

                        G.at<float>(0,0)+=gp[0]*gp[0];
                        G.at<float>(0,1)+=gp[0]*gp[1];
                        G.at<float>(1,0)+=gp[1]*gp[0];
                        G.at<float>(1,1)+=gp[1]*gp[1];
                        b.at<float>(0,0)+=gp[0]*gp[0]*u+gp[0]*gp[1]*v;
                        b.at<float>(1,0)+=gp[1]*gp[0]*u+gp[1]*gp[1]*v;
                        //std::cout<<"G:"<<G<<std::endl;
                        //std::cout<<"b:"<<b<<std::endl;
                        //std::cout<<gp[0]*gp[0]<<' '<<gp[0]*gp[1]<<' '<<gp[1]*gp[0]<<' '
                        //           <<gp[1]*gp[1]<<' '<<gp[0]*gp[0]*u+gp[0]*gp[1]*v<<' '<<gp[1]*gp[0]*u+gp[1]*gp[1]*v<<std::endl;

                    }
                    //std::cout<<"G:"<<G<<" b:"<<b<<std::endl;
                }

            }
        }
        cv::Mat u,w,vt;
        cv::SVDecomp(A1,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        cornersEdge1[i][0]=vt.at<float>(1,0);
        cornersEdge1[i][1]=vt.at<float>(1,1);

        cv::Mat u2,w2,vt2;
        cv::SVDecomp(A2,w2,u2,vt2,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        cornersEdge2[i][0]=vt2.at<float>(1,0);
        cornersEdge2[i][1]=vt2.at<float>(1,1);

        // std::cout<<"vt:"<<vt<<" vt2:"<<vt2<<std::endl;

        // std::cout<<"newang1:"<<cornersEdge1[i][0]<<' '<<cornersEdge1[i][1]
		// <<" newang2:"<<cornersEdge2[i][0]<<' '<<cornersEdge2[i][1]<<std::endl;
        // float absDeterG=cv::determinant(G);
        // absDeterG=(absDeterG>0)?absDeterG:-absDeterG;
        // std::cout<<"G:"<<G<<std::endl;
        // std::cout<<"det(G):"<<std::abs(cv::determinant(G))<<std::endl;
        if(std::abs(cv::determinant(G))>0.001){
            cv::Mat cornerNew(2,1,CV_32F);
            cornerNew=G.inv()*b;

            // cv::Point2f* pTmp=&cornors[i];
            // float new_u=cornerNew.at<float>(0,0);
            // float new_v=cornerNew.at<float>(1,0);
            // float cur_u=pTmp->x;
            // float cur_v=pTmp->y;
            //std::cout<<"new_u:"<<new_u<<" cur_u:"<<cur_u<<" new_v:"<<new_v<<" cur_v:"<<cur_v<<std::endl;
//            if(sqrt((new_u-cur_u)*(new_u-cur_u)+(new_v-cur_v)*(new_v-cur_v))>=4){
//                cornersEdge1[i][0]=0.0;
//                cornersEdge1[i][1]=0.0;
//                cornersEdge2[i][0]=0.0;
//                cornersEdge2[i][1]=0.0;

//            }
            // pTmp->x = new_u;
            // pTmp->x = new_v;

        } else{
            cornersEdge1[i][0]=0.0;
            cornersEdge1[i][1]=0.0;
            cornersEdge2[i][0]=0.0;
            cornersEdge2[i][1]=0.0;
        }
	}
}
//compute corner statistics
void FindCorners::cornerCorrelationScore(Mat img, Mat imgWeight, vector<Point2f> cornersEdge, float &score){
	//center
	int c[] = { imgWeight.cols / 2, imgWeight.cols / 2 };

	//compute gradient filter kernel(bandwith = 3 px)
	Mat img_filter = Mat::ones(imgWeight.size(), imgWeight.type());
	img_filter = img_filter*-1;
	for (int i = 0; i < imgWeight.cols; i++)
	{
		for (int j = 0; j < imgWeight.rows; j++)
		{
			Point2f p1 = Point2f(i - c[0], j - c[1]);
			Point2f p2 = Point2f(p1.x*cornersEdge[0].x*cornersEdge[0].x + p1.y*cornersEdge[0].x*cornersEdge[0].y,
				p1.x*cornersEdge[0].x*cornersEdge[0].y + p1.y*cornersEdge[0].y*cornersEdge[0].y);
			Point2f p3 = Point2f(p1.x*cornersEdge[1].x*cornersEdge[1].x + p1.y*cornersEdge[1].x*cornersEdge[1].y,
				p1.x*cornersEdge[1].x*cornersEdge[1].y + p1.y*cornersEdge[1].y*cornersEdge[1].y);
			float norm1 = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
			float norm2 = sqrt((p1.x - p3.x)*(p1.x - p3.x) + (p1.y - p3.y)*(p1.y - p3.y));
			if (norm1 <= 1.5 || norm2 <= 1.5)
			{
				img_filter.ptr<float>(j)[i] = 1;
			}
		}
	}

	//normalize
	Mat mean, std, mean1, std1;
	meanStdDev(imgWeight, mean, std);
	meanStdDev(img_filter, mean1, std1);
	for (int i = 0; i < imgWeight.cols; i++)
	{
		for (int j = 0; j < imgWeight.rows; j++)
		{
			imgWeight.ptr<float>(j)[i] = (float)(imgWeight.ptr<float>(j)[i] - mean.ptr<double>(0)[0]) / (float)std.ptr<double>(0)[0];
			img_filter.ptr<float>(j)[i] = (float)(img_filter.ptr<float>(j)[i] - mean1.ptr<double>(0)[0]) / (float)std1.ptr<double>(0)[0];
		}
	}

	//convert into vectors
	vector<float> vec_filter, vec_weight;
	for (int i = 0; i < imgWeight.cols; i++)
	{
		for (int j = 0; j < imgWeight.rows; j++)
		{
			vec_filter.push_back(img_filter.ptr<float>(j)[i]);
			vec_weight.push_back(imgWeight.ptr<float>(j)[i]);
		}
	}

	//compute gradient score
	float sum = 0;
	for (int i = 0; i < (int)vec_weight.size(); i++)
	{
		sum += vec_weight[i] * vec_filter[i];
	}
	sum = (float)sum / (float)(vec_weight.size() - 1);
	float score_gradient = sum >= 0 ? sum : 0;

	//create intensity filter kernel
	Mat kernelA, kernelB, kernelC, kernelD;
	createkernel(atan2(cornersEdge[0].y, cornersEdge[0].x), atan2(cornersEdge[1].y, cornersEdge[1].x), c[0], kernelA, kernelB, kernelC, kernelD);//1.1 产生四种核

	//checkerboard responses
	float a1, a2,b1,b2;
	a1 = kernelA.dot(img);
	a2 = kernelB.dot(img);
	b1 = kernelC.dot(img);
	b2 = kernelD.dot(img);

	float mu = (a1 + a2 + b1 + b2) / 4;

	float score_a = (a1 - mu) >= (a2 - mu) ? (a2 - mu) : (a1 - mu);
	float score_b = (mu - b1) >= (mu - b2) ? (mu - b2) : (mu - b1);
	float score_1 = score_a >= score_b ? score_b : score_a;

	score_b = (b1 - mu) >= (b2 - mu) ? (b2 - mu) : (b1 - mu);
	score_a = (mu - a1) >= (mu - a2) ? (mu - a2) : (mu - a1);
	float score_2 = score_a >= score_b ? score_b : score_a;

	float score_intensity = score_1 >= score_2 ? score_1 : score_2;

	score = score_gradient*score_intensity;
}
//score corners
void FindCorners::scoreCorners(Mat img, Mat imgAngle, Mat imgWeight, vector<Point2f> &cornors, vector<int> radius, vector<float> &score){
	//for all corners do
	for (int i = 0; i < (int)cornors.size(); i++)
	{
		//corner location
		float u = cornors[i].x;
		float v = cornors[i].y;

		//compute corner statistics @ radius 1
		vector<float> scores;
		for (int j = 0; j < (int)radius.size(); j++)
		{
			scores.push_back(0);
			int r = radius[j];
			if (u > r&&u <= (img.cols - r - 1) && v>r && v <= (img.rows - r -1))
			{
				int startX, startY, ROIwidth, ROIheight;
				startX = int(u-r);
				startY = int(v-r);
				ROIwidth = 2 * r + 1;
				ROIheight = 2 * r + 1;

				Mat sub_img = img(Rect(startX, startY, ROIwidth, ROIheight));
				Mat sub_imgWeight = imgWeight(Rect(startX, startY, ROIwidth, ROIheight));
				vector<Point2f> cornersEdge;
				cornersEdge.push_back(Point2f((float)cornersEdge1[i][0], (float)cornersEdge1[i][1]));
				cornersEdge.push_back(Point2f((float)cornersEdge2[i][0], (float)cornersEdge2[i][1]));
				cornerCorrelationScore(sub_img, sub_imgWeight, cornersEdge, scores[j]);
			}
		}
		//take highest score
		score.push_back(*max_element(begin(scores), end(scores)));
	}
	
}
void FindCorners::detectCorners(Mat &gray, vector<Point2f> &resultCornors, float scoreThreshold){
	Mat imageNorm;

	Mat imageDu = Mat::zeros(gray.size(), CV_32F);
	Mat imageDv = Mat::zeros(gray.size(), CV_32F);
	Mat img_angle = Mat::zeros(gray.size(), CV_32F);
	Mat img_weight = Mat::zeros(gray.size(), CV_32F);
	getImageAngleAndWeight(gray, imageDu, imageDv, img_angle, img_weight);

	normalize(gray, imageNorm, 0, 1, cv::NORM_MINMAX, CV_32F);

    //img corner extract from convolusion
	Mat imgCorners = Mat::zeros(imageNorm.size(), CV_32F);
    // convolusion
	for (int i = 0; i < 4; i++)
	{
        //1.1 generate 4 kernel
		Mat kernelA1, kernelB1, kernelC1, kernelD1;
		createkernel(templateProps[i].x, templateProps[i].y, radius[i / 2], kernelA1, kernelB1, kernelC1, kernelD1);

		Mat imgCornerA1(imageNorm.size(), CV_32F);
		Mat imgCornerB1(imageNorm.size(), CV_32F);
		Mat imgCornerC1(imageNorm.size(), CV_32F);
		Mat imgCornerD1(imageNorm.size(), CV_32F);
		filter2D(imageNorm, imgCornerA1, CV_32F, kernelA1);
		filter2D(imageNorm, imgCornerB1, CV_32F, kernelB1);
		filter2D(imageNorm, imgCornerC1, CV_32F, kernelC1);
		filter2D(imageNorm, imgCornerD1, CV_32F, kernelD1);

		Mat imgCornerMean(imageNorm.size(), CV_32F);
		imgCornerMean = (imgCornerA1 + imgCornerB1 + imgCornerC1 + imgCornerD1) / 4;//1.3 按照公式进行计算
		Mat imgCornerA(imageNorm.size(), CV_32F);
		Mat imgCornerB(imageNorm.size(), CV_32F);
		Mat imgCorner1(imageNorm.size(), CV_32F);
		Mat imgCorner2(imageNorm.size(), CV_32F);

		getMin(imgCornerA1 - imgCornerMean, imgCornerB1 - imgCornerMean, imgCornerA);
		getMin(imgCornerMean - imgCornerC1, imgCornerMean - imgCornerD1, imgCornerB);
		getMin(imgCornerA, imgCornerB, imgCorner1);

		getMin(imgCornerMean - imgCornerA1, imgCornerMean - imgCornerB1, imgCornerA);
		getMin(imgCornerC1 - imgCornerMean, imgCornerD1 - imgCornerMean, imgCornerB);
		getMin(imgCornerA, imgCornerB, imgCorner2);

		getMax(imgCorners, imgCorner1, imgCorners);
		getMax(imgCorners, imgCorner2, imgCorners);

	}

	nonMaximumSuppression(imgCorners, cornerPoints, 0.025, 5, 3);//1.5 非极大值抑制算法进行过滤，获取棋盘格角点初步结果
	cout <<"cornerPoints: " << cornerPoints.size() << endl;

	//subpixel refinement

    refineCorners(cornerPoints, imageDu, imageDv, img_angle, img_weight, 10);
    if (cornerPoints.size()>0) {
		for (int i = 0; i < (int)cornerPoints.size(); i++) {
            if ((cornersEdge1[i][0] == 0 && cornersEdge1[i][1] == 0)
				|| (cornersEdge2[i][0] == 0 && cornersEdge2[i][1] == 0)) {
                cornerPoints[i].x = 0; cornerPoints[i].y = 0;
            }
            // cv::Scalar color = CV_RGB(255, 0, 0);
			// circle(gray, cornerPoints[i], 5, color, 2);
		}
	}
	// namedWindow("src");
	// imshow("src", gray); waitKey(0);

	//score corners
	vector<float> score;
	scoreCorners(imageNorm, img_angle, img_weight, cornerPoints, radius, score);

	if (cornerPoints.size()>0) {
		for (int i = 0; i < (int)cornerPoints.size(); i++) {
			if (score[i]>scoreThreshold) {
                resultCornors.push_back(cornerPoints[i]);
				// circle(gray, cornerPoints[i], 5, CV_RGB(0, 0, 255), 2);
			}
		}
	}
	cout << "resultCornors: " << resultCornors.size() << endl;
	// namedWindow("src");
	// imshow("src", gray); waitKey(0);

}
