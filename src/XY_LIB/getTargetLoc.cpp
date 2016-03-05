#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/gpu/gpumat.hpp"
#include "opencv2/video/tracking.hpp"
#include "fisheye.hpp"
#include <string>
#include <stdio.h>
#include <iostream>
#include "getTargetLoc.hpp"
#include "inifile.h"
#include <time.h>
#include <math.h>
using namespace cv;
using namespace std;

#define  BUF_SIZE 256
static double max(double a, double b)
{
	if (a > b)
		return a;
	else
		return b;
}
static double min(double a, double b)
{
	if(a < b)
		return a;
	else
		return b;
}

xyVision::GetTarget::GetTarget(string configName)
{
	this->setCameraParams(configName);
	frameCounter = 0;
	frameCounterTarget = 0;
	isDetected = true;
	this->useGPU = false;
	time1 = 0;
	time2 = 0;
	time3 = 0;


	if (this->cameraInfo.cameraType == 1)
	{
		cameraInfo.newCameraMatrix = cameraInfo.cameraMatrix * proInfo.scale;
		cameraInfo.newCameraMatrix(2, 2) = 1.0f;
	}
		// compute rectify mappings
	else if (this->cameraInfo.cameraType == 2)
	{
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraInfo.cameraMatrix, cameraInfo.distortCoeff,
			cameraInfo.imageSize, cv::Matx33d::eye(), cameraInfo.newCameraMatrix, 0.8, cameraInfo.newSize, 1);
		cv::fisheye::initUndistortRectifyMap(cameraInfo.cameraMatrix, cameraInfo.distortCoeff,
			cv::Matx33d::eye(), cameraInfo.newCameraMatrix, cameraInfo.newSize, CV_16SC2, cameraInfo.map1, cameraInfo.map2);
	}

	// initialize Kalman fileter
	KF = KalmanFilter(6, 3, 0);
	KF.transitionMatrix = *(Mat_<float>(6,6) << 1,0,0,1,0,0,   0,1,0,0,1,0,  0,0,1,0,0,1,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(0.5));
	setIdentity(KF.errorCovPost, Scalar::all(1e-2));

	// compute mapping
	this->mapping.resize(256);
	double p[5] = {-3.283780259159030e-07, 1.182791454914262e-04, -0.006030084747775, 0.254476804888576, 2.376613986565857};
	for (int i = 0; i < 256; ++i)
	{
		double r = 0;
		for (int o = 4; o >=0; o--)
		{
			r = r + p[4-o]*pow((double)i, (double)o);
		}
		r = max(r, 0);
		r = min(255, r);
		r = int(floor(r));
		mapping[i] = r;
	}
}

void xyVision::GetTarget::setCameraParams(string configFileName)
{
	// read camera parameters
	this->configFileName = configFileName;
	const char * section = "cameraConfig";
	const char * key = "cameraType";
	this->cameraInfo.cameraType = read_profile_int(section, key, 0, configFileName.c_str());
	key = "fx";
	float fx = read_profile_float(section, key, 0, configFileName.c_str());
	key = "fy";
	float fy = read_profile_float(section, key, 0, configFileName.c_str());
	key = "u0";
	float u0 = read_profile_float(section, key, 0, configFileName.c_str());
	key = "v0";
	float v0 = read_profile_float(section, key, 0, configFileName.c_str());
	key = "k1";
	float k1 = read_profile_float(section, key, 0, configFileName.c_str());
	key = "k2";
	float k2 = read_profile_float(section, key, 0, configFileName.c_str());
	key = "k3";
	float k3 = read_profile_float(section, key, 0, configFileName.c_str());
	key = "k4";
	float k4 = read_profile_float(section, key, 0, configFileName.c_str());
	key = "imageWidth";
	int imageWidth = read_profile_int(section, key, 0, configFileName.c_str());
	key = "imageHeight";
	int imageHeight = read_profile_int(section, key, 0, configFileName.c_str());

	Matx33f _cameraMat(fx, 0, u0, 0, fy, v0, 0, 0, 1);
	Matx14f _dis(k1, k2, k3, k4);
	this->cameraInfo.cameraMatrix = _cameraMat;
	this->cameraInfo.distortCoeff = _dis;
	this->cameraInfo.imageSize = Size(imageWidth, imageHeight);

	// read target information
	section = "targetConfig";
	key = "width";
	float width = read_profile_float(section, key, 0, configFileName.c_str());
	this->targetInfo.targetWidth = width;

	// read processing information
	section = "processingConfig";
	key = "scale";
	float scale = read_profile_float(section, key, 0, configFileName.c_str());
	this->proInfo.scale = scale;

	this->cameraInfo.newSize = Size(int(imageWidth*scale), int(imageHeight*scale));


}

xyVision::GetTarget& xyVision::GetTarget::operator<<(const Mat& image)
{
	if (this->frameCounter > 0)
	{
		this->previousImg = this->currentImg.clone();
	}
	this->currentImg = image.clone();

	if (this->useGPU)
	{
		this->runOneFrame_gpu();
	}
	else
	{
		this->runOneFrame();
	}

	if (isDetected)
	{
		// run kalman filter here
		// first frame, initialize kalman filter
		Point3f _point = this->getCurrentLoc();
		if (this->frameCounterTarget == 0)
		{

			KF.statePre = Mat(Matx16f(_point.x, _point.y, _point.z, 0.f, 0.f, 0.f));
		}
		else if (this->frameCounterTarget > 0)
		{
			KF.predict();
			Mat estimate = KF.correct(Mat(_point));
			this->currentLoc = Point3f(estimate.at<float>(0), estimate.at<float>(1), estimate.at<float>(2));
		}
		this->frameCounterTarget ++;
	}
	this->frameCounter ++;
	return *this;
}

Point3f xyVision::GetTarget::getCurrentLoc()
{
	return this->currentLoc;
}

void xyVision::GetTarget::imadjust(const Mat1b& src, Mat1b& dst, int tol, Vec2i in, Vec2i out)
{
	dst = src.clone();
	tol = max(0, min(100, tol));
	if (tol > 0)
	{
		// Compute in and out limits
		// Histogram
		vector<int> hist(256, 0);
		for (int r = 0; r < src.rows; ++r) {
			for (int c = 0; c < src.cols; ++c) {
				hist[src(r,c)]++;
			}
		}
		// Cumulative histogram
		vector<int> cum = hist;
		for (int i = 1; i < (int)hist.size(); ++i) {
			cum[i] = cum[i - 1] + hist[i];
		}
		// Compute bounds
		int total = src.rows * src.cols;
		int low_bound = total * tol / 100;
		int upp_bound = total * (100-tol) / 100;
		in[0] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), low_bound));
		in[1] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), upp_bound));
	}
	// Stretching
	float scale = float(out[1] - out[0]) / float(in[1] - in[0]);
	for (int r = 0; r < dst.rows; ++r)
	{
		for (int c = 0; c < dst.cols; ++c)
		{
			int vs = max(src(r, c) - in[0], 0);
			int vd = min(int(vs * scale + 0.5f) + out[0], out[1]);
			dst(r, c) = saturate_cast<uchar>(vd);
		}
	}
}

void xyVision::GetTarget::adjustImg(Mat & img)
{
	CV_Assert(img.channels() == 3);
	// if (img.channels() != 3)
	// {
	// 	printf("img channel is not 3 \n");
	// 	return ;
	// }

	Size sz = img.size();
	float scaleFactor = this->proInfo.scale;
	if (cameraInfo.cameraType == 1)
	{
		cv::resize(img, img, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
	}
	else
	{
		cv::remap(img, img, cameraInfo.map1, cameraInfo.map2, INTER_LINEAR, BORDER_CONSTANT);
		this->rectified = img.clone();
	}

	//vector<Mat> chs;
	//split(img, chs);
	//Mat1b out;
	// b g r
	//imadjust(chs[2], out);
	//imadjust_mapping(chs[2], out, this->mapping);
	//cv::equalizeHist(chs[2], out);
	//chs[2] = out.clone();

	//imadjust(chs[1], out);
	//imadjust_mapping(chs[1], out, this->mapping);
	//chs[1] = out.clone();
	//imadjust(chs[0], out);
	//imadjust_mapping(chs[0], out, this->mapping);
	//chs[0] = out.clone();

	//merge(chs, img);
	//cv::imwrite("adj.bmp", img);
}

void xyVision::GetTarget::adjustImg_gpu(gpu::GpuMat& img)
{
	CV_Assert(img.channels() == 3);

	Size sz = img.size();
	float scaleFactor = this->proInfo.scale;
	gpu::GpuMat img2;
	gpu::resize(img, img2, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
	img = img2.clone();

	int a = gpu::getCudaEnabledDeviceCount();
	gpu::setDevice(0);
	vector<gpu::GpuMat> chs;
	gpu::split(img, chs);
	gpu::GpuMat out;
	// b g r
	//imadjust(chs[2], out);
	gpu::equalizeHist(chs[2], out);
	chs[2] = out;
	gpu::merge(chs, img);
}

void xyVision::GetTarget::imadjust_mapping(const Mat& src, Mat& dst, vector<int> mapping)
{
	CV_Assert(src.type() == CV_8UC1);
	dst = src.clone();
	const unsigned char* pt_src = src.ptr<unsigned char>(0);
	unsigned char* pt_dst = dst.ptr<unsigned char>(0);
	int n = (int)src.total();
	for (int i = 0; i < n; ++i)
	{
		pt_dst[i] = this->mapping[(int)pt_src[i]];
	}
}

void xyVision::GetTarget::binarizeTarget(const Mat & img, Mat & bi)
{
	CV_Assert(img.channels() == 3);

	std::vector<Mat> chs;

	Mat img2 = img.clone();
	//img2.convertTo(img2, CV_32FC3);
	split(img2, chs);
	Mat img_t = chs[2] - chs[0] - chs[1];
	cv::threshold(img_t, bi, 30, 255, THRESH_BINARY);
	bi.convertTo(bi, CV_8UC1);
	cv::imwrite("bi.bmp", bi);
	// resize binary image
	//Size sz = img2.size();
	//float scaleFactor = this->proInfo.scale;
	//resize(bi, bi, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));

	// improve binary image
	//Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
	//dilate(bi, bi, kernel);
	//erode(bi, bi, kernel);

}

void xyVision::GetTarget::binarizeTarget_gpu(const gpu::GpuMat img, gpu::GpuMat & bi)
{
	CV_Assert(img.channels() == 3);
	gpu::GpuMat img2 = img.clone();
	std::vector<gpu::GpuMat> chs;
	gpu::split(img2, chs);

	Mat tmp;

	//gpu::GpuMat img_t = chs[2] - chs[0] - chs[1];
	gpu::GpuMat img_t, img_tmp;
	gpu::subtract(chs[2], chs[0], img_tmp);
	gpu::subtract(img_tmp, chs[1], img_t);

	gpu::threshold(img_t, bi, 40, 255, THRESH_BINARY);

	bi.convertTo(bi, CV_8UC1);
}
void xyVision::GetTarget::binarizeTarget_HSV(const Mat& img, Mat & bi)
{
	Mat img_hsv;
	cvtColor(img, img_hsv, CV_BGR2HSV);
	std::vector<Mat> chs;
	Mat img2 = img.clone();
	img2.convertTo(img2, CV_32F);
	split(img2, chs);

	Mat img_t = chs[2] - chs[0] - chs[1];
	split(img2, chs);
	
	// H is 0-180, 221-255 is red
	Mat bi_hsv1, bi_hsv2,bi_rgb;

	inRange(img_hsv, Scalar(160, 40, 40), Scalar(180, 255, 255), bi_hsv1);
	//inRange(img_hsv, Scalar(0, 40, 40), Scalar(20, 255, 255), bi_hsv2);
	cv::threshold(chs[2], bi_rgb, 100, 255, THRESH_BINARY);
	//inRange(img, Scalar(0, 0, 100), Scalar(200, 200, 255), bi_rgb);
	//inRange(img, Scalar(0, 0, 0), Scalar(255, 255, 255), bi_rgb);
	//cv::imwrite("bi_hsv1.bmp", bi_hsv1);
	//cv::imwrite("bi_hsv2.bmp", bi_hsv2);
	//cv::imwrite("bi_rgb.bmp", bi_rgb);
	bi_hsv1.convertTo(bi_hsv1, CV_32F);
	//bi_hsv2.convertTo(bi_hsv2, CV_32F);
	bi_rgb.convertTo(bi_rgb, CV_32F);
	bi_hsv1 = bi_hsv1 / 255;
	//bi_hsv2 = bi_hsv2 / 255;
	bi_rgb = bi_rgb / 255;

	//bi = bi_hsv1.mul(bi_rgb).mul(bi_hsv2) * 255;
	cv::threshold((bi_hsv1).mul(bi_rgb), bi, 0, 255, THRESH_BINARY);
	bi.convertTo(bi, CV_8U);


	Mat kernel = cv::Mat::ones(5, 5, CV_8UC1);
	dilate(bi, bi, kernel);
	erode(bi, bi, kernel);
	//cv::imwrite("bi.bmp", bi);
}

void xyVision::GetTarget::binarizeTarget_LAB(const Mat& img, Mat & bi)
{

	Mat img_lab;
	cvtColor(img, img_lab, CV_BGR2Lab);
	std::vector<Mat> chs;
	Mat img2 = img.clone();
	img2.convertTo(img2, CV_32F);
	split(img2, chs);

	Mat bi_lab, bi_rgb;
	inRange(img_lab, Scalar(0, 140, 0), Scalar(255, 255, 255), bi_lab);
	//inRange(img_lab, Scalar(0, 130, 0), Scalar(255, 255, 255), bi_lab);
	cv::threshold(chs[2], bi_rgb, 100, 255, THRESH_BINARY);

	bi_lab.convertTo(bi_lab, CV_32F);
	bi_rgb.convertTo(bi_rgb, CV_32F);

	cv::threshold(bi_lab.mul(bi_rgb), bi, 0, 255, THRESH_BINARY);
	bi.convertTo(bi, CV_8U);

	Mat kernel = cv::Mat::ones(10, 10, CV_8UC1);
	dilate(bi, bi, kernel);
	erode(bi, bi, kernel);
	//cv::imwrite("bi.bmp", bi);
}

bool xyVision::GetTarget::contourDetect(Mat& bi, vector<Point> & tarContour)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Size sz = bi.size();

	//clock_t start,finish;
	//double totaltime;

	findContours(bi.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//cout << "time for finding Contours " << totaltime << endl;
	// no contours
	if ((int)contours.size() == 0)
		return false;

	// filter areas that are not square
	vector<vector<Point> > contours_filter;
	std::vector<double> contours_aeras_filter;
	std::vector<double> center_dis_filter;
	std::vector<double> ratio_filter;
	std::vector<RotatedRect> boxs_filter;
	for (int i = 0; i < (int)contours.size(); ++i)
	{
		RotatedRect box = minAreaRect(Mat(contours[i]));
		float height = float(box.size.height);
		float width = float(box.size.width);
		double aera = contourArea(contours[i]);
		double dis = std::abs(box.center.x - (double)sz.width / 2.0) + std::abs(box.center.y - (double)sz.height/2.0);
		// 计算旋转矩形覆盖比例
		double ratio = computeRotatedRect(contours[i], sz, box, bi);
		if (height / width < 3 && width / height < 3 && aera > (sz.width/80.0f*sz.height/80.0f) && ratio > 0.7)
		{
			contours_filter.push_back(contours[i]);
			center_dis_filter.push_back(dis);
			boxs_filter.push_back(box);
			contours_aeras_filter.push_back(aera);
			ratio_filter.push_back(ratio);
		}
	}
	contours = contours_filter;

	if ((int)contours.size() == 0)
	{
		return false;
	}

	// choose the max one
	int choosen_idx = 0;
	int maxIdx = 0;
	double maxAera = contours_aeras_filter[0];
	for (int i = 1; i < (int)contours.size(); ++i)
	{
		double conAera_i = contours_aeras_filter[i];
		if (conAera_i > maxAera)
		{
			maxAera = conAera_i;
			maxIdx = i;
		}
	}
	choosen_idx = maxIdx;

	// if max area is 1.5 times larger than the second large one,
	//then choose the largest one, otherwise, choose the center one.
	//int n_contour = (int)contours.size();
	//std::vector<double> contours_aeras(n_contour);
	//std::vector<double> center_dis(n_contour);
	//for (int i = 0; i < n_contour; ++i)
	//{
	//	contours_aeras[i] = contourArea(contours[i]);
	//	Rect rect = boundingRect(contours[i]);
	//	center_dis[i] = std::abs(rect.x - (double)sz.width / 2.0) + std::abs(rect.y - (double)sz.height/2.0);
	//}
	//std::vector<double> sorted_area = contours_aeras;
	//std::sort(sorted_area.begin(), sorted_area.end());
	//int choosen_idx = 0;
	//if ((int)contours.size() > 1 && sorted_area[n_contour - 1] / sorted_area[n_contour - 2] > 1.5 )
	//{
	//	// choose the largest one
	//	int maxIdx = 0;
	//	double maxAera = contours_aeras[0];
	//	for (int i = 1; i < (int)contours.size(); ++i)
	//	{
	//		double conAera_i = contours_aeras[i];
	//		if (conAera_i > maxAera)
	//		{
	//			maxAera = conAera_i;
	//			maxIdx = i;
	//		}
	//	}
	//	choosen_idx = maxIdx;
	//}
	//else
	//{
	//	int nearCenIdx = 0;
	//	double nearCenterDis = center_dis[0];
	//	for (int i = 1; i < (int)contours.size(); ++i)
	//	{
	//		double cenDis_i = center_dis[i];
	//		if (cenDis_i < nearCenterDis)
	//		{
	//			nearCenterDis = cenDis_i;
	//			nearCenIdx = i;
	//		}
	//	}
	//	choosen_idx = nearCenIdx;
	//}
	double choosen_aera = contours_aeras_filter[choosen_idx];

	//if (choosen_aera < (sz.width/80.0f*sz.height/80.0f))
	//	return false;
	if (this->frameCounterTarget > 0 &&
		((double)choosen_aera / (double)this->lastArea < 3.0/10 ))
	{
		return false;
	}
	// set zeros for other contours
	for (int i = 0; i < (int)contours.size(); ++i)
	{
		if (i != choosen_idx)
		{
			drawContours(bi, contours, i, cv::Scalar(0), CV_FILLED, 8);
			//drawContours(img_t, contours, i, cv::Scalar(0), CV_FILLED, 8);
		}
	}
	tarContour = contours[choosen_idx];
	this->lastArea = choosen_aera;
	return true;
}

double xyVision::GetTarget::computeRotatedRect(const vector<Point>& contour, cv::Size sz, RotatedRect box, Mat& bi)
{
	Point2f vertices[4];
	box.points(vertices);
	Point points[4];
	for (int i = 0; i < 4; ++i)
	{
		points[i] = Point((int)vertices[i].x, (int)vertices[i].y);
	}

	Mat tmp = Mat::zeros(sz, CV_8U);
	cv::fillConvexPoly(tmp, points, 4, Scalar(1));

	Mat boxIdx;
	findNonZero(tmp, boxIdx);
	double n_pixels = (double)boxIdx.total();
	double n_nonZero = 0;
	for (int i = 0; i < boxIdx.total(); ++i)
	{
		Point _p = boxIdx.at<Point>(i);
		if (bi.at<unsigned char>(_p.y, _p.x) > 0)
		{
			n_nonZero = n_nonZero + 1;
		}
	}
	return n_nonZero / (n_pixels+1e-5);
}


void xyVision::GetTarget::locating(vector<Point> & tarContour)
{
	float scaleFactor = this->proInfo.scale;
	float boardWidth = this->targetInfo.targetWidth;
	Matx33f _K = this->cameraInfo.newCameraMatrix;
	//if (scaleFactor != 1.0)
	//{
	//	Size sz = currentImg.size();
	//	_K = _K * scaleFactor;
	//	_K(2, 2) = 1.0f;
	//}

	RotatedRect _box = minAreaRect(Mat(tarContour));
	this->box = _box;

	float aveLen;
	if (this->frameCounterTarget == 0 || this->lastLoc.z < 500)
	{
		aveLen = std::max(_box.size.width, _box.size.height);
	}
	else
	{
		aveLen = std::min(_box.size.width, _box.size.height);
	}
	//float aveLen = (_box.size.width + _box.size.height) / 2;
	Point2f vtx[4];
	_box.points(vtx);
	Point2f center2d = _box.center;
	Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
	Point3f pointInPlane = _K.inv() * center3d;
	this->currentLoc = pointInPlane * _K(0,0) * (boardWidth / aveLen);
	this->lastLoc = this->currentLoc;
}

void xyVision::GetTarget::runOneFrame()
{
	Mat _img = this->currentImg.clone();

	//clock_t start,finish;
	//double totaltime;
	//start = clock();

	adjustImg(_img);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time1 = time1 + totaltime;

	Mat bi;
	vector<Point> tarContour;
	bool detected;

	//start = clock();

	//this->binarizeTarget(_img, bi);
	//this->binarizeTarget_HSV(_img, bi);
	this->binarizeTarget_LAB(_img, bi);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time2 = time2 + totaltime;

	//start = clock();

	detected = this->contourDetect(bi, tarContour);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time3 = time3 + totaltime;

	if (!detected)
	{
		isDetected = false;
		return;
	}
	else
	{
		isDetected = true;
	}
	locating(tarContour);
}

void xyVision::GetTarget::runOneFrame_gpu()
{

	//clock_t start,finish;
	//double totaltime;

	//start = clock();

	Mat _img = currentImg.clone();
	gpu::GpuMat _img_gpu;
	_img_gpu.upload(_img);
	adjustImg_gpu(_img_gpu);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time1 = time1 + totaltime;

	vector<Point> tarContour;
	bool detected;

	//start = clock();

	gpu::GpuMat bi_gpu;
	binarizeTarget_gpu(_img_gpu, bi_gpu);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time2 = time2 + totaltime;

	//start = clock();

	Mat bi;
	bi_gpu.download(bi);

	detected = this->contourDetect(bi, tarContour);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time3 = time3 + totaltime;

	if (!detected)
	{
		isDetected = false;
		return;
	}
	else
	{
		isDetected = true;
	}
	locating(tarContour);
}

void xyVision::GetTarget::clearStates()
{	
	frameCounter = 0;
	frameCounterTarget = 0;
	isDetected = true;
	this->useGPU = false;

}
//void xyVision::GetTarget::runOneFrame()
//{
//	Matx33f _K = this->cameraInfo.cameraMatrix;
//	float scaleFactor = this->proInfo.scale;
//	float boardWidth = this->targetInfo.targetWidth;
//	Mat img = this->currentImg;
//	Mat _img = img.clone();
//
//	clock_t start,finish;
//	double totaltime;
//	start=clock();
//	//gpu::GpuMat _imgGpu;
//	//cout << gpu::getCudaEnabledDeviceCount() << endl;
//	if (scaleFactor != 1.0f)
//	{
//		Size sz = img.size();
//		_K = _K * scaleFactor;
//		_K(2, 2) = 1.0f;
//
//		
//		//_imgGpu.upload(_img);
//		//gpu::resize(_imgGpu, _imgGpu, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
//		resize(_img, _img, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
//	}
//	//_imgGpu.download(_img);
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for resize " << totaltime << endl;
//
//	start=clock();
//	this->adjustImg(_img);
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for adjust image " << totaltime << endl;
//	Mat bi;
//	vector<Point> tarContour;
//	bool detected;
//
//	start = clock();
//	detected = this->binarizeTarget(_img, bi, tarContour);
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for binaryTarget " << totaltime << endl;
//
//	if (!detected)
//	{
//		isDetected = false;
//		return;
//	}
//	else
//	{
//		isDetected = true;
//	}
//
//	start = clock();
//	RotatedRect box = minAreaRect(Mat(tarContour));
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for locating " << totaltime << endl;
//
//	//float aveLen = std::max(box.size.width, box.size.height);
//	float aveLen = (box.size.width + box.size.height) / 2;
//	Point2f vtx[4];
//	box.points(vtx);
//	Point2f center2d = box.center;
//	Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
//	Point3f pointInPlane = _K.inv() * center3d;
//	this->currentLoc = pointInPlane * _K(0,0) * (boardWidth / aveLen);
//}