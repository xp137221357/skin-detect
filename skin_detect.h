#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;
using namespace std;

class skin_detect
{

private:
	//类私有属性


public:
	//类公有属性

	IplImage * change4channelTo3InIplImage(IplImage * src);
	float on_white(IplImage *pSrcImage);
	int on_trackbar(int pos, IplImage *image);
	float* getImageProperty(char* path);
	void doCanny(Mat src, Mat& dst);
	void imageStretchByHistogram(Mat src, Mat &dst);
	void getGrayImage(Mat src, Mat &dst);
	void getColorMap(Mat src, Mat &dst);
	void getStreImage(Mat src, Mat &dst);
	void getGaussianBlur(Mat src, Mat &dst);
	void getCannyImage(Mat src, Mat &dst);
	void getBwImage(Mat src, Mat &dst);
	void getInvertGaussianBlur(Mat src, Mat &dst);
	void getSurfImage(Mat src, Mat &dst);
	void getErodeOperator(Mat src, Mat &dst);
	void getDilateOperator(Mat src, Mat &dst);
	int * imageMergeOperator(Mat src);
	void  getBaseicOperatorImages(Mat src, vector<int*>& images);

	void findContoursOperator(const Mat srcBw, Mat &dstImage);
	void boundaryDispose(Mat &srcImage);
	void colorspotsDetect(Mat src, Mat &dst);


};
