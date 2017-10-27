#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>


using namespace cv;
using namespace std;

class skin_indexs_detect
{

private:


public:

	void preDisposeImage(cv::Mat image, cv::Mat &dstImage, int channel, int reverse);
	int getImageSumValue(cv::Mat image, int channel);
	int getImageMeanValue(cv::Mat image, int channel);
	int getImageMaxValue(cv::Mat image, int channel);
	int getImageMinValue(cv::Mat image, int channel);
	int getImageMinRValue(cv::Mat image, int channel);
	int getImageMaxRValue(cv::Mat image, int channel);
	int getImageVarValue(cv::Mat image, int channel);
	float getImageEntropyValue(cv::Mat image, int channel);
	int getImagePowerValue(cv::Mat image, int channel);
	int* getFeatureData(cv::Mat image);
	float* getImageProperty(char* imagePath);

};