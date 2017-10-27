// skin-indexs-detect.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "skin_indexs_detect.h"

void skin_indexs_detect::preDisposeImage(cv::Mat image, cv::Mat &dstImage, int channel, int reverse) {
	Mat img_gry;
	if (reverse == 1){
		int nr = image.rows; // number of rows
		int nc = image.cols; // number of columns
		for (int j = 0; j<nr; j++) {
			for (int i = 0; i<nc; i++) {
				image.at<cv::Vec3b>(j, i)[channel] = 255 - image.at<cv::Vec3b>(j, i)[channel];
			}
		}
	}
	int blockSize = 25;
	int constValue = 12;
	cvtColor(image, img_gry, CV_BGR2GRAY);
	//cv::medianBlur(img_gry, result, 3);
	//ImageStretchByHistogram(img_gry, img_gry);
	//threshold(img_gry, result, 120, 255, CV_THRESH_BINARY);
	//adaptiveThreshold(img_gry, result, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	threshold(img_gry, dstImage, 0, 255, CV_THRESH_OTSU);
}

int skin_indexs_detect::getImageSumValue(cv::Mat image, int channel) {
	//int nc = image.cols * image.channels(); // total number of elements per line
	int imageSumValue = 0;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			imageSumValue += image.at<cv::Vec3b>(j, i)[channel];
		}
	}
	return imageSumValue;
}

int skin_indexs_detect::getImageMeanValue(cv::Mat image, int channel) {
	int imageSumValue = 0;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			imageSumValue += image.at<cv::Vec3b>(j, i)[channel];
		}
	}
	return imageSumValue / (nr*nc);
}


int skin_indexs_detect::getImageMaxValue(cv::Mat image, int channel) {
	int imageMaxValue = 0;
	int tempValue = 0;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			tempValue = image.at<cv::Vec3b>(j, i)[channel];
			imageMaxValue = tempValue>imageMaxValue ? tempValue : imageMaxValue;
		}
	}
	return imageMaxValue;
}

int skin_indexs_detect::getImageMinValue(cv::Mat image, int channel) {
	int imageMinValue = 0;
	int tempValue = 0;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	for (int j = 0; j < nr; j++) {
		for (int i = 0; i < nc; i++) {
			tempValue = image.at<cv::Vec3b>(j, i)[channel];
			imageMinValue = tempValue<imageMinValue ? tempValue : imageMinValue;
		}
	}
	return imageMinValue + 1;
}


int skin_indexs_detect::getImageMinRValue(cv::Mat image, int channel) {
	int imageMinRValue = 0;
	int tempvalue = 0;
	Mat img;
	preDisposeImage(image, img, channel, 1);
	int nr = img.rows; // number of rows
	int nc = img.cols; // number of columns
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			tempvalue = img.at<cv::Vec3b>(j, i)[channel];
			if (tempvalue == 255){
				imageMinRValue += 1;
			}
		}
	}
	return imageMinRValue;
}


int skin_indexs_detect::getImageMaxRValue(cv::Mat image, int channel) {
	int imageMaxRValue = 0;
	int tempvalue = 0;
	Mat img;
	preDisposeImage(image, img, channel, 0);
	int nr = img.rows; // number of rows
	int nc = img.cols; // number of columns
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			tempvalue = img.at<cv::Vec3b>(j, i)[channel];
			if (tempvalue == 255){
				imageMaxRValue += 1;
			}
		}
	}
	return imageMaxRValue;
}



int skin_indexs_detect::getImageVarValue(cv::Mat image, int channel) {
	int imageSumValue = 0;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	int avg = 0;
	int var = 0;
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			imageSumValue += image.at<cv::Vec3b>(j, i)[channel];
		}
	}

	avg = imageSumValue / (nr*nc);

	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			var += pow(image.at<cv::Vec3b>(j, i)[channel] - avg, 2);
		}
	}

	return var / (nc*nr - 1);
}

float skin_indexs_detect::getImageEntropyValue(cv::Mat image, int channel) {
	float imageEntropyValue = 0;
	int imageSumValue = 0;
	int imageValue = 0;
	float value;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			imageSumValue += image.at<cv::Vec3b>(j, i)[channel];
		}
	}

	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			imageValue = image.at<cv::Vec3b>(j, i)[channel];
			if (imageValue == 0){
				imageEntropyValue = imageEntropyValue + 0;
			}
			else{
				//entropyValue=entropyValue-(x/sumValue)*log2(x/sumValue);
				value = imageValue*1.0f / imageSumValue*1.0f;
				imageEntropyValue = imageEntropyValue - (value)*log(value) / log(2);
			}
		}
	}
	return imageEntropyValue;
}


int skin_indexs_detect::getImagePowerValue(cv::Mat image, int channel) {
	unsigned int imagePowerValue = 0;
	unsigned int singlePower = 0;
	float valueFloat = 0.0f;
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns
	int y = round(nr / 2);
	int x = round(nc / 2);
	int value = 0;
	int valueCenter = 0;
	for (int j = 0; j<nr; j++) {
		for (int i = 0; i<nc; i++) {
			if (i != x && j != y){
				value = image.at<cv::Vec3b>(j, i)[channel];
				valueCenter = image.at<cv::Vec3b>(y, x)[channel];
				if (value - valueCenter<0){
					singlePower = 0;
				}
				else{
					valueFloat = (value - valueCenter)*1.0f / sqrt(pow((i - x), 2) + pow((j - y), 2));
					singlePower = round((valueFloat));
				}
				imagePowerValue += singlePower;
			}
		}
	}
	return imagePowerValue;
}


int* skin_indexs_detect::getFeatureData(cv::Mat image){
	int* data = new int[15];
	for (int i = 0; i<3; i++){

		data[5 * i + 0] = getImageMeanValue(image, 2 - i);
		data[5 * i + 1] = getImageVarValue(image, 2 - i);
		data[5 * i + 2] = getImageMaxRValue(image, 2 - i);
		data[5 * i + 3] = getImageMinRValue(image, 2 - i);
		data[5 * i + 4] = getImagePowerValue(image, 2 - i);
	}
	return data;
}

float* skin_indexs_detect::getImageProperty(char* imagePath){
	Mat img;
	img = imread(imagePath, 1);
	float * result = new float[17]; 
	int* data = getFeatureData(img);

	for (int i; i<15; i++){
		result[i] = data[i] * 1.0f;
	}

	if (result[4]<100 && result[9]<100 && result[14]<100)
	{ //
		result[15] = 0;
		result[16] = 0;
		return result;
	}

	if (result[4]>10000 && result[9]>10000 && result[14]>10000)
	{ //
		result[15] = 0;
		result[16] = 0;
		return result;
	}

	if (result[1]>1000 && data[6]>1000 && result[11]>1000)
	{ //
		result[15] = 0;
		result[16] = 0;
		return result;
	}

	if (result[1]<50 && result[6]<50 && result[11]<50)
	{ //
		result[15] = 0;
		result[16] = 0;
		return result;
	}

	for (int i = 0; i<15; i++){
		if (result[i]<10){
			result[15] = 0;
			result[16] = 0;
			return result;
		}
	}

	float value0 = (result[5] + result[10] - result[0])*1.5f / (result[5] + result[10]); //����
	float value1 = 10.7 / (log(result[2]) + 1);
	float value2 = 4 / (log(result[11]) + 1);
	float value3 = result[15] * 0.7 + 4 / (log(result[11]) + 1)*0.3;
	float value4 = (result[15] - (0.8 - result[16]) + 4 / (log(result[11]) + 1) + result[15] * 0.7 + 4 / (log(result[11]) + 1)*0.3) / 3;

	for (int i = 15; i<20; i++){
		if (result[i]>0.99){
			result[i] = 0.99;
		}
		else if (result[i]<0.36){
			result[i] = 0.36;
		}
	}
	result[19] = result[19] * 100;

	return result;
}
