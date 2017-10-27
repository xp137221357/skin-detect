#include "skin_detect.h"

IplImage* skin_detect::change4channelTo3InIplImage(IplImage * src) {
	if (src->nChannels != 4) {
		return NULL;
	}

	IplImage * destImg = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	for (int row = 0; row < src->height; row++) {
		for (int col = 0; col < src->width; col++) {
			CvScalar s = cvGet2D(src, row, col);
			cvSet2D(destImg, row, col, s);
		}
	}

	return destImg;
}

/*
* 得到美白度
* */
/**
*  获取图片平均亮度(美白度)
*
*  @param pSrcImage 检测的图片
*
*  @return 结果
*/
float skin_detect::on_white(IplImage *pSrcImage){
	long long num = 0;
	long long l = 0;

	char* data = pSrcImage->imageData;
	int step = pSrcImage->widthStep / sizeof(uchar);
	uchar* tmp = new uchar[307200];
	for (int i = 0; i < pSrcImage->height; i++) {
		for (int j = 0; j < pSrcImage->width; j++) {
			*tmp = data[i*step + j];
			num += ((float)*tmp);
			l += 255;
		}
	}
	float  result = ((float)num) / l;
	return result;
}
/**
*  获取毛孔面积在整个图片上所占的比例
*
*  @param pos   阀值
*  @param image 检测的图片
*
*  @return 结果
*/
int skin_detect::on_trackbar(int pos, IplImage *image) {
	//灰度化
	IplImage *g_pGrayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

	cvCvtColor(image, g_pGrayImage, CV_BGR2GRAY);
	//二值化
	IplImage *g_pBinaryImage = cvCreateImage(cvGetSize(g_pGrayImage), IPL_DEPTH_8U, 1);

	cvThreshold(g_pGrayImage, g_pBinaryImage, pos, 255, CV_THRESH_BINARY);

	int num = 0;
	float l = 0;
	char* data = g_pBinaryImage->imageData;
	int step = g_pBinaryImage->widthStep / sizeof(uchar);
	uchar* tmp = new uchar[307200];
	for (int i = 0; i < g_pBinaryImage->height; i++) {
		for (int j = 0; j < g_pBinaryImage->width; j++) {
			*tmp = data[i*step + j];
			num += (int)*tmp;
			l += 255;
		}
	}

	int  result = (num * 100) / l;
	return result;
}

/*
以下是第二次修改的内容----2017-4-9
*/
void skin_detect::doCanny(Mat src, Mat& dst)
{
	double lowThresh = 10;
	double highThresh = 100;
	double aperture = 3;
	Canny(src, dst, lowThresh, highThresh, aperture);
};
//灰度


void skin_detect::imageStretchByHistogram(Mat src, Mat &dst)
/*************************************************
Function:      通过直方图变换进行图像增强，将图像灰度的域值拉伸到0-255
src:               单通道灰度图像
dst:              同样大小的单通道灰度图像
*************************************************/
{
	assert(src.cols == dst.cols);
	double p[256], p1[256], num[256];

	memset(p, 0, sizeof(p));
	memset(p1, 0, sizeof(p1));
	memset(num, 0, sizeof(num));
	int height = src.cols;
	int width = src.rows;
	long wMulh = height * width;
	int widthStep = (src.cols*src.elemSize() + 3) / 4 * 4; // 补齐行字节数，使它能够被4整除
	//statistics
	for (int x = 0; x<src.cols; x++)
	{
		for (int y = 0; y<src.rows; y++){
			uchar v = ((uchar*)(src.data + widthStep*y))[x];
			num[v]++;
		}
	}
	//calculate probability
	for (int i = 0; i<256; i++)
	{
		p[i] = num[i] / wMulh;
	}

	//p1[i]=sum(p[j]);	j<=i;
	for (int i = 0; i<256; i++)
	{
		for (int k = 0; k <= i; k++)
			p1[i] += p[k];
	}

	// histogram transformation
	for (int x = 0; x<src.cols; x++)
	{
		for (int y = 0; y<src.rows; y++){
			uchar v = ((uchar*)(src.data + widthStep*y))[x];
			((uchar*)(dst.data + widthStep*y))[x] = p1[v] * 255 + 0.5;
		}
	}
};
//A
void skin_detect::getGrayImage(Mat src, Mat &dst){
	Mat img_gry;
	cvtColor(src, dst, CV_BGR2GRAY);
}

//B
void skin_detect::getColorMap(Mat src, Mat &dst){
	applyColorMap(src, dst, COLORMAP_JET);
}

//C
void skin_detect::getStreImage(Mat src, Mat &dst){

	imageStretchByHistogram(src, dst);
}

//D
void skin_detect::getGaussianBlur(Mat src, Mat &dst){
	GaussianBlur(src, dst, Size(7, 7), 0, 0);
}

//E
void skin_detect::getCannyImage(Mat src, Mat &dst){
	doCanny(src, dst);
}

//F
void skin_detect::getBwImage(Mat src, Mat &dst){
	int blockSize = 25;
	int constValue = 10;
	//adaptiveThreshold(canny_img,dst, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	threshold(src, dst, 0, 255, CV_THRESH_OTSU);
}
//G
void skin_detect::getInvertGaussianBlur(Mat src, Mat &dst){
	dst = ~src;
}
//H
void skin_detect::getSurfImage(Mat src, Mat &dst){
	cv::cvtColor(src, dst, CV_BGR2GRAY);
	imageStretchByHistogram(dst, dst);
	std::vector<cv::KeyPoint> keypoints;//毛孔数目大小
	SurfFeatureDetector surf(2500);
	surf.detect(dst, keypoints);
	cv::drawKeypoints(dst, keypoints, dst, Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

//I
void skin_detect::getErodeOperator(Mat src, Mat &dst){
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
	morphologyEx(src, dst, MORPH_OPEN, element);
	//cv::erode(src, dst, element);
}

//J
void skin_detect::getDilateOperator(Mat src, Mat &dst){
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
	morphologyEx(src, dst, MORPH_CLOSE, element);
	//cv::dilate(src, dst, element);
}

int * skin_detect::imageMergeOperator(Mat src){

	int row = src.rows;
	int col = src.cols;
	Mat imgMat(row, col, CV_8UC1);
	if (src.channels() == 1)
	{
		imgMat = src;
	}
	else{
		cvtColor(src, imgMat, CV_BGR2GRAY);
	}
	vector<Mat> channelsMat;
	channelsMat.push_back(src);
	for (int i = 0; i<4 - src.channels(); i++)
	{
		channelsMat.push_back(imgMat);
	}
	Mat mergeImage;
	merge(channelsMat, mergeImage);

	int* outImage = new int[row*col];
	for (int i = 0; i < row; i++)
	{
		uchar* pxvec = mergeImage.ptr<uchar>(i);
		for (int j = 0; j < col; j++){
			outImage[i*col + j] = pxvec[j];
		}
	}
	return outImage;
}

void skin_detect::findContoursOperator(const Mat srcBw, Mat &dstImage){
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//findContours(openMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	findContours(srcBw, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	Mat Contours = Mat::zeros(srcBw.size(), CV_8UC1);  //绘制
	//vector<vector<Point> > contours_poly(contours.size());        //近似后的轮廓点集
	//vector<Rect> boundRect(contours.size());                      //包围点集的最小矩形vector
	//vector<Point2f>center(contours.size());                       //包围点集的最小圆形vector
	//vector<float>radius(contours.size());                         //包围点集的最小圆形半径vector
	for (int i = 0; i<contours.size(); i++)
	{
		Rect rect = boundingRect(contours[i]);
		//boundRect[i] = boundingRect(Mat(contours_poly[i]));             //计算并返回包围轮廓点集的最小矩形
		//minEnclosingCircle(contours_poly[i], center[i], radius[i]);     //计算并返回包围轮廓点集的最小圆形及其半径
		int area = contourArea(contours[i]);
		float ratioSize = area*1.0f / (rect.width*rect.height);
		float ratioSide = rect.width*1.0f / rect.height;
		//printf("i=%d,ratioSize=%f,ratioSide=%f,area=%d\n", i, ratioSize, ratioSide, area);
		//rectangle(openMat, rect, Scalar(0, 0, 255), 1, 1, 0);
		//cv::circle(dstImage, cvPoint(rect.x, rect.y), 6, cv::Scalar(255, 255, 255), 1);
		//putText(dstImage,to_string(i), cvPoint(rect.x, rect.y), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
		if (ratioSide<5 && ratioSide>0.2 && ratioSize>0.25 && area>50 && area<2000){
			drawContours(dstImage, contours, i, cv::Scalar(255, 0, 255), 1, 8, hierarchy);
			string str;
			stringstream tmp;
			tmp << i;
			tmp >> str;
			//putText(dstImage, "area:" + to_string(i), cvPoint(rect.x, rect.y), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
			putText(dstImage, "area:" + str, cvPoint(rect.x, rect.y), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
		}

	}

}

void skin_detect::boundaryDispose(Mat &srcImage){
	//转灰度图,yCrCb图
	int row = srcImage.rows;
	int col = srcImage.cols;
	for (int i = 0; i < srcImage.rows; i++)
	{
		uchar* pxvec1 = srcImage.ptr<uchar>(i);
		//三通道数据都在第一行依次排列，按照BGR顺序
		//依次赋值为1,img.at<cv::Vec3b>(j, i)[0];
		for (int j = 0; j < srcImage.cols; j++){
			if (i<3 || i>row - 4 || j<3 || j>col - 4){
				pxvec1[j] = 255;
			}

		}
	}

}

void skin_detect::colorspotsDetect(Mat src, Mat &image){
	Mat bw, yCrCb, yImage;
	vector<Mat> yCrCbG;
	image = src.clone();
	//image = cv::imread(srcImagePath);
	if (!image.data)
	{
		printf("Cannot Open!!");
	}
	bw = Mat::zeros(image.size(), CV_8UC1);
	//转灰度图,yCrCb图
	//cvtColor(image, gray, CV_BGR2GRAY);
	cvtColor(image, yCrCb, CV_BGR2YCrCb);
	split(yCrCb, yCrCbG);
	yImage = yCrCbG[0];
	float radioMean = 2.0f;
	threshold(yImage*radioMean, bw, 0, 255, CV_THRESH_OTSU);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, Size(7, 7));
	//cv::dilate(bw, bw, element);
	cv::erode(bw, bw, element);
	cv::dilate(bw, bw, element);
	cv::dilate(bw, bw, element);
	//处理边界上的3个像素
	boundaryDispose(bw);
	findContoursOperator(bw, image);
	//cv::imwrite(dstImagePath, image);
}

void skin_detect::getBaseicOperatorImages(Mat src, vector<int*>& images){

	Mat grayImg, colormapImg, streImg, gsImg, cannyImg, bwImg, invertGsImg, erodeImg, surfImg, spotsImg;
	int *srcArr = imageMergeOperator(src);
	images.push_back(srcArr);

	getGrayImage(src, grayImg);
	int *grayImgArr = imageMergeOperator(grayImg);
	images.push_back(grayImgArr);

	getColorMap(grayImg, colormapImg);
	int *colormapImgArr = imageMergeOperator(colormapImg);
	images.push_back(colormapImgArr);

	//getStreImage(grayImg, streImg);
	int *streImgArr = imageMergeOperator(streImg);
	images.push_back(streImgArr);

	//streImg->grayImg
	getGaussianBlur(grayImg, gsImg);
	int *gsImgArr = imageMergeOperator(gsImg);
	images.push_back(gsImgArr);

	getCannyImage(grayImg, cannyImg);
	int *cannyImgArr = imageMergeOperator(cannyImg);
	images.push_back(cannyImgArr);

	getBwImage(cannyImg, bwImg);
	int *bwImgArr = imageMergeOperator(bwImg);
	images.push_back(bwImgArr);

	getInvertGaussianBlur(gsImg, invertGsImg);
	int *invertGsImgArr = imageMergeOperator(invertGsImg);
	images.push_back(invertGsImgArr);

	getErodeOperator(bwImg, erodeImg);
	int *erodeImgArr = imageMergeOperator(erodeImg);
	images.push_back(erodeImgArr);

	//streImg->grayImg
	getSurfImage(grayImg, surfImg);
	int *surfImgArr = imageMergeOperator(surfImg);
	images.push_back(surfImgArr);

	colorspotsDetect(src, spotsImg);
	int *spotsImgArr = imageMergeOperator(spotsImg);
	images.push_back(spotsImgArr);
}