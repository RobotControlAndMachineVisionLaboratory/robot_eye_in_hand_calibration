#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void getRGBimage(Mat color)
{
	Mat srcImageRGB;
	color.copyTo(srcImageRGB);
	vector<Mat> channels;
	vector<Mat> mbgr(3);
	split(srcImageRGB, channels);
	Mat B = channels.at(0);
	Mat G = channels.at(1);
	Mat R = channels.at(2);

	Mat imageB(srcImageRGB.size(), CV_8UC3);
	Mat bk1(srcImageRGB.size(), CV_8UC1, Scalar(0));
	mbgr[0] = B;
	mbgr[1] = bk1;
	mbgr[2] = bk1;
	merge(mbgr, imageB);


	imshow("test", srcImageRGB);
	imshow("image Blue", imageB);
	imshow("B channels", B);
	imshow("G channels", G);
	imshow("R channels", R);
}


//Laplacian(V, V, CV_16S, 3, 1, 0, BORDER_DEFAULT);
//convertScaleAbs(V, V);
//imshow("【Laplacian边缘检测效果图V】", V);

//Sobel(V,V,CV_16S,1,1,3,1,1,BORDER_DEFAULT);
//convertScaleAbs(V, V);
//imshow("【Sobel边缘检测效果图V】", V);