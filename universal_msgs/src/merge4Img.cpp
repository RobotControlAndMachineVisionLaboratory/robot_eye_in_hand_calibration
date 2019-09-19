#include <iostream>
#include <opencv2/opencv.hpp>
#include "snooker_recognizer/merge4Img.h"

using namespace std;
using namespace cv;

void merge4Img(Mat & dst, Mat &src1, Mat &src2, Mat &src3, Mat &src4)
{
	int rows = src1.rows + 1 + src2.rows;
	int cols = src1.cols + 1 + src2.cols;
	//CV_Assert(src1.type() == src2.type());
	dst.create(rows, cols, src1.type());
	src1.copyTo(dst(Rect(0, 0, src1.cols, src1.rows)));
	src2.copyTo(dst(Rect(src1.cols + 1, 0, src2.cols, src2.rows)));
	src3.copyTo(dst(Rect(0, src1.rows + 1, src3.cols, src3.rows)));
	src4.copyTo(dst(Rect(src1.cols + 1, src1.rows + 1, src4.cols, src4.rows)));
}