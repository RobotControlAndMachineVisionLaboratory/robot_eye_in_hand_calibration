#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "snooker_recognizer/merge4Img.h"
#include "snooker_recognizer/config.h"

using namespace cv;
using namespace std;

std::string pkg_loc = ros::package::getPath("snooker_recognizer");


int g_min_dist = 20;
int g_min_radius = 5;
int g_max_radius = 15;
int g_MORPH_OPEN = 9;

int g_min_canny_threshold = 20;
int g_max_canny_threshold = 60;
int g_canny_kernel_size = 3;

int g_x_min = 0;
int g_y_min = 0;
int g_x_max = 0;
int g_y_max = 0;

std::vector<std::vector<Vec3f> > StatisticalCircleCenters;
std::vector<Vec3f> res_circles;
int maxStatisticalCircleCenters = 21 ;
int CurrentStatisticalCircleCentersCount = 0;
int RemoveSimilarLocation_distance_threshold = 0;

bool original = 1;
bool RGB = 1;
bool HSV = 1;

void on_Trackbar_g_min_dist(int, void *);
void on_Trackbar_g_min_radius(int, void *);
void on_Trackbar_g_max_radius(int, void *);
void on_Trackbar_g_min_dist(int, void *){}
void on_Trackbar_g_min_radius(int, void *){}
void on_Trackbar_g_max_radius(int, void *){}
void on_Trackbar_g_MORPH_OPEN(int, void *)
{
	if(g_MORPH_OPEN%2){;}
	else{g_MORPH_OPEN++;}
}

void runCanny(cv::Mat &srcImage)
{
	Canny(srcImage, srcImage, 60, 20, 3, 1);
}

void runCanny(cv::Mat &srcImage, int threshold1, int threshold2, int kernel_size)
{
	Canny(srcImage, srcImage, threshold1, threshold2, kernel_size, 1);
	/*																								*\
	void cvCanny(  const CvArr* image,  CvArr* edges,  double threshold1,double threshold2,  int aperture_size=3);
	函数说明：
	第一个参数表示输入图像，必须为单通道灰度图。
	第二个参数表示输出的边缘图像，为单通道黑白图。
	第三个参数和第四个参数表示阈值，这二个阈值中当中的小阈值用来控制边缘连接，大的阈值用来控制强边缘的初始分割即如果一个像素的梯度大与上限值，
	则被认为是边缘像素，如果小于下限阈值，则被抛弃。如果该点的梯度在两者之间则当这个点与高于上限值的像素点连接时我们才保留，否则删除。
	第五个参数表示Sobel 算子大小，默认为3即表示一个3*3的矩阵。Sobel 算子与高斯拉普拉斯算子都是常用的边缘算子，详细的数学原理可以查阅专业书籍。
	\*																								*/

}

void runHC(cv::Mat &srcImage, vector<Vec3f> &circles)
{
	HoughCircles(srcImage, circles, CV_HOUGH_GRADIENT, 3, 20, 100, 40, 5, 15);
}

void runHC(cv::Mat &srcImage, vector<Vec3f> &circles, int min_dist, int min_radius, int max_radius)
{
	HoughCircles(srcImage, circles, CV_HOUGH_GRADIENT, 3, min_dist, 100, 40, min_radius, max_radius);

	/*																								*\
	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
	with the arguments:

	src_gray: Input image (grayscale)
	circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
	CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
	dp = 1: The inverse ratio of resolution
	min_dist = src_gray.rows/8: Minimum distance between detected centers
	param_1 = 200: Upper threshold for the internal Canny edge detector
	param_2 = 100*: Threshold for center detection.
	min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
	max_radius = 0: Maximum radius to be detected. If unknown, put zero as default
	\*																								*/
}

void drawHC(cv::Mat &srcImage, vector<Vec3f> &circles)
{
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(srcImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		circle(srcImage, center, radius, Scalar(155, 50, 255), 3, 8, 0);
	}
}

void runMORPH_OPEN(cv::Mat &srcImage)
{
	Mat element = getStructuringElement(MORPH_RECT, Size(g_MORPH_OPEN, g_MORPH_OPEN));
	morphologyEx(srcImage, srcImage, MORPH_OPEN, element);
}

int calculateDistance(cv::Point point1, cv::Point point2)
{
	return int(std::sqrt(std::pow((point1.x-point2.x),2)+std::pow((point1.y-point2.y),2)));
}

int calculateDistance(cv::Vec3f circle1, cv::Vec3f circle2)
{
	int res = 0;
	res = std::sqrt(std::pow((circle1[0]-circle2[0]),2) + std::pow((circle1[1]-circle2[1]),2));
	return res;
}

void RemoveSameLocation(std::vector<cv::Point>& centers, int maxMatchNum)
{
	for (int i = centers.size() - 1; i > 0; i--)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			if (calculateDistance(centers[i], centers[j]) == 0)
			{
				centers.erase(centers.begin() + i);
				break;
			}
		}
	}
}

cv::Vec3f RemoveSimilarLocation(std::vector<cv::Vec3f>& circles, int maxMatchNum, int _RemoveSimilarLocation_distance_threshold)
{
	float avg_x=0;
	float avg_y=0;
	float avg_r=0;
	float avg_d=0;
	for (int i = 0; i < circles.size(); ++i)
	{
		avg_x+=circles[i][0];
		avg_y+=circles[i][1];
		std::cout<<circles[i][0]<<" "<<circles[i][1]<<" "<<circles[i][2]<<std::endl;
	}
	avg_x/=circles.size();
	avg_y/=circles.size();
	std::vector<float> distance;

	for (int i = 0; i < circles.size(); ++i)
	{
		distance.push_back( calculateDistance( cv::Point(avg_x, avg_y),  cv::Point(circles[i][0], circles[i][1]) ) );
		avg_d+=distance[i];
	}
	avg_d/=circles.size();

	float res_x=0;
	float res_y=0;
	float res_r=0;
	int count_d =0;
	for (int i = 0; i < distance.size(); ++i)
	{
		if( distance[i] < avg_d || distance[i] < _RemoveSimilarLocation_distance_threshold)
		{
			res_x+=circles[i][0];
			res_y+=circles[i][1];
			res_r+=circles[i][2];
			count_d++;
			std::cout<<"Final circles: "<<res_x<<" "<<res_y<<" "<<res_r<<std::endl;
		}
	}
	if (count_d)
	{
		return cv::Vec3f(int(res_x/count_d), int(res_y/count_d), int(res_r/count_d));
	}
	else
	{
		std::cout<<"-----------------------------------------------------------------"<<
		"cv::Vec3f(0, 0, 0) detected! return cv::Vec3f(0, 0, 10) instead!"
		<<"-----------------------------------------------------------------"<<std::endl;
		return cv::Vec3f(0, 0, 10);
	}
}


void removeCircleOutofROI(std::vector<cv::Vec3f>& circles)
{
	std::vector<cv::Vec3f> res;
	for (size_t i = 0; i < circles.size(); ++i)
	{
		if ( (cvRound(circles[i][0]) >= g_x_min ) && (cvRound(circles[i][1]) >= g_y_min) &&
			(cvRound(circles[i][0]) <= g_x_max) && (cvRound(circles[i][1]) <= g_y_max) )
			res.push_back(circles[i]);
	}
	circles.clear();
	for (size_t i = 0; i < res.size(); ++i)
	{
		circles.push_back(res[i]);
	}
}



void Statistical_circles2res_circles(
	std::vector<std::vector<Vec3f> > &StatisticalCircleCenters,
	vector<Vec3f> &circles,
	std::vector<Vec3f> &res_circles,
	int &maxStatisticalCircleCenters,
	int &CurrentStatisticalCircleCentersCount,
	int &RemoveSimilarLocation_distance_threshold)
{
	if(CurrentStatisticalCircleCentersCount == 0)
	{
		std::cout<<"\n\nCurrentStatisticalCircleCentersCount = "<<CurrentStatisticalCircleCentersCount<< "; StatisticalCircleCenters size:"<<StatisticalCircleCenters.size()<<std::endl;
		StatisticalCircleCenters.clear();
		for (int i = 0; i < circles.size(); ++i)
		{
			std::vector<Vec3f> temp;
			temp.push_back(circles[i]);
			StatisticalCircleCenters.push_back(temp);
			std::cout<<StatisticalCircleCenters[i][0][0]<<" "<<StatisticalCircleCenters[i][0][1]<<" "<<StatisticalCircleCenters[i][0][2]<<std::endl;
		}
		std::cout<<"CurrentStatisticalCircleCentersCount = "<<CurrentStatisticalCircleCentersCount<< "; StatisticalCircleCenters size:"<<StatisticalCircleCenters.size()<<std::endl;
		CurrentStatisticalCircleCentersCount++;
	}
	else
	{
		std::cout<<"\nCurrentStatisticalCircleCentersCount = "<<CurrentStatisticalCircleCentersCount<< "; StatisticalCircleCenters size:"<<StatisticalCircleCenters.size()<<std::endl;

		for (int i = 0; i < circles.size(); ++i)
		{
			bool found=false;
			for (int j = 0; j < StatisticalCircleCenters.size(); ++j)
			{
				if (calculateDistance(circles[i], StatisticalCircleCenters[j][0])< g_min_radius)
				{
					StatisticalCircleCenters[j].push_back(circles[i]);
					std::cout<<StatisticalCircleCenters[j][0][0]<<" "<<StatisticalCircleCenters[j][0][1]<<" "<<circles[i][0]<<" "<<circles[i][1]<<std::endl;
					found=true;
					break;

				}
			}
			if (!found)
			{
				std::vector<cv::Vec3f> temp;
				temp.push_back(circles[i]);
				StatisticalCircleCenters.push_back(temp);
			}
		}

		std::cout<<"CurrentStatisticalCircleCentersCount = "<<CurrentStatisticalCircleCentersCount<< "; StatisticalCircleCenters size:"<<StatisticalCircleCenters.size()<<std::endl;
		CurrentStatisticalCircleCentersCount++;
	}
	std::cout<<"circles size:"<<circles.size()<<std::endl;
	std::cout<<"StatisticalCircleCenters size:"<<StatisticalCircleCenters.size()<<std::endl;
	for (int i = 0; i < StatisticalCircleCenters.size(); ++i)
	{
		std::cout<<"StatisticalCircleCenters[i] size: "<< i <<" "<<StatisticalCircleCenters[i].size()<<std::endl;
	}

	if (CurrentStatisticalCircleCentersCount == maxStatisticalCircleCenters)
	{
		std::cout<<"CurrentStatisticalCircleCentersCount = "<<CurrentStatisticalCircleCentersCount<< "; StatisticalCircleCenters size:"<<StatisticalCircleCenters.size()<<std::endl;
		std::cout<<"Final locations:\n"<<std::endl;

		res_circles.clear();
		CurrentStatisticalCircleCentersCount = 0;
		for (int i = 0; i < StatisticalCircleCenters.size(); ++i)
		{
			res_circles.push_back( RemoveSimilarLocation(StatisticalCircleCenters[i], 4, RemoveSimilarLocation_distance_threshold));

			std::cout<<res_circles[i]<<std::endl;
		}
	}

}



void process_image_once(cv::Mat& input_color)
{
	// Creating OpenCV Matrix from a color image
	Mat color(Size(640, 480), CV_8UC3);
	input_color.copyTo(color);

	cv::imshow("input_color_image", color);
	cv::waitKey(10);

	createTrackbar("g_min_dist", "input_color_image", &g_min_dist, Config::get<int>("max_g_min_dist"), on_Trackbar_g_min_dist);
	createTrackbar("g_min_radius", "input_color_image", &g_min_radius, Config::get<int>("max_g_min_radius"), on_Trackbar_g_min_radius);
	createTrackbar("g_max_radius", "input_color_image", &g_max_radius, Config::get<int>("max_g_max_radius"), on_Trackbar_g_max_radius);
	createTrackbar("g_MORPH_OPEN", "input_color_image", &g_MORPH_OPEN, Config::get<int>("max_g_MORPH_OPEN"), on_Trackbar_g_MORPH_OPEN);


	if (original)
	{
		Mat srcImage;
		color.copyTo(srcImage);

		cv::Mat midImage;
		srcImage.copyTo(midImage);
		cvtColor(midImage, midImage, CV_BGR2GRAY);

		Mat BlurImage;
		midImage.copyTo(BlurImage);
		GaussianBlur(BlurImage, BlurImage, Size(9, 9), 2, 2);

		Mat CannyImage;
		BlurImage.copyTo(CannyImage);
		runCanny(CannyImage);

		vector<Vec3f> circles;
		Mat HoughImage;
		srcImage.copyTo(HoughImage);

		runHC(CannyImage, circles, g_min_dist, g_min_radius, g_max_radius);

		removeCircleOutofROI(circles);
		drawHC(HoughImage, circles);
		Statistical_circles2res_circles(StatisticalCircleCenters,circles,res_circles,maxStatisticalCircleCenters,CurrentStatisticalCircleCentersCount,RemoveSimilarLocation_distance_threshold);

		// cv::Mat Statistical;
		// srcImage.copyTo(Statistical);
		// drawHC(Statistical, res_circles);
		// imshow("Statistical", Statistical);


		cv::Mat dst;
		cvtColor(BlurImage, BlurImage, CV_GRAY2BGR);
		cvtColor(CannyImage, CannyImage, CV_GRAY2BGR);
		merge4Img(dst, srcImage, BlurImage, CannyImage, HoughImage);
		imshow("original", dst);
		waitKey(1);
	}

	if (RGB)
	{
		Mat srcImageRGB;
		color.copyTo(srcImageRGB);
		vector<Mat> channels;
		vector<Mat> mbgr(3);
		split(srcImageRGB, channels);
		Mat B = channels.at(0);
		Mat G = channels.at(1);
		Mat R = channels.at(2);

		Mat bk1(srcImageRGB.size(), CV_8UC1, Scalar(0));
		Mat imageB(srcImageRGB.size(), CV_8UC3);
		mbgr[0] = B;
		mbgr[1] = bk1;
		mbgr[2] = bk1;
		merge(mbgr, imageB);

		Mat MORPH_OPEN_R;
		R.copyTo(MORPH_OPEN_R);
		runMORPH_OPEN(MORPH_OPEN_R);

		Mat BlurImage;
		MORPH_OPEN_R.copyTo(BlurImage);
		GaussianBlur(BlurImage, BlurImage, Size(9, 9), 2, 2);

		Mat CannyImage;
		BlurImage.copyTo(CannyImage);
		runCanny(CannyImage);

		vector<Vec3f> circles;
		Mat HoughImage;
		color.copyTo(HoughImage);

		runHC(CannyImage, circles, g_min_dist, g_min_radius, g_max_radius);

		removeCircleOutofROI(circles);
		drawHC(HoughImage, circles);
		Statistical_circles2res_circles(StatisticalCircleCenters,circles,res_circles,maxStatisticalCircleCenters,CurrentStatisticalCircleCentersCount,RemoveSimilarLocation_distance_threshold);


		cv::Mat dst;
		cvtColor(BlurImage, BlurImage, CV_GRAY2BGR);
		cvtColor(CannyImage, CannyImage, CV_GRAY2BGR);
		merge4Img(dst, srcImageRGB, BlurImage, CannyImage, HoughImage);
		imshow("RGB", dst);
		waitKey(1);

	}

	if (HSV)
	{
		Mat srcImageHSV;
		color.copyTo(srcImageHSV);

		Mat src_hsv;
		cvtColor(srcImageHSV, src_hsv, CV_BGR2HSV_FULL);
		//cvtColor(srcImageRGB, src_hsv, CV_BGR2HSV);

		vector<Mat> hsv_channels;
		split(src_hsv, hsv_channels);
		Mat H = hsv_channels.at(0);
		Mat S = hsv_channels.at(1);
		Mat V = hsv_channels.at(2);

		//imshow("H", H);
		//imshow("S", S);
		//imshow("V", V);

		Mat MORPH_OPEN_H;
		H.copyTo(MORPH_OPEN_H);
		runMORPH_OPEN(MORPH_OPEN_H);

		Mat BlurImage;
		MORPH_OPEN_H.copyTo(BlurImage);
		GaussianBlur(BlurImage, BlurImage, Size(9, 9), 2, 2);

		Mat CannyImage;
		BlurImage.copyTo(CannyImage);
		runCanny(CannyImage);

		vector<Vec3f> circles;
		Mat HoughImage;
		color.copyTo(HoughImage);

		runHC(CannyImage, circles, g_min_dist, g_min_radius, g_max_radius);

		drawHC(HoughImage, circles);

		Statistical_circles2res_circles(StatisticalCircleCenters,circles,res_circles,maxStatisticalCircleCenters,CurrentStatisticalCircleCentersCount,RemoveSimilarLocation_distance_threshold);


		cv::Mat dst;
		cvtColor(BlurImage, BlurImage, CV_GRAY2BGR);
		cvtColor(CannyImage, CannyImage, CV_GRAY2BGR);
		merge4Img(dst, srcImageHSV, BlurImage, CannyImage, HoughImage);
		imshow("HSV", dst);
		waitKey(1);

	}

	cv::Mat Statistical;
	input_color.copyTo(Statistical);
	drawHC(Statistical, res_circles);
	cv::rectangle(Statistical,cvPoint(g_x_min,g_y_min),cvPoint(g_x_max,g_y_max),CV_RGB(0,0,255),2);
	// C++: void rectangle(Mat& img, Point pt1,Point pt2,const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	imshow("Statistical", Statistical);
	cv::waitKey(1);

}

class SnookerRecognizer
{

private:

	//ROS
	ros::NodeHandle nh;
	ros::Subscriber image_sub;
	ros::Publisher  cmd_pub;
	cv::Mat color;

	void callBack_img(const sensor_msgs::ImageConstPtr& img_ptr);

public:
	SnookerRecognizer(string color_topic);
	~SnookerRecognizer();
};

SnookerRecognizer::SnookerRecognizer(std::string color_topic)
{
	cv::namedWindow("input_color_image", cv::WINDOW_NORMAL);
	cv::namedWindow("original", cv::WINDOW_NORMAL);
	cv::namedWindow("RGB", cv::WINDOW_NORMAL);
	cv::namedWindow("HSV", cv::WINDOW_NORMAL);
	cv::namedWindow("Statistical", cv::WINDOW_NORMAL);
	// cv::namedWindow("color_topic", cv::WINDOW_NORMAL);
	// cv::namedWindow("HoughImage", cv::WINDOW_NORMAL);
	image_sub = nh.subscribe(Config::get<std::string>("color_topic"),1,&SnookerRecognizer::callBack_img,this);
}

SnookerRecognizer::~SnookerRecognizer()
{
}

void SnookerRecognizer::callBack_img(const sensor_msgs::ImageConstPtr& img_ptr)
{
	//---------------------convert image msg--------------------------------
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	color = cv_ptr->image.clone();
	// cv::imshow("color_topic",color);
	// cv::waitKey(1);
	process_image_once(color);

}

int main(int argc, char** argv)
{
	Config::setParameterFile(pkg_loc+"/parameter.yml");
	g_min_dist = Config::get<int>("g_min_dist");
	g_min_radius = Config::get<int>("g_min_radius");
	g_max_radius = Config::get<int>("g_max_radius");
	g_MORPH_OPEN = Config::get<int>("g_MORPH_OPEN");

	g_x_min = Config::get<int>("x_min");
	g_y_min = Config::get<int>("y_min");
	g_x_max = Config::get<int>("x_max");
	g_y_max = Config::get<int>("y_max");


	RemoveSimilarLocation_distance_threshold = Config::get<int>("RemoveSimilarLocation_distance_threshold");

	ros::init(argc,argv,"snooker_recognizer_node_inside");
	SnookerRecognizer snookerRecognizer(Config::get<string>("color_topic"));

	while(ros::ok())
	{
		ros::spin();
	}

	cv::destroyAllWindows();
	return 0;
}