////-------------------------------------------------
////  this .cc is used for draw a cross at the center
////  of the image(principle point) to varify the accuracy
////  of eye-hand calibration. It starts a ros node and subscribe
////  the realsense sr300 color topic.
////
////  Author: LI KAI
////  Date: 2017.10.27
////--------------------------------------------------
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string.h>
//Eigen
#include <Eigen/Eigen>

#include <universal_msgs/Command.h>
#include "config.h"

static const std::string OPENCV_WINDOW = "Cross";

using std::cout;
using std::endl;
template <typename T>
void getAxisAngle(T &x, T &y, T &z, T &angle)
{
	angle = sqrt(x*x+y*y+z*z);
	x=x/angle;
	y=y/angle;
	z=z/angle;

}

class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	cv::Point2f centre;
	ros::Publisher pub;
	cv::Mat img;
	std::vector<cv::Mat> varify_poses; //varify pose vec
	int vari_cnt;
	int varify_total;
public:
	ImageConverter(std::string camera_file, std::string para_file)
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(Config::get<std::string>("color_topic"), 1,
			&ImageConverter::imageCb, this);
		cv::namedWindow(OPENCV_WINDOW);

		//read intrinc parameters
		cv::FileStorage fs(camera_file,cv::FileStorage::READ);
		cv::Mat camera_matrix(3,3,CV_64FC1);
		fs["camera_matrix"] >> camera_matrix;
		fs.release();
		centre.x = camera_matrix.at<double>(0,2);
		centre.y = camera_matrix.at<double>(1,2);

		//read pre-defined varify poses
		cv::FileStorage fs_1(para_file,cv::FileStorage::READ);
		fs_1["varify_num"] >> varify_total;

		for(int i = 0; i < varify_total; i++)
		{
			std::stringstream ss; ss << i+1;
			cv::Mat varify_p;
			fs_1["varify_pose_" + ss.str()] >> varify_p;
			cout << varify_p <<endl;
			varify_poses.push_back(varify_p);
		}
		fs_1.release();
		vari_cnt = 0;


		bool IsRotationInRPY = Config::get<int>("IsRotationInRPY");
		std::cout << "IsRotationInRPY？ " << IsRotationInRPY << std::endl;
		double Scale2Radian =1.0;
		double Scale2Millimeter  = 1000.0;
		// double Scale2Millimeter  = Config::get<double>("Scale2Millimeter");
		if (!Config::get<bool>("IsRotationInRadian"))
			Scale2Radian = 180.0/CV_PI;
		std::cout << "IsRotationInRadian " << Config::get<bool>("IsRotationInRadian") << std::endl;
		std::cout << "Scale2Radian: " << Scale2Radian << std::endl;
		for (int i = 0; i < varify_poses.size(); ++i)
		{
			if (IsRotationInRPY)
			{
				std::cout << varify_poses[i].at<double>(3) << " " << varify_poses[i].at<double>(4) << " " << varify_poses[i].at<double>(5) << std::endl;
				double x,y,z,angle =0;
				x=varify_poses[i].at<double>(3);
				y=varify_poses[i].at<double>(4);
				z=varify_poses[i].at<double>(5);

				getAxisAngle(x,y,z,angle);

				Eigen::Matrix3d rotation_matrix3d = Eigen::Matrix3d::Identity();
				Eigen::AngleAxisd rotation_vector(angle, Eigen::Vector3d(x,y,z));
				std::cout << "rotation vector =\n" << rotation_vector.axis()*rotation_vector.angle() << std::endl;
				rotation_matrix3d = rotation_vector.toRotationMatrix();
				Eigen::Vector3d euler_angles_of_in_mat = rotation_matrix3d.eulerAngles(2,1,0);

				varify_poses[i].at<double>(0) = varify_poses[i].at<double>(0)*Scale2Millimeter;
				varify_poses[i].at<double>(1) = varify_poses[i].at<double>(1)*Scale2Millimeter;
				varify_poses[i].at<double>(2) = varify_poses[i].at<double>(2)*Scale2Millimeter;

				varify_poses[i].at<double>(3) = euler_angles_of_in_mat(0)*Scale2Radian;
				varify_poses[i].at<double>(4) = euler_angles_of_in_mat(1)*Scale2Radian;
				varify_poses[i].at<double>(5) = euler_angles_of_in_mat(2)*Scale2Radian;
				std::cout << varify_poses[i].at<double>(3) << " " << varify_poses[i].at<double>(4) << " " << varify_poses[i].at<double>(5) << std::endl;
			}

			std::cout<< "\n";
			for (int j = 0; j < 6; ++j)
			{
				std::cout<< varify_poses[i].at<double>(j) << " ";
			}
		}

		//initialize command publisher
		pub = nh_.advertise<universal_msgs::Command>("command", 1000);
	}
	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}
	void imageCb(const sensor_msgs::ImageConstPtr& msg)  //call back
	{
		//cout << "callback" << endl;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		img = cv_ptr->image.clone();
		// cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		// cv::waitKey(1);
	}

	void ImgController()
	{
		while(ros::ok())
		{
			universal_msgs::Command cmd;
			cmd.type = 0;
			if(img.data)
			{
				//// Draw a cross at the image center
				////      a
				////    d---b
				///       c
				cv::Point2f a(centre.x,centre.y - 20);
				cv::Point2f b(centre.x + 20,centre.y);
				cv::Point2f c(centre.x,centre.y + 20);
				cv::Point2f d(centre.x - 20,centre.y);
				cv::line(img, a, c, cv::Scalar(0,0,255));
				cv::line(img, d, b, cv::Scalar(0,0,255));

				cv::imshow(OPENCV_WINDOW,img);
				char ch = cv::waitKey(30);
				if(ch == ' ')
				{
					if(vari_cnt < varify_total)
					{
						cmd.type = 5;
						cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,0));
						cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,1));
						cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,2));
						cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,3));
						cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,4));
						cmd.pose.push_back(varify_poses[vari_cnt++].at<double>(0,5));
						cmd.speed = 10;
						cmd.acce = 0.5;
						cmd.time = 0;

						// cmd.type = 2;
						// cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,0));
						// cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,1));
						// cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,2));
						// cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,3));
						// cmd.pose.push_back(varify_poses[vari_cnt].at<double>(0,4));
						// cmd.pose.push_back(varify_poses[vari_cnt++].at<double>(0,5));
						// cmd.speed = 0.1;
						// cmd.acce = 0.5;
						// cmd.time = 0;
					}
					else
						cout << "varify finished!" << endl;
				}
				else if(ch == '1')
			{   //movel
				cmd.type = 1;
				cmd.pose.push_back(-0.619);
				cmd.pose.push_back(0.4);
				cmd.pose.push_back(0.6);
				cmd.pose.push_back(3.01);
				cmd.pose.push_back(0.25);
				cmd.pose.push_back(-0.18);
			}
			else if(ch == '2')
			{  //movej
				cmd.type = 2;
				cmd.pose.push_back(-0.619);
				cmd.pose.push_back(0.4);
				cmd.pose.push_back(0.6);
				cmd.pose.push_back(3.01);
				cmd.pose.push_back(0.25);
				cmd.pose.push_back(-0.18);
				cmd.speed = 0.1;
				cmd.acce = 0.5;
				cmd.time = 1;
			}
			else if(ch == 'o'){ // io on
				cmd.type = 3;
				cmd.io = 2;
			}
			else if(ch == 'f'){ // io off
				cmd.type = 3;
				cmd.io = -2;
			}
			else if(ch == 's'){  //stop
				cmd.type = 4;
			}
			else if(ch == 'd'){ // move down and up
				cmd.type = 5;
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0.1);
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0);
			}
			else{
				cmd.type = 0;
			}
		}
		pub.publish(cmd);
		ros::spinOnce();
	}
}
};

int main(int argc, char** argv)
{
	Config::setParameterFile(argv[1]);
	ros::init(argc, argv, "cross");
	ImageConverter ic(Config::get<std::string>("outputFileName"),Config::get<std::string>("variFileName"));
	ic.ImgController();
	ros::spin();
	return 0;
}