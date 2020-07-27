//C++
#include <iostream>
#include <fstream>

//OpenCV
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Customed
#include "mkdir.hpp"
#include "config.h"

//universal_msgs for communication
#include <universal_msgs/Command.h>
#include <universal_msgs/RobotMsg.h>

using namespace std;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
universal_msgs::RobotMsg> MySyncPolicy;

static const std::string OPENCV_WINDOW = "Image Collector Window";
std::string pkg_loc = ros::package::getPath("eye_in_hand_calibration");



void TransMsg2Image(const sensor_msgs::ImageConstPtr &ImageMsg, cv::Mat& imageMat)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr=cv_bridge::toCvCopy(ImageMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
		return;
	}
	imageMat= cv_ptr->image.clone();
}


class ImageCollector
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image>* image_sub;
	message_filters::Subscriber<universal_msgs::RobotMsg>* robot_sub;
	message_filters::Synchronizer<MySyncPolicy>* sync;
	ros::Publisher pub_cmd;

	//robot pose to observe the scene
	std::vector<std::vector<float> > observe_pose;

	//robot pose and joint angle
	std::vector<float> robot_pose;
	std::vector<float> robot_joint;

	cv::Mat color;

	int cnt;
	int im_num;
	std::ofstream ofs_joint;
	std::ofstream ofs_cart;
	//call back to recieve image and ur data
	void callback(const sensor_msgs::ImageConstPtr& img_ptr,
		const boost::shared_ptr<const universal_msgs::RobotMsg>& robot_msg_ptr);

	//let the robot observe the scene
	void observeScene(universal_msgs::Command& cmd);

public:
	ImageCollector(int im_num,const char* robot_file,std::string topic_name);
	~ImageCollector();
	void displayImg();
};

ImageCollector::ImageCollector(int im_num_,const char* robot_file, std::string topic_name)
{
	observe_pose.clear();
	ofs_joint.open(pkg_loc + "/" + Config::get<string>("TeachedRobotJointFileName").c_str(), ios::trunc);
	ofs_cart.open(pkg_loc + "/" + Config::get<string>("TeachedRobotCartFileName").c_str(), ios::trunc);
	ofs_joint.close();
	ofs_cart.close();
	im_num =im_num_;


	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,
		topic_name, 1);

	robot_sub =  new message_filters::Subscriber<universal_msgs::RobotMsg>(nh, "jaka_pose", 1);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
		*image_sub, *robot_sub);

	sync->registerCallback(boost::bind(&ImageCollector::callback, this, _1, _2));

	pub_cmd = nh.advertise<universal_msgs::Command>("command", 1);
	robot_pose = std::vector<float>(6,0.0);
	robot_joint = std::vector<float>(6,0.0);

	cnt = 0;
	cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
}

ImageCollector::~ImageCollector()
{
	delete image_sub;
	delete robot_sub;
	delete sync;
	cv::destroyWindow(OPENCV_WINDOW);
	ofs_joint.close();
	ofs_cart.close();
}

void ImageCollector::callback(const sensor_msgs::ImageConstPtr& img_ptr,
	const boost::shared_ptr<const universal_msgs::RobotMsg>& robot_msg_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	color = cv_ptr->image.clone();

	robot_pose[0] = robot_msg_ptr->data.tool_vector[0];
	robot_pose[1] = robot_msg_ptr->data.tool_vector[1];
	robot_pose[2] = robot_msg_ptr->data.tool_vector[2];
	robot_pose[3] = robot_msg_ptr->data.tool_vector[3];
	robot_pose[4] = robot_msg_ptr->data.tool_vector[4];
	robot_pose[5] = robot_msg_ptr->data.tool_vector[5];

	robot_joint[0] = robot_msg_ptr->data.q_actual[0];
	robot_joint[1] = robot_msg_ptr->data.q_actual[1];
	robot_joint[2] = robot_msg_ptr->data.q_actual[2];
	robot_joint[3] = robot_msg_ptr->data.q_actual[3];
	robot_joint[4] = robot_msg_ptr->data.q_actual[4];
	robot_joint[5] = robot_msg_ptr->data.q_actual[5];
}

void ImageCollector::displayImg()
{
	while(ros::ok())
	{
		if(color.data)
		{
			cv::imshow(OPENCV_WINDOW,color);
			char ch = cv::waitKey(20);
			if(ch == '9')
			{
				// moveJ
				universal_msgs::Command cmd;
				cmd.type = 9;
				cmd.joint = observe_pose[cnt];
				cmd.speed = 20;
				cmd.acce = 0;
				pub_cmd.publish(cmd);
			}
			if(ch == '5')
			{
				// moveE
				universal_msgs::Command cmd;
				cmd.type = 5;
				cmd.pose = observe_pose[cnt];
				cmd.speed = 20;
				cmd.acce = 0;
				pub_cmd.publish(cmd);
			}
			else if(ch == 't')
			{
				std::stringstream ss; ss << cnt++;
				cv::imwrite(pkg_loc + "/" + Config::get<std::string>("imagefolder") + "/image"+ss.str()+".png", color);
				std::cout << "image" + ss.str() + " is saved!" << std::endl;
				ofs_joint.open(pkg_loc + "/" + Config::get<string>("TeachedRobotJointFileName").c_str(), ios::app);
				ofs_cart.open(pkg_loc + "/" + Config::get<string>("TeachedRobotCartFileName").c_str(), ios::app);

				ofs_cart << robot_pose[0] << " " << robot_pose[1] << " " << robot_pose[2] << " "
				<< robot_pose[3] << " " << robot_pose[4] << " " << robot_pose[5] << "\n";
				ofs_joint << robot_joint[0] << " " << robot_joint[1] << " " << robot_joint[2] << " "
				<< robot_joint[3] << " " << robot_joint[4] << " " << robot_joint[5] << "\n";

				ofs_joint.close();
				ofs_cart.close();

				if(cnt == im_num)
				{
					std::cout << "image collection finished!" << std::endl;
					ros::shutdown();
				}
			}
			else if(ch == 'o')
			{
				// io on
				universal_msgs::Command cmd;
				cmd.type = 3;
				cmd.io = 2;
				pub_cmd.publish(cmd);
			}
			else if(ch == 'f')
			{
				// io offf
				universal_msgs::Command cmd;
				cmd.type = 3;
				cmd.io = -2;
				pub_cmd.publish(cmd);
			}
			else if(ch == ' ')
			{
				// stop
				universal_msgs::Command cmd;
				cmd.type = 4;
				pub_cmd.publish(cmd);
			}
			else if(ch == 'd')
			{
				// move down and up
				universal_msgs::Command cmd;
				cmd.type = 5;
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0.05);
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0);
				cmd.delta_pose.push_back(0);
				cmd.speed = 2.0;
				cmd.acce = 1.0;
				pub_cmd.publish(cmd);
			}
			else
			{
			}
		}
		// end if image data
		ros::spinOnce();
	}
	// end of while
	exit(0);
}
// end of this function


int main(int argc, char** argv)
{
	Config::setParameterFile(pkg_loc + "/parameters/parameter.yml");
	createDirectory(pkg_loc + "/" + Config::get<std::string>("imagefolder") + "/");
	ros::init(argc, argv, "image_collector");
	ImageCollector ic(Config::get<int>("img_num_to_collect"), (pkg_loc+"/"+Config::get<std::string>("robotFileName")).c_str(),Config::get<std::string>("color_topic"));
	ic.displayImg();
	return 0;
}