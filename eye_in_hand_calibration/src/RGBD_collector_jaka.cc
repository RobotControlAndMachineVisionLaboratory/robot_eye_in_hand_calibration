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
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

//Customed
#include "mkdir.hpp"
#include "config.h"

//universal_msgs for communication
#include <universal_msgs/Command.h>
#include <universal_msgs/RobotMsg.h>

using namespace std;


typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2,
universal_msgs::RobotMsg> MySyncPolicy;

static const std::string OPENCV_WINDOW = "Image Collector Window";
std::string pkg_loc = ros::package::getPath("eye_in_hand_calibration");
boost::shared_ptr<pcl::visualization::CloudViewer> viewer;


template <typename T>
void TransMsg2Cloud(const sensor_msgs::PointCloud2& input, T& cloud)
{
	pcl::fromROSMsg(input, cloud);
}


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


void TransMsg2Depth(const sensor_msgs::ImageConstPtr &DepthMsg, cv::Mat& depthMat)
{
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(DepthMsg, DepthMsg->encoding);
	pCvImage->image.copyTo(depthMat);
}


class ImageCollector
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image>* image_sub;
	message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* cloud_sub;
	message_filters::Subscriber<universal_msgs::RobotMsg>* robot_sub;
	message_filters::Synchronizer<MySyncPolicy>* sync;
	ros::Publisher pub_cmd;

	//robot pose to observe the scene
	std::vector<std::vector<float> > observe_pose;

	//robot pose and joint angle
	std::vector<float> robot_pose;
	std::vector<float> robot_joint;

	cv::Mat color;
	cv::Mat depth;
	// PointXYZRGB
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	// PointXYZ
	// pcl::PointCloud<pcl::PointXYZ> cloud;

	int cnt;
	int im_num;
	//call back to recieve image and ur data
	void callback(const sensor_msgs::ImageConstPtr& img_ptr,
		const sensor_msgs::ImageConstPtr& depth_ptr,
		const sensor_msgs::PointCloud2ConstPtr& cloud_ptr,
		const boost::shared_ptr<const universal_msgs::RobotMsg>& robot_msg_ptr);

	//let the robot observe the scene
	void observeScene(universal_msgs::Command& cmd);

public:
	ImageCollector(int im_num,const char* robot_file,std::string topic_name);
	~ImageCollector();
	void displayImg();
};

ImageCollector::ImageCollector(int im_num_, const char* robot_file, std::string topic_name)
{
	observe_pose.clear();
	std::ifstream fin(robot_file);
	std::string temp;
	im_num = im_num_;
	while(getline(fin, temp))
	{
		std::vector<float> temp_pose = std::vector<float>(6,0.0);
		sscanf(temp.c_str(), "%f%f%f%f%f%f",&temp_pose[0],
			&temp_pose[1],&temp_pose[2],&temp_pose[3],&temp_pose[4],&temp_pose[5]);
		for(int i = 0; i < 6; i++){
			std::cout << temp_pose[i] << " ";
		}
		std::cout << '\n';
		observe_pose.push_back(temp_pose);
	}
	if (im_num> observe_pose.size())
	{
		std::cout << "Not Enough observe pose in ${robotFileName}.txt" << std::endl;
		exit(-1);
	}
	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, Config::get<std::string>("color_topic"), 1);
	depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, Config::get<std::string>("depth_topic"), 1);
	cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, Config::get<std::string>("cloud_topic"), 1);
	robot_sub = new message_filters::Subscriber<universal_msgs::RobotMsg>(nh, Config::get<std::string>("robot_topic"), 1);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
		*image_sub, *depth_sub, *cloud_sub, *robot_sub);

	sync->registerCallback(boost::bind(&ImageCollector::callback, this, _1, _2, _3, _4));

	pub_cmd = nh.advertise<universal_msgs::Command>(Config::get<std::string>("publisher_command_topic"), 1);
	robot_pose = std::vector<float>(6,0.0);
	robot_joint = std::vector<float>(6,0.0);

	cnt = 0;
	cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
}

ImageCollector::~ImageCollector()
{
	delete image_sub;
	delete depth_sub;
	delete cloud_sub;
	delete robot_sub;
	delete sync;
	cv::destroyWindow(OPENCV_WINDOW);
}

void ImageCollector::callback(
	const sensor_msgs::ImageConstPtr& img_ptr,
	const sensor_msgs::ImageConstPtr& depth_ptr,
	const sensor_msgs::PointCloud2ConstPtr& cloud_ptr,
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

	TransMsg2Depth(depth_ptr, depth);
	TransMsg2Cloud(*cloud_ptr, cloud);
	if(! viewer->wasStopped()) viewer->showCloud(cloud.makeShared());

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
				cmd.speed = Config::get<float>("moveSpeed");
				cmd.acce = 0;
				pub_cmd.publish(cmd);
			}
			if(ch == '5')
			{
				// moveE
				universal_msgs::Command cmd;
				cmd.type = 5;
				cmd.pose = observe_pose[cnt];
				cmd.speed = Config::get<float>("moveSpeed");
				cmd.acce = 0;
				pub_cmd.publish(cmd);
			}
			else if(ch == 't')
			{
				std::stringstream ss; ss << cnt++;
				cv::imwrite(pkg_loc + "/" + Config::get<std::string>("imagefolder") + "/image"+ss.str()+".png", color);
				std::cout << "image" + ss.str() + " is saved!" << std::endl;
				cv::imwrite(pkg_loc + "/" + Config::get<std::string>("imagefolder") + "/depth"+ss.str()+".png", depth);
				std::cout << "depth" + ss.str() + " is saved!" << std::endl;
				if(pcl::io::savePLYFile(pkg_loc + "/" + Config::get<std::string>("imagefolder") + "/cloud"+ss.str()+".ply", cloud, true) == 0)
					std::cout << "cloud"+ss.str()+".ply"<<" is saved!" << std::endl;
				if(cnt == im_num)
				{
					std::cout << "image collection finished!" << std::endl;
					ros::shutdown();
					exit(-1);
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
				cmd.speed = Config::get<float>("moveSpeed");
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
	ros::init(argc, argv, "RGBD_image_collector");
	boost::shared_ptr<pcl::visualization::CloudViewer> v(new pcl::visualization::CloudViewer("CloudViewer"));
	viewer = v;
	ImageCollector ic(Config::get<int>("img_num_to_collect"), (pkg_loc+"/"+Config::get<std::string>("robotFileName")).c_str(),Config::get<std::string>("color_topic"));
	ic.displayImg();
	return 0;
}