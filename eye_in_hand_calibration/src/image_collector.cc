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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Customed
#include "eye_hand_calibration/config.h"

//universal_msgs for communication
#include <universal_msgs/Command.h>
#include <universal_msgs/RobotMsg.h>

using namespace std;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
universal_msgs::RobotMsg> MySyncPolicy;

static const std::string OPENCV_WINDOW = "Image Collector Window";
std::string pkg_loc = ros::package::getPath("eye_hand_calibration");



class ImageCollector
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image>* image_sub;
	message_filters::Subscriber<universal_msgs::RobotMsg>* ur_sub;
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
	//call back to recieve image and ur data
	void callback(const sensor_msgs::ImageConstPtr& img_ptr,
		const boost::shared_ptr<const universal_msgs::RobotMsg>& ur_ptr);

	//let the robot observe the scene
	void observeScene(universal_msgs::Command& cmd);

public:
	ImageCollector(int im_num,const char* robot_file,std::string topic_name);
	~ImageCollector();
	void displayImg();
};

ImageCollector::ImageCollector(int im_num_,const char* robot_file,std::string topic_name)
{
	observe_pose.clear();
	std::ifstream fin(robot_file);
	std::string temp;
	im_num = im_num_;
	while(getline(fin, temp))
	{
		std::vector<float> temp_pose = std::vector<float>(6,0.0);
		sscanf(temp.substr(2).c_str(), "%f%*s%f%*s%f%*s%f%*s%f%*s%f",&temp_pose[0],
			&temp_pose[1],&temp_pose[2],&temp_pose[3],&temp_pose[4],&temp_pose[5]);
		for(int i = 0; i < 6; i++){
			std::cout << temp_pose[i] << " ";
		}
		std::cout << '\n';
		observe_pose.push_back(temp_pose);
	}
	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,
		topic_name, 1);

	ur_sub =  new message_filters::Subscriber<universal_msgs::RobotMsg>(nh, "ur", 1);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
		*image_sub, *ur_sub);

	sync->registerCallback(boost::bind(&ImageCollector::callback, this, _1, _2));

	pub_cmd = nh.advertise<universal_msgs::Command>("command", 1000);
	robot_pose = std::vector<float>(6,0.0);
	robot_joint = std::vector<float>(6,0.0);

	cnt = 0;
	cv::namedWindow(OPENCV_WINDOW);
}

ImageCollector::~ImageCollector()
{
	delete image_sub;
	delete ur_sub;
	delete sync;
	cv::destroyWindow(OPENCV_WINDOW);
}

void ImageCollector::callback(const sensor_msgs::ImageConstPtr& img_ptr,
				  const boost::shared_ptr<const universal_msgs::RobotMsg>& ur_ptr)   //callback
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

	robot_pose[0] = ur_ptr->data.tool_vector[0];
	robot_pose[1] = ur_ptr->data.tool_vector[1];
	robot_pose[2] = ur_ptr->data.tool_vector[2];
	robot_pose[3] = ur_ptr->data.tool_vector[3];
	robot_pose[4] = ur_ptr->data.tool_vector[4];
	robot_pose[5] = ur_ptr->data.tool_vector[5];

	robot_joint[0] = ur_ptr->data.q_actual[0];
	robot_joint[1] = ur_ptr->data.q_actual[1];
	robot_joint[2] = ur_ptr->data.q_actual[2];
	robot_joint[3] = ur_ptr->data.q_actual[3];
	robot_joint[4] = ur_ptr->data.q_actual[4];
	robot_joint[5] = ur_ptr->data.q_actual[5];
}

void ImageCollector::displayImg()
{
	while(ros::ok())
	{
		if(color.data)
		{
			cv::imshow(OPENCV_WINDOW,color);
			char ch = cv::waitKey(20);
			if(ch == '2'){
			 //movej
				universal_msgs::Command cmd;
				cmd.type = 2;
				cmd.pose = observe_pose[cnt];
				cmd.speed = 0.5;
				cmd.acce = 0.5;
				pub_cmd.publish(cmd);
			}
			else if(ch == 't'){
				std::stringstream ss; ss << cnt++;
				cv::imwrite("parameters/image/image"+ss.str()+".png", color);
				std::cout << "image" + ss.str() + " is saved!" << std::endl;
				if(cnt == im_num){
					std::cout << "image collection finished!" << std::endl;
					exit(0);
				}
			}
			else if(ch == 'o'){
			// io on
				universal_msgs::Command cmd;
				cmd.type = 3;
				cmd.io = 2;
				pub_cmd.publish(cmd);
			}
			else if(ch == 'f'){
			// io offf
				universal_msgs::Command cmd;
				cmd.type = 3;
				cmd.io = -2;
				pub_cmd.publish(cmd);
			}
			else if(ch == ' '){
			//stop
				universal_msgs::Command cmd;
				cmd.type = 4;
				pub_cmd.publish(cmd);
			}
			else if(ch == 'd'){
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
			else{
				universal_msgs::Command cmd;
				cmd.type = 0;
				pub_cmd.publish(cmd);
			}
		}
		// end if image data
		//broadcastPose();
		ros::spinOnce();
	}
	// end of while
	exit(0);
}
// end of this function


int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibration_image_collector");
	Config::setParameterFile(pkg_loc + "/parameters/parameter.yml");
	ImageCollector ic(Config::get<int>("img_num"),Config::get<std::string>("robotFileName").c_str(),Config::get<std::string>("color_topic"));
	ic.displayImg();
	return 0;
}