#ifndef _EYEHAND_H_
#define _EYEHAND_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream> //文件输入输出流
#include <stdio.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <universal_msgs/Command.h>

static const std::string OPENCV_WINDOW = "Cross";

using namespace cv;
using namespace std;

bool find(int i,std::vector<int> noise_);

Mat MatrixCombine(Mat upper, Mat lower);

double vec_norm(Mat a);

bool check(vector<pair<int,int>> points, pair<int,int> new_points);

double getDistance(double x,double y,Mat line);

Mat ransac(int numOfp,vector<double> samples, int k,double threshold_In, double& average);

void readRobotData(string robotFileName, vector<Mat>&RobotPose, vector<Mat>&RobotPosition,int nFrame,vector<int> failedIndex);

void saveEyeHand(Mat eyehandTranslation, Mat eyehandRotation,vector<int> failedIndex);

void homoTrans(Mat translation, Mat rotation, Mat& homo);

void homo2vector(Mat& translation, Mat& rotation, const Mat homo);

Mat ex_matrix(Mat rotation, Mat translation, int originIndex,float chessboardSize,int w,int h);

void varify(Mat EH_translation,
	Mat EyeHandRotation,
	vector<Mat> RobotPosition,
	vector<Mat> RobotPose,
	vector<Mat> cameraPosition,
	vector<Mat> cameraPose,double alpha,double beta, double Varify_X,double Varify_Y, double Varify_R,Mat& varify_robot_pose);

void eyeHandCalibraion(string cameraFileName,string robotFileName,vector<int> failedIndex);



#endif
