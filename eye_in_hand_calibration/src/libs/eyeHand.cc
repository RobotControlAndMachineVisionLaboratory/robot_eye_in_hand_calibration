////-----------------------------------------------------------
////  This .cc is used for robot eye-hand calibration. It
////  reads robot pose and camera extrinsic and intrinsic
////  parameters to perform eye-hand calibration. Camera
////  calibration is done in another file.
////
////  Author: LI KAI
////  Date: 2017.10.27
////-----------------------------------------------------------

#include "eyeHand.h"
#include "config.h"
//Eigen
#include <Eigen/Eigen>

bool find(int i,std::vector<int> noise_)
{
	int j = 0;
	while(j < noise_.size() )
	{
		if( i == noise_[j])
			return true;
		j++;
	}
	return false;
};

/*----------------------------
 * 功能 : 矩阵拼接
 *----------------------------
 * 函数 : MatrixCombine
 * 返回 : 拼接后的矩阵
 *
 * 参数 : upper		        [in]	待拼接的上部矩阵
 * 参数 : lower   	        [in]	待拼接的下部矩阵
 * 参数 : combinedMat   	[in]	拼接后的矩阵
 */
Mat MatrixCombine(Mat upper, Mat lower)
{
	int c_new;
	int r_new;
	c_new = upper.cols ;				//列数不变
	r_new = upper.rows + lower.rows;    //行数相加
	Mat combinedMat(Size(c_new,r_new),CV_64FC1);
	for(int i = 0; i < r_new ;i++)
	{
		double* dataCombined = combinedMat.ptr<double>(i);

		if(i < (upper.rows))
		{
			double* data_upper = upper.ptr<double>(i);
			for(int j = 0;j < c_new;j ++)
			{
				dataCombined[j] = data_upper[j];
			}
		}
		else
		{
			double* data_lower = lower.ptr<double>(i-(upper.rows));
			for(int j = 0;j < c_new;j ++)
			{
				dataCombined[j] = data_lower [j];
			}
		}
	}
	return combinedMat;
}

/*----------------------------
 * 功能 : 求3元向量长度
 *----------------------------
 * 函数 : vec_norm
 * 返回 : 输入向量长度
 *
 * 参数 : a		        [in]	输入向量
 * 参数 :    	        [out]	向量长度
 */
double vec_norm(Mat a)
{
	return sqrt( a.at<double>(0)*a.at<double>(0) + a.at<double>(1) * a.at<double>(1) + a.at<double>(2) * a.at<double>(2));
}

/*----------------------------
 * 功能 : 判断生成的随机数点，是否重复
 *----------------------------
 * 函数 : check
 * 返回 : bool，true表示重复，false表示不重复
 *
 * 参数 : index     	   [in]	已生成的点下标，vector
 * 参数 : k         	   [in]	需要比较的点
  */
bool check(vector<pair<int,int>> points, pair<int,int> new_points)
{
	for(int i = 0; i < points.size(); i++)
	{
		if( new_points.first == points[i].first &&  new_points.second == points[i].second)
			return true;
		else if(new_points.second == points[i].first &&  new_points.first == points[i].second)
			return true;
	}
	return false;
}
/*----------------------------
 * 功能 : 输入样本点，返回拟合直线参数
 *----------------------------
 * 函数 : getLine
 * 返回 : 拟合直线参数：a0 a1
 *
 * 参数 : samplePoints		[in]	样本，vector
 * 参数 : sampleWeights		[in]	样本权重，vector
 * 参数 : Mat(2*1)	        [out]	a0 a1,存储在Mat中
 */
template<class T>
Mat getLine(vector<T> samplePoints,vector<T> sampleWeights = vector<T>(1) )
{
	Mat left = (Mat_<T>(2,2) << 0, 0, 0, 0);
	Mat right = (Mat_<T>(2,1) << 0, 0);

	if(sampleWeights.size() == 1)
		sampleWeights = vector<T>(samplePoints.size(),1);

	for(int i = 0; i < samplePoints.size();i ++)
	{
		left.at<T>(0,0) += sampleWeights[i];
		left.at<T>(0,1) += (sampleWeights[i]) * i;
		left.at<T>(1,0) += (sampleWeights[i]) * i;
		left.at<T>(1,1) += (sampleWeights[i]) * i * i;
		right.at<T>(0,0) += (sampleWeights[i]) * (samplePoints[i]);
		right.at<T>(1,0) += (sampleWeights[i]) * (samplePoints[i]) * i;

	}
	return left.inv() * right;
}

/*----------------------------
 * 功能 : 计算点到直线距离
 *----------------------------
 * 函数 : getDistance
 * 返回 : 点到直线距离
 *
 * 参数 : y		           [in]	点y坐标
 * 参数 : x		           [in]	点x坐标（vector下标）
 * 参数 : line	           [in]	直线参数（Mat）
 * 参数 : distance	       [out]点到直线距离
 */
double getDistance(double x,double y,Mat line)
{
	return abs(line.at<double>(1) * x - y + line.at<double>(0)) / sqrt(line.at<double>(1) * line.at<double>(1) +1);
}
/*----------------------------
 * 功能 : Ransac
 *----------------------------
 * 函数 : ransac
 * 返回 : 拟合直线参数
 *
 * 参数 : numOfp		   [in]	观测样本点数
 * 参数 : samples		   [in]	样本vector
 * 参数 : k  	           [in]	迭代次数
 * 参数 : threshold_In	   [in] 判断为内点的阈值
 * 参数 : average	       [in] 内点取值均值，引用
 * 参数 : Result	       [in] 拟合直线参数（Mat）
 */

Mat ransac(int numOfp,vector<double> samples, int k,double threshold_In, double& average)
{
	RNG rng;						                     //随机数生成器
	vector<double> weights(numOfp,1);
//	double threshold_In = 1;							//判断为内点的距离阈值
//	int k = 200;										//迭代次数
	int num_ofInliners = 0;   						    //内点数目
	Mat Result = (Mat_<double>(2,1) << 0, 0);			//迭代结果直线

	if (samples.size() < 2)
	{
		cout<<"Too little samples!"<<endl;
		return Result;
	}

	vector<pair<int,int>> randomP_array;         //随机点对容器
	vector<int> p1_array;                       //随机选取点容器 1
	vector<int> p2_array;                       //随机选取点容器 2

	for(int i = 0; i < k; i++)
	{
		// 随机选取两点，计算方程
		int p1 = 0;
		int p2 = 0;
		pair<int,int> p(0,0);
		while(p1 == p2 || check(randomP_array,p))
		{
 			p1 = rng.uniform(0,numOfp);  //随机选取两个点（下标）
 			p2 = rng.uniform(0,numOfp);
 		}
 		p.first = p1;
 		p.second = p2;
 		randomP_array.push_back(p);
 		vector<double> linePara;
 		vector<double> lineWeight;
 		linePara.push_back(samples[p1]);
 		linePara.push_back(samples[p2]);
 		lineWeight.push_back(1);
 		lineWeight.push_back(1);
 		Mat x = (Mat_<double>(2,1) << 0, 0);
 		x = getLine(linePara,lineWeight);

		// 计算点到直线距离，判断是否为内点
		vector<double> inliners;          //内点容器
		for(int j = 0; j < numOfp; j++)
		{
			if(getDistance(j,samples[j],x) < threshold_In)
				inliners.push_back(samples[j]);
		}
		// 根据内点重新估计模型
		Mat lineTemp = getLine(inliners);
		if(inliners.size() > num_ofInliners)
		{
			Result = lineTemp;
			num_ofInliners = inliners.size();
			//计算内点取值均值
			double sum = 0;
			for (vector<double>::iterator iter = inliners.begin();iter != inliners.end();iter++)
			{
				sum += *iter;
			}
			average = sum / inliners.size();;
		}
	}
	return Result;
}

void readRobotData(string robotFileName, vector<Mat>&RobotPose, vector<Mat>&RobotPosition,int nFrame,vector<int> failedIndex)
{
	RobotPose.clear();
	RobotPosition.clear();
	ifstream fin(robotFileName);
	const int LINE_LENGTH = 200;
	char str[LINE_LENGTH];
	int k = 2;					//读取起始地址  p and ( occupies the 1st and 2nd places
	int poseLength = 0;
	int valid_pose = 0;
	while(fin.getline(str,LINE_LENGTH))
	{
		if(find(valid_pose,failedIndex))  //pay attention to failed images!!!
		{
			valid_pose ++;
			continue;
		}
		Mat RobotPose_data(Size(1,3),CV_64FC1);
		Mat RobotPosition_data(Size(1,3),CV_64FC1);
		k = 2;
		poseLength = 0;
		double* pose = new double[6];    //位姿数组
		for(int i = 0;i < 6; i++ )
		{
			char buffer[200];
			while(str[k] != ',')
			{
				buffer[k-2-poseLength] = str[k];
				k++;
				if(str[k] == ')')
					break;
			}
			poseLength = k-1;
			switch (i)
			{	case 0:
				RobotPosition_data.at<double>(0) = atof(buffer) * 1000.0;  //注意机器人输出的位置，单位是m
				break;
				case 1:
				RobotPosition_data.at<double>(1) = atof(buffer) * 1000.0;
				break;
				case 2:
				RobotPosition_data.at<double>(2) = atof(buffer) * 1000.0;
				break;
				case 3:
				RobotPose_data.at<double>(0) = atof(buffer);
				break;
				case 4:
				RobotPose_data.at<double>(1) = atof(buffer);
				break;
				case 5:
				RobotPose_data.at<double>(2) = atof(buffer);
				break;
			}
			k++;
		}
		RobotPosition.push_back(RobotPosition_data);
		RobotPose.push_back(RobotPose_data);
		delete[] pose;
		valid_pose ++;
		if(RobotPose.size() == nFrame - failedIndex.size())
			break;
	}
}


void readRobotDataInFileStream(string robotFileName, vector<Mat>&RobotPose, vector<Mat>&RobotPosition,int nFrame,vector<int> failedIndex)
{
	std::cout << "read robot pose data from: " << robotFileName << std::endl;

	std::ifstream ifs;
	ifs.open (robotFileName);
	if (ifs.is_open())
	{
		std::cout << "robot pose data reading ..." << std::endl;
	}
	else
	{
		std::cout << "Error opening file";
	}


	RobotPose.clear();
	RobotPosition.clear();

	bool IsRotationInRPY = Config::get<int>("IsRotationInRPY");
	std::cout << "IsRotationInRPY？ " << IsRotationInRPY << std::endl;
	double Scale2Radian =1.0;
	double Scale2Millimeter  = Config::get<double>("Scale2Millimeter");
	if (!Config::get<bool>("IsRotationInRadian"))
	{
		Scale2Radian = CV_PI/180.0;
	}
	for (int i = 0; i < nFrame; ++i)
	{
		if(find(i,failedIndex))
			continue;

		Mat RobotPose_data(Size(1,3),CV_64FC1);
		Mat RobotPosition_data(Size(1,3),CV_64FC1);
		for(int i = 0;i < 6; i++ )
		{
			double tmp = 0;
			ifs >> tmp;
			switch (i)
			{	case 0:
				RobotPosition_data.at<double>(0) = tmp * Scale2Millimeter;
				break;
				case 1:
				RobotPosition_data.at<double>(1) = tmp * Scale2Millimeter;
				break;
				case 2:
				RobotPosition_data.at<double>(2) = tmp * Scale2Millimeter;
				break;
				case 3:
				RobotPose_data.at<double>(0) = tmp * Scale2Radian;
				break;
				case 4:
				RobotPose_data.at<double>(1) = tmp * Scale2Radian;
				break;
				case 5:
				RobotPose_data.at<double>(2) = tmp * Scale2Radian;
				break;
			}
		}

		if (IsRotationInRPY)
		{
			std::cout << RobotPose_data.at<double>(0) << " " << RobotPose_data.at<double>(1) << " " << RobotPose_data.at<double>(2) << std::endl;
			Eigen::Vector3d current_ea(RobotPose_data.at<double>(0), RobotPose_data.at<double>(1), RobotPose_data.at<double>(2));
			Eigen::AngleAxisd rotation_vector;
			rotation_vector =
			Eigen::AngleAxisd(current_ea[2], Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(current_ea[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(current_ea[0], Eigen::Vector3d::UnitX());
			RobotPose_data.at<double>(0) = rotation_vector.axis()(0)*rotation_vector.angle();
			RobotPose_data.at<double>(1) = rotation_vector.axis()(1)*rotation_vector.angle();
			RobotPose_data.at<double>(2) = rotation_vector.axis()(2)*rotation_vector.angle();
			std::cout << RobotPose_data.at<double>(0) << " " << RobotPose_data.at<double>(1) << " " << RobotPose_data.at<double>(2) << std::endl;

			// std::cout << rotation_vector.axis()(0)*rotation_vector.angle() << " "
			// << rotation_vector.axis()(1)*rotation_vector.angle()
			// << " " << rotation_vector.axis()(2)*rotation_vector.angle() << std::endl;
		}

		std::cout<< "\n";
		for (int i = 0; i < 3; ++i)
		{
			std::cout<< RobotPosition_data.at<double>(i) << " ";
		}
		for (int i = 0; i < 3; ++i)
		{
			std::cout<< RobotPose_data.at<double>(i) << " ";
		}
		RobotPosition.push_back(RobotPosition_data);
		RobotPose.push_back(RobotPose_data);
	}
}


void saveEyeHand(Mat rotationVec, Mat translationVec, int nFrames, vector<int> failedIndex)
{
	FileStorage fs(Config::get<string>("eyehandFileName"),FileStorage::WRITE); //Config::get<double>("varify_alpha");
	float theta = norm(rotationVec);
	Mat axis = rotationVec / theta;
	if (fs.isOpened())
	{
		time_t rawtime;
		time(&rawtime);
		fs << "calibrationDate" << asctime(localtime(&rawtime));
		fs << "failedIndex" << failedIndex;
		fs << "Eye-hand translation" << translationVec;
		fs << "Eye-hand rotation" << rotationVec;
		fs << "Eye-hand rotation theta" << theta;
		fs << "Eye-hand rotation axis" << axis;
	};
	fs.release();
}

void homoTrans(Mat translation, Mat rotation, Mat& homo)
{   int i,j;
	Mat temp(Size(1,3),CV_64FC1);
	if(rotation.cols == 1)
	{
		Rodrigues(rotation,temp);
	}
	else temp = rotation;
	for(i = 0; i  < 3 ;i ++)
	{
		for(j = 0; j  <3 ;j ++)
		{
			homo.at<double>(i,j) = temp.at<double>(i,j);
		}
	}
	homo.at<double>(0,3) = translation.at<double>(0);
	homo.at<double>(1,3) = translation.at<double>(1);
	homo.at<double>(2,3) = translation.at<double>(2);
	homo.at<double>(3,0) = 0;
	homo.at<double>(3,1) = 0;
	homo.at<double>(3,2) = 0;
	homo.at<double>(3,3) = 1;
}

void homo2vector(Mat& translation, Mat& rotation, const Mat homo)
{   int i,j;
	Mat temp = (Mat_<double>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
	for(i = 0; i  < 3 ;i ++)
	{
		for(j = 0; j  < 3 ;j ++)
		{
			temp.at<double>(i,j)  = homo.at<double>(i,j);
		}
	}
	Rodrigues(temp, rotation);
	translation.at<double>(0) = homo.at<double>(0,3);
	translation.at<double>(1) = homo.at<double>(1,3);
	translation.at<double>(2) = homo.at<double>(2,3);
}

////get extrinsic para for camera
/// @param  rotation        original rotation    vec 3*1
/// @param  translation     original translation vec 3*1
/// @param  originIndex     index of origin relative to the true fixed origin
/// @param  chessboardSize  size of chessboard  mm
/// @param  w               width of chessboard
/// @param  h               height of chessboard
Mat ex_matrix(Mat rotation, Mat translation, int originIndex,float chessboardSize,int w,int h)
{
    Mat homo = (Mat_<double>(4,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0,0,0,0); //返回齐次变换矩阵
    Mat t1 = (Mat_<double>(4,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0,0,0,0);
    Mat t2 = (Mat_<double>(4,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0,0,0,0);
    Mat t3 = (Mat_<double>(4,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0,0,0,0);
    Mat rot1 = (Mat_<double>(3,1) << 0, 0, -CV_PI /2);
    Mat rot2 = (Mat_<double>(3,1) << 0, 0, CV_PI /2);
    Mat rot3 = (Mat_<double>(3,1) << 0, 0, CV_PI );
    Mat translation1 = (Mat_<double>(3,1) << (w-1)*chessboardSize, 0, 0);  //
    Mat translation2 = (Mat_<double>(3,1) << 0, (h-1)*chessboardSize, 0);
    Mat translation3 = (Mat_<double>(3,1) << (w-1)*chessboardSize, (h-1)*chessboardSize, 0);

    homoTrans(translation1,rot1,t1);
    homoTrans(translation2,rot2,t2);
    homoTrans(translation3,rot3,t3);
    homoTrans(translation,rotation,homo);

    switch(originIndex)
    {   case 0:   break;
    	case 1:
    	{
    		homo = homo * t1;break;
    	};
    	case 2:
    	{
    		homo = homo * t2;break;
    	};
    	case 3:
    	{
    		homo = homo * t3;break;
    	};
    	default:;
    };
    return homo;
}


void varify(Mat EH_translation, Mat EyeHandRotation,vector<Mat> RobotPosition,vector<Mat> RobotPose,
	vector<Mat> cameraPosition,vector<Mat> cameraPose,double alpha,double beta,
	double Varify_X,double Varify_Y, double Varify_R,Mat& varify_robot_pose)
{
		// 验证照射位姿生成，变量定义
		Mat EHMatrix(Size(4,4),CV_64FC1);        //手眼矩阵 4*4
		Mat ROBOTMatrix(Size(4,4),CV_64FC1);	 //机器人末端相对于base矩阵 4*4
		Mat EXTRMatrix(Size(4,4),CV_64FC1);		 //相机外参数矩阵 4*4
		Mat incline(Size(4,4),CV_64FC1);         //相机坐标系相对标定板变换
		Mat incline_180(Size(4,4),CV_64FC1);     //相机坐标系相对标定板，二次变换，绕z轴转180度，防止撞到相机
		Mat point_pattern(Size(4,4),CV_64FC1);   //标定板坐标系某点 相对 标定板原点坐标
		Mat point_world(Size(1,4),CV_64FC1);     //标定板坐标系某点  转换至机器人世界坐标系 齐次坐标
		Mat vali_pose(Size(4,4),CV_64FC1);		 //为使验证照射至某点，机器人末端位姿

		vector<double> vali_Result_x;
		vector<double> vali_Result_y;
		vector<double> vali_Result_z;
		vector<double> vali_Result_rx;
		vector<double> vali_Result_ry;
		vector<double> vali_Result_rz;

		//====================验证照射点坐标=============================
		point_pattern.at<double>(0,3) = Varify_X;
		point_pattern.at<double>(1,3) = Varify_Y;
		point_pattern.at<double>(2,3) = 0;
		point_pattern.at<double>(3,3) = 1;

		point_pattern.at<double>(0,2) = 0;
		point_pattern.at<double>(1,2) = 0;
		point_pattern.at<double>(2,2) = 1;
		point_pattern.at<double>(3,2) = 0;

		point_pattern.at<double>(0,1) = 0;
		point_pattern.at<double>(1,1) = 1;
		point_pattern.at<double>(2,1) = 0;
		point_pattern.at<double>(3,1) = 0;

		point_pattern.at<double>(0,0) = 1;
		point_pattern.at<double>(1,0) = 0;
		point_pattern.at<double>(2,0) = 0;
		point_pattern.at<double>(3,0) = 0;

		//====================验证照射姿态变量=============================
		alpha *= -1; //<<-------------------------------------------------pay attention
		//double beta = CV_PI / (3.0);
		double r = Varify_R;

		//验证姿态矩阵
		incline.at<double>(0,0) = -sin(beta);
		incline.at<double>(0,1) = cos(beta) * sin(alpha);
		incline.at<double>(0,2) = -cos(alpha) * cos(beta);
		incline.at<double>(0,3) = r * cos(alpha) * cos(beta);

		incline.at<double>(1,0) = cos(beta);
		incline.at<double>(1,1) = sin(alpha) * sin(beta);
		incline.at<double>(1,2) = -cos(alpha) * sin(beta);
		incline.at<double>(1,3) = r * cos(alpha) * sin(beta);

		incline.at<double>(2,0) = 0;
		incline.at<double>(2,1) = -cos(alpha);
		incline.at<double>(2,2) = -sin(alpha);
		incline.at<double>(2,3) = r * sin(alpha);

		incline.at<double>(3,0) = 0;
		incline.at<double>(3,1) = 0;
		incline.at<double>(3,2) = 0;
		incline.at<double>(3,3) = 1;

		//二次变换矩阵
		incline_180.at<double>(0,0) = -1;
		incline_180.at<double>(0,1) = 0;
		incline_180.at<double>(0,2) = 0;
		incline_180.at<double>(0,3) = 0;

		incline_180.at<double>(1,0) = 0;
		incline_180.at<double>(1,1) = -1;
		incline_180.at<double>(1,2) = 0;
		incline_180.at<double>(1,3) = 0;

		incline_180.at<double>(2,0) = 0;
		incline_180.at<double>(2,1) = 0;
		incline_180.at<double>(2,2) = 1;
		incline_180.at<double>(2,3) = 0;

		incline_180.at<double>(3,0) = 0;
		incline_180.at<double>(3,1) = 0;
		incline_180.at<double>(3,2) = 0;
		incline_180.at<double>(3,3) = 1;

		vector<Mat> vali_position;				//为使验证照射至某点，机器人末端位置
		vector<Mat> vali_rotation;				//为使验证照射至某点，机器人末端姿态

		for(int i = 0;i < RobotPosition.size();i++ )
		{
			homoTrans(EH_translation,EyeHandRotation,EHMatrix);
			homoTrans(RobotPosition[i],RobotPose[i],ROBOTMatrix);
			homoTrans(cameraPosition[i],cameraPose[i],EXTRMatrix);

			vali_pose = ROBOTMatrix * EHMatrix * EXTRMatrix * point_pattern * incline * incline_180 * EHMatrix.inv();   //相机光轴对准标定板某点，机器人所处位姿
		//	point_world = ROBOTMatrix * EHMatrix * EXTRMatrix * point_pattern;

			Mat vali_pose_rotation(Size(3,3),CV_64FC1);
			Mat vali_pose_rotation_axis(Size(1,3),CV_64FC1);
			for(int i = 0; i  < 3 ;i ++)
				{	for(int j = 0; j  <3 ;j ++)
					{
						vali_pose_rotation.at<double>(i,j) = vali_pose.at<double>(i,j);
					}
				}
				Rodrigues(vali_pose_rotation,vali_pose_rotation_axis);

				vali_Result_x.push_back(vali_pose.at<double>(0,3));
				vali_Result_y.push_back(vali_pose.at<double>(1,3));
				vali_Result_z.push_back(vali_pose.at<double>(2,3));

				vali_Result_rx.push_back(vali_pose_rotation_axis.at<double>(0));
				vali_Result_ry.push_back(vali_pose_rotation_axis.at<double>(1));
				vali_Result_rz.push_back(vali_pose_rotation_axis.at<double>(2));
			//cout << "valid_pose" << endl;
			//cout << vali_pose.at<double>(0,3) << "  " << vali_pose.at<double>(1,3) << "  " << vali_pose.at<double>(2,3) << endl;
			}

			//利用RANSAC剔除外点
			double vX(0), vY(0), vZ(0),vRx(0),vRy(0),vRz(0);

			Mat vali_x = ransac(vali_Result_x.size(),vali_Result_x,100,0.5,vX);
			Mat vali_y = ransac(vali_Result_y.size(),vali_Result_y,100,0.5,vY);
			Mat vali_z= ransac(vali_Result_z.size(),vali_Result_z,100,0.5,vZ);
			Mat vali_Rx = ransac(vali_Result_rx.size(),vali_Result_rx,100,0.01,vRx);
			Mat vali_Ry = ransac(vali_Result_ry.size(),vali_Result_ry,100,0.01,vRy);
			Mat vali_Rz = ransac(vali_Result_rz.size(),vali_Result_rz,100,0.01,vRz);

			varify_robot_pose = Mat::zeros(1,6,CV_64FC1);
			varify_robot_pose.at<double>(0,0) = vX/1000;
			varify_robot_pose.at<double>(0,1) = vY/1000;
			varify_robot_pose.at<double>(0,2) = vZ/1000;
			varify_robot_pose.at<double>(0,3) = vRx;
			varify_robot_pose.at<double>(0,4) = vRy;
			varify_robot_pose.at<double>(0,5) = vRz;

		// cout<<"varify position:"<<endl;
		// cout<<vX<<endl;
		// cout<<vY<<endl;
		// cout<<vZ<<endl;
		// cout<<"varify orientation:"<<endl;
		// cout<<vRx<<endl;
		// cout<<vRy<<endl;
		// cout<<vRz<<endl;
		}

		void eyeHandCalibraion(string cameraFileName,string robotFileName,vector<int> failedIndex)
		{
			FileStorage fs(cameraFileName,FileStorage::READ);
			int NumOfImg = 0;
			Mat extrinsic_parameters;
			fs["nframes"] >> NumOfImg;
			fs["extrinsic_parameters"] >> extrinsic_parameters;
			fs.release();

			vector<Mat> RobotPose;
			vector<Mat> cameraPose;
			vector<Mat> RobotPosition;
			vector<Mat> cameraPosition;

			vector<Mat> EyeHandR;
	vector<Mat> EyeHandAxisAngle;//axis angle表示的eye hand姿态
	vector<Mat> EyeHandP_L12;//求解位置 方程左边（求逆）
	vector<Mat> EyeHandP_L23;//求解位置 方程左边（求逆）
	vector<Mat> EyeHandP_R12;//求解位置 方程右边
	vector<Mat> EyeHandP_R23;//求解位置 方程右边

	//// ----------------------read camera parameters----------------------------
	for(int i = 0;i < NumOfImg ; i++)
	{
		Mat temp_pose = Mat::zeros(Size(1,3),CV_64FC1);
		Mat temp_position = Mat::zeros(Size(1,3),CV_64FC1);

		temp_pose.at<double>(0,0) = extrinsic_parameters.at<double>(i,0);
		temp_pose.at<double>(0,1) = extrinsic_parameters.at<double>(i,1);
		temp_pose.at<double>(0,2) = extrinsic_parameters.at<double>(i,2);

		temp_position.at<double>(0,0) = extrinsic_parameters.at<double>(i,3);
		temp_position.at<double>(0,1) = extrinsic_parameters.at<double>(i,4);
		temp_position.at<double>(0,2) = extrinsic_parameters.at<double>(i,5);

		cameraPosition.push_back(temp_position );
		cameraPose.push_back(temp_pose);
	}
	//// ----------------------read robot parameters----------------------------
	readRobotDataInFileStream(robotFileName,RobotPose,RobotPosition,NumOfImg,failedIndex);
	// readRobotData(robotFileName,RobotPose,RobotPosition,NumOfImg,failedIndex);
	//// ----------------------求解手眼矩阵变量定义----------------------------
	Mat RobotPose_1(Size(1,3),CV_64FC1);			 // i-1次机器人姿态，axis/angle
	Mat RobotPose_2(Size(1,3),CV_64FC1);			// i次机器人姿态，axis/angle
	Mat RobotPose_3(Size(1,3),CV_64FC1);			// i+1次机器人姿态，axis/angle
	Mat RobotPosition_1(Size(1,3),CV_64FC1);        // i-1次机器人位置，
	Mat RobotPosition_2(Size(1,3),CV_64FC1);        // i次机器人位置，
	Mat RobotPosition_3(Size(1,3),CV_64FC1);        // i+1次机器人位置，

	Mat RobotPose_Matrix1(Size(3,3),CV_64FC1);		// i-1次机器人姿态，33矩阵
	Mat RobotPose_Matrix2(Size(3,3),CV_64FC1);		// i次机器人姿态，33矩阵
	Mat RobotPose_Matrix3(Size(3,3),CV_64FC1);		// i+1次机器人姿态，33矩阵

	Mat cameraPose_1(Size(1,3),CV_64FC1);			// i-1次camera姿态，axis/angle
	Mat cameraPose_2(Size(1,3),CV_64FC1);			// i次camera姿态，axis/angle
	Mat cameraPose_3(Size(1,3),CV_64FC1);			// i+1次camera姿态，axis/angle
	Mat cameraPosition_1(Size(1,3),CV_64FC1);        // i-1次camera位置，
	Mat cameraPosition_2(Size(1,3),CV_64FC1);        // i次camera位置，
	Mat cameraPosition_3(Size(1,3),CV_64FC1);        // i+1次camera位置，

	Mat cameraPose_Matrix1(Size(3,3),CV_64FC1);		// i-1次camera姿态，33矩阵
	Mat cameraPose_Matrix2(Size(3,3),CV_64FC1);		// i次camera姿态，33矩阵
	Mat cameraPose_Matrix3(Size(3,3),CV_64FC1);		// i+1次camera姿态，33矩阵

	int a,b,c;
	Mat L,R,L_temp,R_temp;//用于求解最小2乘的参数

	vector<double> position_Result_x;
	vector<double> position_Result_y;
	vector<double> position_Result_z;
	vector<double> orientation_Result_Rx;
	vector<double> orientation_Result_Ry;
	vector<double> orientation_Result_Rz;

	//// -------------------------------求解手眼矩阵：计算所有组合------------------------------------------，
	for(a = 0; a <  RobotPosition.size(); a++)
	{
		for(b = a +1; b < RobotPosition.size(); b++)
		{
			for(c = b+1; c < RobotPosition.size(); c++)
			{
				cameraPose_1 = cameraPose[a];
				cameraPose_2 = cameraPose[b];
				cameraPose_3 = cameraPose[c];
				cameraPosition_1 = cameraPosition[a];
				cameraPosition_2 = cameraPosition[b];
				cameraPosition_3 = cameraPosition[c];

				RobotPose_1 = RobotPose[a];
				RobotPose_2 = RobotPose[b];
				RobotPose_3 = RobotPose[c];
				RobotPosition_1 = RobotPosition[a];
				RobotPosition_2 = RobotPosition[b];
				RobotPosition_3 = RobotPosition[c];

				Rodrigues(RobotPose_1,RobotPose_Matrix1);
				Rodrigues(RobotPose_2,RobotPose_Matrix2);
				Rodrigues(RobotPose_3,RobotPose_Matrix3);

				Rodrigues(cameraPose_1,cameraPose_Matrix1);
				Rodrigues(cameraPose_2,cameraPose_Matrix2);
				Rodrigues(cameraPose_3,cameraPose_Matrix3);


				Mat R_Left1(Size(3,3),CV_64FC1);
				Mat R_Right1(Size(3,3),CV_64FC1);
				Mat R_Left2(Size(3,3),CV_64FC1);
				Mat R_Right2(Size(3,3),CV_64FC1);

				Mat R_Left_angle1(Size(1,3),CV_64FC1);
				Mat R_Right_angle1(Size(1,3),CV_64FC1);
				Mat R_Left_angle2(Size(1,3),CV_64FC1);
				Mat R_Right_angle2(Size(1,3),CV_64FC1);

				Mat pL1(Size(1,3),CV_64FC1);
				Mat pL2(Size(1,3),CV_64FC1);
				Mat pL3(Size(1,3),CV_64FC1);

				Mat pR1(Size(1,3),CV_64FC1);
				Mat pR2(Size(1,3),CV_64FC1);
				Mat pR3(Size(1,3),CV_64FC1);

				R_Left1 = RobotPose_Matrix1.t() * RobotPose_Matrix2;
				R_Left2 = RobotPose_Matrix2.t() * RobotPose_Matrix3;
				R_Right1 = cameraPose_Matrix1 * cameraPose_Matrix2.t();
				R_Right2 = cameraPose_Matrix2 * cameraPose_Matrix3.t();

				Rodrigues(R_Left1,R_Left_angle1);
				Rodrigues(R_Left2,R_Left_angle2);
				Rodrigues(R_Right1,R_Right_angle1);
				Rodrigues(R_Right2,R_Right_angle2);

				Mat LeftMatrix(Size(3,3),CV_64FC1);
				Mat RightMatrix(Size(3,3),CV_64FC1);
				Mat LeftCross(Size(1,3),CV_64FC1);
				Mat RightCross(Size(1,3),CV_64FC1);

				int i,j;
				for(i = 0;i < 3; i++)
				{
					for(j = 0;j < 3; j++)
						if (j == 0)
						{
							LeftMatrix.at<double>(i,j) = R_Left_angle1.at<double>(i) / vec_norm(R_Left_angle1);
							RightMatrix.at<double>(i,j) = R_Right_angle1.at<double>(i) / vec_norm(R_Right_angle1);
						}
						else if (j ==1)
						{
							LeftMatrix.at<double>(i,j) = R_Left_angle2.at<double>(i) / vec_norm(R_Left_angle2);
							RightMatrix.at<double>(i,j) = R_Right_angle2.at<double>(i) / vec_norm(R_Right_angle2);
						}
						else
							{   LeftCross = R_Left_angle1.cross(R_Left_angle2);
								RightCross = R_Right_angle1.cross(R_Right_angle2);
								LeftMatrix.at<double>(i,j) = LeftCross.at<double>(i) / vec_norm(LeftCross);
								RightMatrix.at<double>(i,j) = RightCross.at<double>(i) / vec_norm(RightCross);
							}

						}

						Mat EyeHandMatrix(Size(3,3),CV_64FC1);
						Mat EyeHand_AxisAngle(Size(1,3),CV_64FC1);
						Mat EyeHand_position(Size(1,3),CV_64FC1);

						Mat E = Mat::eye(3,3,CV_64FC1);
						Mat temp;

						//求解手眼矩阵旋转向量
						EyeHandMatrix = LeftMatrix * RightMatrix.inv();

						pL1 = RobotPose_Matrix1.t() * RobotPosition_2 - RobotPose_Matrix1.t() * RobotPosition_1;
						pR1 = (-1) * cameraPose_Matrix1 * cameraPose_Matrix2.t() * cameraPosition_2 + cameraPosition_1;

						pL2 = RobotPose_Matrix2.t() * RobotPosition_3 - RobotPose_Matrix2.t() * RobotPosition_2;
						pR2 = (-1) * cameraPose_Matrix2 * cameraPose_Matrix3.t() * cameraPosition_3 + cameraPosition_2;

						temp = R_Left2 - E;

						EyeHandR.push_back(EyeHandMatrix);
						Rodrigues(EyeHandMatrix,EyeHand_AxisAngle);

						//输出各种组合的标定结果，用于比较
			/*				printf("%d %d %d\n",a,b,c);
						printf("%f\n",EyeHand_AxisAngle.at<double>(0) * 180.0 / CV_PI);
						printf("%f\n",EyeHand_AxisAngle.at<double>(1) * 180.0 / CV_PI);
						printf("%f\n",EyeHand_AxisAngle.at<double>(2)* 180.0 / CV_PI);
						printf("\n");*/
						//write(EyeHand_AxisAngle,true,"Orientation.txt");
						//printf("%d %d %d\n",a,b,c);

						//手眼矩阵旋转向量放入vector容器，供求平均值
						EyeHandAxisAngle.push_back(EyeHand_AxisAngle);

						//手眼矩阵旋转向量,各个分量放入vector容器，供RANSAC
						orientation_Result_Rx.push_back(EyeHand_AxisAngle.at<double>(0));
						orientation_Result_Ry.push_back(EyeHand_AxisAngle.at<double>(1));
						orientation_Result_Rz.push_back(EyeHand_AxisAngle.at<double>(2));

						Mat L12(Size(3,3),CV_64FC1);
						Mat L23(Size(3,3),CV_64FC1);
						Mat R12(Size(3,3),CV_64FC1);
						Mat R23(Size(3,3),CV_64FC1);

						R12 = EyeHandMatrix * pR1 - pL1;
						R23 = EyeHandMatrix * pR2 - pL2;
						L12 = R_Left1 - E;
						L23 = R_Left2 - E;

					   //将求解平移向量的矩阵方程，左右矩阵依次拼接
						if(a == 0 && b ==1 && c ==2)  //初始化矩阵，然后拼接
						{
							L = L12; R = R23;
						}
						L_temp = MatrixCombine(L12,L23);
						L = MatrixCombine(L,L_temp);
						R_temp = MatrixCombine(R12,R23);
						R = MatrixCombine(R,R_temp);

					//	L_temp = MatrixCombine(L12,L23);
					//	R_temp = MatrixCombine(R12,R23);

						Mat Result_(Size(1,3),CV_64FC1);
						solve(L_temp,R_temp,Result_,DECOMP_QR);
						position_Result_x.push_back(Result_.at<double>(0));
						position_Result_y.push_back(Result_.at<double>(1));
						position_Result_z.push_back(Result_.at<double>(2));
					}
				}
			}

	//利用RANSAC剔除外点
			double positionResultX, positionResultY, positionResultZ,orientationResultRx,orientationResultRy,orientationResultRz;
			Mat temp_X = ransac(position_Result_x.size(),position_Result_x,1000,0.5,positionResultX);
			Mat temp_Y = ransac(position_Result_y.size(),position_Result_y,1000,0.5,positionResultY);
			Mat temp_Z = ransac(position_Result_z.size(),position_Result_z,1000,0.5,positionResultZ);
			Mat temp_Rx = ransac(orientation_Result_Rx.size(),orientation_Result_Rx,1000,0.01,orientationResultRx);
			Mat temp_Ry = ransac(orientation_Result_Ry.size(),orientation_Result_Ry,1000,0.01,orientationResultRy);
			Mat temp_Rz = ransac(orientation_Result_Rz.size(),orientation_Result_Rz,1000,0.01,orientationResultRz);

			Mat EH_translation(Size(1,3),CV_64FC1);
			Mat EH_rotation(Size(1,3),CV_64FC1);

			EH_translation.at<double>(0) =  positionResultX;
			EH_translation.at<double>(1) =  positionResultY;
			EH_translation.at<double>(2) =  positionResultZ;

			EH_rotation.at<double>(0) =  orientationResultRx;
			EH_rotation.at<double>(1) =  orientationResultRy;
			EH_rotation.at<double>(2) =  orientationResultRz;

			cout << "eye hand translation:\n" << EH_translation << endl;
			cout << "eye hand rotation:\n" << EH_rotation << endl;

			saveEyeHand(EH_rotation,EH_translation,NumOfImg,failedIndex);

			cout << "Start to varify the result...\n";
			vector<Mat> varify_poses;
			int vari_num = Config::get<int>("varify_num");
			FileStorage fw(Config::get<string>("variFileName"),FileStorage::WRITE);
			fw << "varify_num" << vari_num;
			for(int i = 0; i < vari_num; i++){
				stringstream ss;ss << i+1;
				double alpha = Config::get<double>("varify_alpha_" + ss.str());
				double beta = Config::get<double>("varify_beta_" + ss.str());
				double x = Config::get<double>("varify_x_" + ss.str());
				double y = Config::get<double>("varify_y_" + ss.str());
				double r = Config::get<double>("varify_R_" + ss.str());

				Mat robot_vari_pose;
				varify(EH_translation, EH_rotation,RobotPosition,RobotPose,
					cameraPosition,cameraPose,alpha,beta,x,y,r,robot_vari_pose);
				cout << "varify pose" + ss.str() << "  " << robot_vari_pose << "\n";
				if (fw.isOpened())
				{
					fw << "varify_pose_" + ss.str() << robot_vari_pose;
				};
			}
			fw.release();
		}

