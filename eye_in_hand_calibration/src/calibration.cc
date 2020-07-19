////-----------------------------------------------------------
////  This .cc is used for single camera calibration. It is adpated
////  from the opencv calibration example. After camera calibration
////  is finished, eye-hand calibration and result varification can be 
////  performed.
////
////  Author: LI KAI 
////  Date: 2017.10.27
////-----------------------------------------------------------
#include <opencv2/opencv.hpp>

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

#include "../include/eye_hand_calibration/eyeHand.h"
#include "../include/eye_hand_calibration/config.h"
using namespace cv;
using namespace std;

const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w 4 -h 5 -s 0.025 -o camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w 4 -h 5 -s 0.025 -o camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";

static void help()
{
    printf( "This is a camera calibration sample.\n"
        "Usage: calibration\n"
        "     -w <board_width>         # the number of inner corners per one of board dimension\n"
        "     -h <board_height>        # the number of inner corners per another board dimension\n"
        "     [-pt <pattern>]          # the type of pattern: chessboard or circles' grid\n"
        "     [-n <number_of_frames>]  # the number of frames to use for calibration\n"
        "                              # (if not specified, it will be set to the number\n"
        "                              #  of board views actually available)\n"
        "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
        "                              # (used only for video capturing)\n"
        "     [-s <squareSize>]       # square size in some user-defined units (1 by default)\n"
        "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
        "     [-op]                    # write detected feature points\n"
        "     [-oe]                    # write extrinsic parameters\n"
        "     [-zt]                    # assume zero tangential distortion\n"
        "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
        "     [-p]                     # fix the principal point at the center\n"
        "     [-v]                     # flip the captured images around the horizontal axis\n"
        "     [-V]                     # use a video file, and not an image list, uses\n"
        "                              # [input_data] string for the video file name\n"
        "     [-su]                    # show undistorted images after calibration\n"
        "     [input_data]             # input data, one of the following:\n"
        "                              #  - text file with a list of the images of the board\n"
        "                              #    the text file can be generated with imagelist_creator\n"
        "                              #  - name of video file with a video of the board\n"
        "                              # if input_data not specified, a live view from the camera is used\n"
        "\n" );
    printf("\n%s",usage);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch(patternType)
    {
      case CHESSBOARD:
      case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

      case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

      default:
        CV_Error(CV_StsBadArg, "Unknown pattern type\n");
    }
}

static bool runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize, Pattern patternType,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}


static void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

void modifyExtrinsics(vector<cv::Mat>& rvec,vector<cv::Mat>& tvec,vector<int> index,float chessboardSize,int w,int h)
{
    CV_Assert(rvec.size() == tvec.size() && rvec.size() == index.size());
    for (int ImageNum = 0;ImageNum < rvec.size() ;ImageNum++)
    {
        Mat ex_rot = (Mat_<double>(3,1) << 0, 0, 0);  //转换后的外参矩阵，旋转部分
        Mat ex_tr = (Mat_<double>(3,1) << 0, 0, 0);   //转换后的外参矩阵，平移部分
        homo2vector(ex_tr, ex_rot ,ex_matrix(rvec[ImageNum],tvec[ImageNum],index[ImageNum],chessboardSize,w,h));
        rvec[ImageNum] = ex_rot.clone();
        tvec[ImageNum] = ex_tr.clone();
    }
}

static bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints,vector<int> cornerIndex)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
    cout << "corner index size:  " << cornerIndex.size() << endl;
    cout << "board size:  " << boardSize.width << "      " << boardSize.height << endl;
    cout << "squareSize:  " << squareSize << endl;

    modifyExtrinsics(rvecs,tvecs,cornerIndex,squareSize,boardSize.width,boardSize.height);

    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if( ok )
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
    return ok;
};
struct Origin
{
	cv::Mat img;
	cv::Point2f p;
	Origin(cv::Mat img):img(img){};
};
void on_mouse(int event,int x,int y,int flags,void* ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号,ustc: user parameter 
{  
	Mat tmp;
	static Point pre_pt(-1,-1);//初始坐标  
	static Point cur_pt(-1,-1);//实时坐标  
	Mat img0 = ((Origin*)ustc)->img;
	Point2f Mouse_origin;
	string temp;  
	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
	{  
		//org.copyTo(img);//将原始图片复制到img中  
		stringstream x_s, y_s;
		x_s << x;
		y_s << y;
		pre_pt = Point(x,y);  
		putText(img0,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);//在窗口上显示坐标  
		circle(img0,pre_pt,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);//划圆  
		Mouse_origin = pre_pt;
		((Origin*)ustc)->p = pre_pt;
		imshow("Image View",img0);  	
	}  
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数  
	{  
		img0.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标  
		stringstream x_s, y_s;
		x_s << x;
		y_s << y;
		temp = "(" + x_s.str() + "," + y_s.str() + ")";
		cur_pt = Point(x,y);  
		putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));//只是实时显示鼠标移动的坐标  
		imshow("Image View",tmp);  
	}  
	else if (event == CV_EVENT_RBUTTONDOWN)
		{ 
			cvDestroyWindow("Image View");		
			return;
		}
}
int getClosestIndex(const vector<int>& cornerIndex,const vector<cv::Point2f>& pointBuf,const cv::Point2f& Origin)
{
	CV_Assert(cornerIndex.size() == 4);
	int res = 0;
	double closestDistance = cv::norm(pointBuf[cornerIndex[0]] - Origin);
	for(int i = 0; i < 4; i++){
		if(cv::norm(pointBuf[cornerIndex[i]] - Origin) < closestDistance){
			closestDistance = cv::norm(pointBuf[cornerIndex[i]] - Origin);
			res = i;
	  }
   }
   return res;
}

int main( int argc, char** argv )
{
    string paraFileName = argv[1];
    Config::setParameterFile(paraFileName);

    Size boardSize, imageSize;
    float squareSize = Config::get<float>("squareSize");//<<-----------------------------parameter
    float aspectRatio = 1.f;   
    Mat cameraMatrix, distCoeffs;
    string outputFilename_str = Config::get<string>("outputFileName");   //imageFileName
    const char* outputFilename = outputFilename_str.c_str();//"out_camera_data.yml";
    string inputFilename_str = Config::get<string>("imageFileName");   //imageFileName
    cout << inputFilename_str << endl;
    const char* inputFilename = inputFilename_str.c_str();

    int i, nframes = 10;
    bool writeExtrinsics = true, writePoints = false;
    bool undistortImage = false;
    int flags = 0;
    VideoCapture capture;
    bool flipVertical = false;
    bool showUndistorted = false;
    bool videofile = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    int cameraId = 0;
    vector<vector<Point2f> > imagePoints;
    vector<string> imageList;
    Pattern pattern = CHESSBOARD;

    vector<int> failedIndex;
    int currentIndex = 0;

    if( argc < 2 )
    {
        help();
        return 0;
    }

	//--------------------------------------read parameters-----------------------------------------------------------------
    boardSize.width = Config::get<int>("boardWidth");
    boardSize.height = Config::get<int>("boardHeight");

    if( inputFilename )
    {
        if( !videofile && readStringList(inputFilename, imageList) )
            mode = CAPTURING;
        else{
            cout << "fail to read image list!" << endl;
            exit(1);
        }
    }
    else{
        cout << "no input file name!" << endl;
        exit(1);
    }

    if( !imageList.empty() )
        nframes = (int)imageList.size();

    namedWindow( "Image View", 1 );

    vector<int> closestIndexVec;
    for(i = 0;;i++)
    {
        Mat view, viewGray;
        bool blink = false;

        if( i < (int)imageList.size() )
            view = imread(imageList[i], 1);
        if(!view.data)   //<<--------------------------------------if all image corners have been detected, run calibration and print failed image index
        {
            if( imagePoints.size() > 0 ){
                runAndSave(outputFilename, imagePoints, imageSize,
                           boardSize, pattern, squareSize, aspectRatio,
                           flags, cameraMatrix, distCoeffs,
                           writeExtrinsics, writePoints,closestIndexVec);
            }
            cout << "failed image index:" << endl;
            for(int fi:failedIndex){
            	cout << fi << endl;
            }
            cout << "closest corner index:" << endl;
            for(int k:closestIndexVec)
                cout << k << " ";
            cout << '\n';
            break;
        }
        else{
        	currentIndex++;
        }
        imageSize = view.size();

        if( flipVertical )
            flip( view, view, 0 );

        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);

        bool found;
        found = findChessboardCorners( view, boardSize, pointbuf,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

       // improve the found corners' coordinate accuracy
        if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(11,11),
            Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

        if( mode == CAPTURING && found &&
           (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
        }

        if(found)
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
        else{
        	failedIndex.push_back(currentIndex-1);   			//<<<---------------------------------------------------record failed 
        }
        string msg = mode == CAPTURING ? "100/100" :
            mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(undistortImage)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), nframes );
        }

        putText( view, msg, textOrigin, 1, 1,
                 mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( mode == CALIBRATED && undistortImage )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        Origin o(view);										//<<<---------------------------------------------------get mouse position
        imshow("Image View", view);
        setMouseCallback("Image View",on_mouse,&o);
        cv::waitKey();

        //---------------get (u,v) of four outer corners------------------------------
        vector<int> cornerIndex;
        cornerIndex.push_back(0);cornerIndex.push_back(0 + boardSize.width - 1);
        cornerIndex.push_back(0 + boardSize.width * (boardSize.height-1));
        cornerIndex.push_back(boardSize.width * boardSize.height - 1);
        
        if(found){
        	int closestIndex = getClosestIndex(cornerIndex,pointbuf,o.p);
        	// cout << "num of corners  " << pointbuf.size() << endl;
        	// cout << "original  " << pointbuf[0] << endl;
        	// cout << "modified  " << o.p << endl;
        	cout << "closest index  " << closestIndex << endl;
            closestIndexVec.push_back(closestIndex);
        }
        else{
        	cout << "failed to detect corners" << endl;
        }

        int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

        if( (key & 255) == 27 )
            break;

        if( key == 'u' && mode == CALIBRATED )
            undistortImage = !undistortImage;
    }
    //------------------------start eye-hand calibration--------------------------------------------------------------
    eyeHandCalibraion(outputFilename,Config::get<string>("robotFileName"),failedIndex); //(string)argv[4]
    return 0;
}
