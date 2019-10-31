#include "calib.h"
#include <vector>

#include "ros/ros.h"
#include "core.h"
//opencv
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "calibinit.hpp"
// #include "videoinput.hpp"

using namespace cv;
#define PI 3.1415926

int current_pose_id = 0;
Mat intrinsic;
Mat distortion;


bool init_calib(string filename)
{
    /* 读取标定参数 */
    static string intrinsics_filename = filename;
    FileStorage fs(intrinsics_filename, FileStorage::READ);
	if(fs.isOpened())
	{
        fs["camera_matrix"]>>intrinsic;
        fs["distortion_coefficients"]>>distortion;
		fs.release();
	}
    else
    {
        return false;
    }

    return true;
}

bool find_chessboard(_YAT_POINTF_THETA &pos_theta)
{
    VideoCapture capture;
    if(!capture.open(0))
    {
        return false;
    }
    Mat framesrc;
    Mat frame;
    Size boardSize(3, 3);
    std::vector<cv::Point2f> imageCorners;

    //frame = vthread.getframe();
    for(int i=0; i<10; i++)
        capture >> framesrc;

    capture.release();

    if(framesrc.empty())
    {
		return false;        
    }

    cvtColor(framesrc, frame, CV_BGR2GRAY);

    double t1 = getTickCount();
    bool found = ytfindChessboardCorners(frame, boardSize, imageCorners, CV_CALIB_CB_FAST_CHECK|CV_CALIB_CB_NORMALIZE_IMAGE|CV_CALIB_CB_ADAPTIVE_THRESH);// 计算角点信息
    double t2 = (getTickCount() - t1) / getTickFrequency() * 1000;

    if(!found)
    {
        return false;
    }

    if(abs(imageCorners[0].x - imageCorners[8].x) < 60 || abs(imageCorners[0].y - imageCorners[8].y) < 50)
    {
        ROS_INFO("Found error chessboard.");
        return false;
    }    

    CvMat* image_points = cvCreateMat(imageCorners.size(), 2, CV_32FC1);
    CvMat* object_points = cvCreateMat(imageCorners.size(), 3, CV_32FC1);

    vector<cv::Point3f> ConP3DVec;
    vector<cv::Point2f> ConP2DVec;

    std::vector<cv::Point2f> reorderimageCorners1;
    std::vector<cv::Point2f> reorderimageCorners2;

    if(imageCorners.size() == boardSize.area())
    {
        if((imageCorners[0].x + imageCorners[0].y)  > (imageCorners[(boardSize.height -1) * boardSize.width].x + imageCorners[(boardSize.height -1) * boardSize.width].y))
        {
            for(int j=0; j<boardSize.width; j++)
            {
                for(int i=0; i<boardSize.height; i++)
                {
                    reorderimageCorners1.push_back(imageCorners[j + boardSize.width *(boardSize.height-i -1)]);
                }
            }

            imageCorners = reorderimageCorners1;
        }

        if((imageCorners[0].x + imageCorners[0].y)  > (imageCorners[(boardSize.height -1) * boardSize.width].x + imageCorners[(boardSize.height -1) * boardSize.width].y))
        {
            for(int j=0; j<boardSize.width; j++)
            {
                for(int i=0; i<boardSize.height; i++)
                {
                    reorderimageCorners2.push_back(imageCorners[j + boardSize.width *(boardSize.height-i -1)]);
                }
            }

            imageCorners = reorderimageCorners2;
        }

        // if((imageCorners[0].x + imageCorners[0].y)  > (imageCorners[boardSize.width - 1].x + imageCorners[boardSize.width - 1].y))
        // {
        //     for(int j=0; j<boardSize.width; j++)
        //     {
        //         for(int i=0; i<boardSize.height; i++)
        //         {
        //             reorderimageCorners2.push_back(imageCorners[i*boardSize.width + boardSize.width-j -1]);
        //         }
        //     }

        //     imageCorners = reorderimageCorners2;
        // }


        int color = 0;
        for(int i = 0; i < boardSize.height; i++)
        {
            for(int j = 0; j < boardSize.width; j++)
            {
                if(i != 1 && j != 1)
                {
                    cv::Point3f ConP3D;
                    ConP3D.x = 1 - j;
                    ConP3D.y = i - 1;     //与地面垂直;
                    ConP3D.z = 0.0;       //i - 1与地面平行
                    ConP3DVec.push_back(ConP3D);

                    Point2f ConP2D;
                    ConP2D.x = imageCorners[i*boardSize.width+j].x;
                    ConP2D.y = imageCorners[i*boardSize.width+j].y;
                    ConP2DVec.push_back(ConP2D);
                }

                // CV_MAT_ELEM(*image_points, float, i*boardSize.width+j, 0) = imageCorners[i*boardSize.width+j].x;
                // CV_MAT_ELEM(*image_points, float, i*boardSize.width+j, 1) = imageCorners[i*boardSize.width+j].y;
                // CV_MAT_ELEM(*object_points, float, i*boardSize.width+j, 0) = i -1;
                // CV_MAT_ELEM(*object_points, float, i*boardSize.width+j, 1) = j -1;
                // CV_MAT_ELEM(*object_points, float, i*boardSize.width+j, 2) = 0.0f;

                color += 40;
                circle(frame, imageCorners[boardSize.width * i + j], 3, Scalar(color, 0, 0), 5);
                char text[100];
                sprintf(text, "%d, %d", 1 - j, i - 1);
                putText(frame, text,imageCorners[boardSize.width * i + j], 1, 1, Scalar(255, 255, 255),1);
            }
        }   
    }

    cv::Mat Rod_r ,TransMatrix;
    bool success = solvePnP(ConP3DVec, ConP2DVec, intrinsic, distortion, Rod_r, TransMatrix,false, cv::SOLVEPNP_ITERATIVE);

    pos_theta.x = TransMatrix.at<double>(0, 0) * 72.0 / 1000;   //x
    pos_theta.y = TransMatrix.at<double>(2, 0) * 72.0 / 1000;   //y
    pos_theta.theta = Rod_r.at<double>(1, 0);                   //rad

    imwrite("/home/raspberry002/catkin_ws/src/zigzag_mower/calibed_frame.jpg", frame);
    ROS_INFO("write one frame");
    return true;
}
