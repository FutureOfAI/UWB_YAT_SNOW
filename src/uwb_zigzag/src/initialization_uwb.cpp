//ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "std_msgs/String.h"

//spdlog
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

//map
//#include <contour.h>
#include <path.h>

// #include <mapping.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

//Synchronize subscription messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

//opencv
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#define PI 3.14159265358979323846
#define degreeToRad 3.14159265358979323846/180

using namespace std;

typedef  struct
{
	double x; 
	double y;
    double z;
} PonitXYZ; 

//get the current date //date -s "20180929 08:00:00"
std::string getDate();

//point file
std::string point_path = "/home/raspberry002/catkin_ws/src/uwb_zigzag/data/";
std::string file_status = point_path + getDate() + "initialization.txt";

/******************* ros publisher ****************************/
ros::Publisher vel_pub;
ros::Publisher initial_pub;

bool initialFinish = false;
bool refreshPIDFlag = true;

bool init_uwb = false;
bool gotPositionUWB = false;
bool motionFinish = false;
Eigen::Vector3d PositionUWB(0,0,0);
Eigen::Vector3d PositionDR(0,0,PI/2);
geometry_msgs::PoseStamped initialPose;

geometry_msgs::Point startPoint;
geometry_msgs::Point endPoint;

vector<PonitXYZ> pathPoint(10);

bool pauseFlag = false;
bool refeshIMU = false;

geometry_msgs::PoseStamped odomData;
geometry_msgs::PoseStamped imuData;
geometry_msgs::PointStamped eulerAngle;

geometry_msgs::Twist targetStates;

geometry_msgs::Point robotPose;
geometry_msgs::Point robotVel;
/*******************************************************************/

double constrainAngle(double angle)
{
    double trueAngle = angle;
    while (trueAngle > PI)
    {
        trueAngle -= 2*PI;
    }

    while (trueAngle < -PI)
    {
        trueAngle += 2*PI;
    }

    return trueAngle;
}

// deal with the margin -180 <=> 180
double deltaAngle(double targetAngle, double currentAngle)
{
    return constrainAngle(targetAngle - currentAngle);
}

void locationDR()
{

    static double yaw_prev = eulerAngle.point.z; 
    static double mileage_prev = odomData.pose.position.x; 


    double yaw_delta;   // delta yaw measured by imu
    double mileage_delta;  // current mileage measured by odom
    
    mileage_delta = odomData.pose.position.x - mileage_prev;
    yaw_delta = deltaAngle(eulerAngle.point.z*degreeToRad,yaw_prev*degreeToRad);
    // yaw_delta = yaw_delta*PI/180;

    /***********************************************  evaluate PositionDR by IMU and mileage *************************************************/ 
    Eigen::Vector3d PositionDR_delta(mileage_delta*cos(PositionDR(2)+yaw_delta/2),mileage_delta*sin(PositionDR(2)+yaw_delta/2),yaw_delta);
    PositionDR = PositionDR + PositionDR_delta;
    /***********************************************************************************************************************************/ 
    PositionDR(2) = constrainAngle(PositionDR(2));

    // ROS_INFO(" robot position is: x = %f , y = %f, theta = %f .",PositionDR(0),PositionDR(1),PositionDR(2)*180/PI);    

    //update the prev data
    yaw_prev = eulerAngle.point.z; 
    mileage_prev = odomData.pose.position.x;
}

void robotPoseCallback()
{
    robotPose.x = PositionDR(0);
    robotPose.y = PositionDR(1);
    robotPose.z = PositionDR(2);//(-180,180]
}

double controlPID(double delta,double delta_integral,double delta_last,double Kp,double Ki,double Kd)
{
    double DeltaP,DeltaI,DeltaD,DeltaControler;
    
    DeltaP=Kp*delta;
    DeltaI=Ki*delta_integral;
    DeltaD=Kd*(delta-delta_last);
    
    DeltaControler=DeltaP+DeltaI+DeltaD;

    return DeltaControler;
}

double deltaDistance(geometry_msgs::Point pointA, geometry_msgs::Point pointB)
{
    double delta_X = pointA.x - pointB.x;
    double delta_Y = pointA.y - pointB.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

double distanceToAB(geometry_msgs::Point pointA, geometry_msgs::Point pointB,geometry_msgs::Point currentPoint)
{
    // angle control
    double pathDistance = deltaDistance(pointA , pointB); 

    YAT_POINTF pathVector,normalVector,targetVector;
    pathVector.x = (pointA.x - pointB.x)/pathDistance;
    pathVector.y = (pointA.y - pointB.y)/pathDistance;

    // clockwise vector
    normalVector.x = pathVector.y;
    normalVector.y = - pathVector.x;

    targetVector.x = pointA.x - currentPoint.x;
    targetVector.y = pointA.y - currentPoint.y;

    return (targetVector.x*normalVector.x + targetVector.y*normalVector.y);
}

int sign(float data)
{
    if(data>0)
    {
        return 1;
    }
    else if (data<0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

bool straightMotion(geometry_msgs::Point nextPoint,geometry_msgs::Point lastPoint)
{
    // angle control
    double pathDistance = deltaDistance(nextPoint , lastPoint); 

    Eigen::Vector2d pathVector( (nextPoint.x - lastPoint.x)/pathDistance, (nextPoint.y - lastPoint.y)/pathDistance );
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    Eigen::Vector2d targetVector(nextPoint.x - robotPose.x, nextPoint.y - robotPose.y);

    double trajectoryDistance = distanceToAB(nextPoint , lastPoint, robotPose);  // left is positive; right is negative;
    double PathAngle = atan2( nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    double aimLength = 0.5;
 
    double controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength)
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    double targetRobotPose = constrainAngle(controlAngle + PathAngle);
    double  controlObject = deltaAngle(targetRobotPose,robotPose.z);  // - odomData.pose.orientation.z;

    static double last_controlObject = controlObject;
    static double conObjectInt = 0;
    static double control_V_start = 0;

    PonitXYZ commandVelocity;

    if(refreshPIDFlag)
    {
        last_controlObject = controlObject;
        conObjectInt = 0;
        control_V_start = 0;

        refreshPIDFlag = false;
    }
    
    /************************************************************************************************************************/
    if( fabs(controlObject) < 5 * PI / 180 )
    {
        conObjectInt = conObjectInt + controlObject;
    }
    commandVelocity.y = controlPID(controlObject, conObjectInt, last_controlObject, 0.5, 0.01, 0);
    last_controlObject = controlObject;

    if (fabs(commandVelocity.y) > 0.5)
        commandVelocity.y = 0.5 * sign( commandVelocity.y );

    // if boundry mode, limit velocity
    double deadLineDistance = targetVector.transpose() * pathVector;
    ROS_INFO("deadLine = %f, toLine = %f, target = %f.",deadLineDistance, trajectoryDistance, targetRobotPose); 

    commandVelocity.x = 0.5*deadLineDistance;

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.3)
    {
         commandVelocity.x = 0.3 * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.15)
    {
         commandVelocity.x = 0.15*sign(commandVelocity.x);
    }
    
    commandVelocity.x = commandVelocity.x*cos(controlObject);

    // need acceleration from scratch
    // control_V_start = control_V_start + 0.35/10.0;
    // if ( fabs(commandVelocity.x) > control_V_start)
    // {
    //     commandVelocity.x = control_V_start * sign(commandVelocity.x);
    // }

    if(deadLineDistance > 0.1)
    {
        targetStates.linear.x = commandVelocity.x;
        targetStates.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        pauseFlag = true;
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }
}

bool zeroTurning(geometry_msgs::Point nextPoint,geometry_msgs::Point lastPoint)
{
    double targetAngle = atan2(nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    double deltaTheta = deltaAngle(targetAngle,robotPose.z);

    ROS_INFO("target angle is %f ",targetAngle*180/PI);

    PonitXYZ commandVelocity;
    commandVelocity.x = 0;
    commandVelocity.y = deltaTheta * 2 / PI;

    if(fabs(commandVelocity.y)<0.5)
    {
        commandVelocity.y = 0.5*sign(deltaTheta);
    }


    if(fabs(deltaTheta) > 20*PI/180)
    {
        targetStates.linear.x = commandVelocity.x;
        targetStates.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        pauseFlag = true;
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }

    // spdlog::get("robot_status")->info("motion control states ----v:{}, w:{}",targetStates.linear.x,targetStates.angular.z); 
}

bool motionControl(vector<PonitXYZ> pathPoint, int num)
{
    static vector<double> imuAngle(num);
    static vector<double> uwbAngle(num);

    static int motionStep = 0;

    static int passNum = 0;
    static geometry_msgs::Point nextPoint;
    static geometry_msgs::Point lastPoint;

    static geometry_msgs::Point imuPointA;
    static geometry_msgs::Point imuPointB;
    static geometry_msgs::Point uwbPointA;
    static geometry_msgs::Point uwbPointB;
    
    static int startCounter = 0;

    int initialNum = 10;

    if(motionStep == 0)
    {
        int nextNum = (passNum+1) % num;
        int lastNum = passNum % num;
        
        nextPoint.x = pathPoint[nextNum].x;
        nextPoint.y = pathPoint[nextNum].y;
        nextPoint.z = pathPoint[nextNum].z;

        lastPoint.x = pathPoint[lastNum].x;
        lastPoint.y = pathPoint[lastNum].y;
        lastPoint.z = pathPoint[lastNum].z;

        imuPointA.x = 0;
        imuPointA.y = 0;

        uwbPointA.x = 0;
        uwbPointA.y = 0;

        imuPointB.x = 0;
        imuPointB.y = 0;

        uwbPointB.x = 0;
        uwbPointB.y = 0;

        motionStep = 1;
    }

    ROS_INFO(" robot point is: x = %f , y = %f, theta = %f .",robotPose.x,robotPose.y,robotPose.z*180/PI);

    if(motionStep == 1)
    {
        if(zeroTurning(nextPoint,lastPoint))
        {
            motionStep = 2;
        }
    }

    if(motionStep == 2)
    {
        if(gotPositionUWB && fabs(odomData.pose.position.y)<0.05)
        {
            startCounter++;

            imuPointA.x = imuPointA.x + robotPose.x/initialNum;
            imuPointA.y = imuPointA.y + robotPose.y/initialNum;

            uwbPointA.x = uwbPointA.x + PositionUWB(0)/initialNum;
            uwbPointA.y = uwbPointA.y + PositionUWB(1)/initialNum;
        }

        if(startCounter >= initialNum)
        {
            ROS_INFO(" point A : x = %f , y = %f.",uwbPointA.x,uwbPointA.y);
            motionStep = 3;
            startCounter = 0;
        }
    }

    if(motionStep == 3)
    {
        if(straightMotion(nextPoint,lastPoint))
        {
            motionStep = 4;
        }
    }

    if(motionStep == 4)
    {
        if(gotPositionUWB && fabs(odomData.pose.position.y)<0.05)
        {
            startCounter++;

            imuPointB.x = imuPointB.x + robotPose.x/initialNum;
            imuPointB.y = imuPointB.y + robotPose.y/initialNum;

            uwbPointB.x = uwbPointB.x + PositionUWB(0)/initialNum;
            uwbPointB.y = uwbPointB.y + PositionUWB(1)/initialNum;
        }

        if(startCounter >= initialNum)
        {
            ROS_INFO(" point B : x = %f , y = %f.",uwbPointB.x,uwbPointB.y);
            motionStep = 5;
            startCounter = 0;
        }
    }

    if(motionStep == 5)
    {
        imuAngle[passNum] = atan2(imuPointB.y - imuPointA.y,imuPointB.x - imuPointA.x);
        uwbAngle[passNum] = atan2(uwbPointB.y - uwbPointA.y,uwbPointB.x - uwbPointA.x);

        ROS_INFO("NO.%d angle imuAngle = %f, uwbAngle = %f",passNum,imuAngle[passNum]*180/PI,uwbAngle[passNum]*180/PI);

        passNum++;
        motionStep = 0;
        if(passNum >= num)
        {
            double deltaTheta = deltaAngle(uwbAngle[0],imuAngle[0]);
            double initialOffsetIMU = deltaTheta;
            for(int i = 1; i < num ; i++)
            {
                initialOffsetIMU += deltaAngle(deltaAngle(uwbAngle[i],imuAngle[i]),deltaTheta)/num;
                ROS_INFO("Now IMU angle = %f, angle delta = %f",PositionDR(2)*180/PI,deltaAngle(uwbAngle[i],imuAngle[i])*180/PI);
            }

            initialPose.pose.orientation.z = constrainAngle( PositionDR(2) + initialOffsetIMU );
            initialPose.pose.position.x = uwbPointB.x - 0.21*cos( initialPose.pose.orientation.z );
            initialPose.pose.position.y = uwbPointB.y - 0.21*sin( initialPose.pose.orientation.z ); 
            initialPose.pose.position.z = 0;
            
            ROS_INFO("Now IMU angle = %f, angle offset = %f",PositionDR(2)*180/PI,initialOffsetIMU*180/PI);

            ROS_INFO("DR position initialization complete: X = %f, Y= % f,theta = %f",initialPose.pose.position.x,initialPose.pose.position.y, initialPose.pose.orientation.z*180/PI);

            return true;
        }
    }

    ROS_INFO("motion step is %d",motionStep);

    gotPositionUWB = false;

    return false;
}

PonitXYZ normalizationVector(PonitXYZ data)
{
    double length = sqrt(data.x*data.x + data.y*data.y + data.z*data.z);
    PonitXYZ result;
    result.x = data.x/length;
    result.y = data.y/length;
    result.z = data.z/length;
    return result;
}


bool initialRillPitch()
{
    static int pointNum = 0;

    int initialNum = 100;

    static vector<PonitXYZ> accData(initialNum);
    static vector<PonitXYZ> gyroData(initialNum);

    bool leftLowSpeed = (fabs(odomData.pose.orientation.z)<0.05)?true:false;
    bool rightLowSpeed = (fabs(odomData.pose.orientation.w)<0.05)?true:false;

    if(leftLowSpeed && rightLowSpeed)
    {
        accData[pointNum].x = imuData.pose.position.x;
        accData[pointNum].y = imuData.pose.position.y;
        accData[pointNum].z = imuData.pose.position.z;

        accData[pointNum] = normalizationVector(accData[pointNum]);

        gyroData[pointNum].x = imuData.pose.orientation.x;
        gyroData[pointNum].y = imuData.pose.orientation.y;
        gyroData[pointNum].z = imuData.pose.orientation.z;

        pointNum++;
    }

    if(pointNum>= initialNum)
    {
        PonitXYZ accAverage;
        accAverage.x = 0;
        accAverage.y = 0;
        accAverage.z = 0;
        for(int i = 0 ; i < pointNum; i++)
        {
            accAverage.x += accData[i].x/pointNum;
            accAverage.y += accData[i].y/pointNum;
            accAverage.z += accData[i].z/pointNum;
        }

        initialPose.pose.orientation.x = atan2(accAverage.y,accAverage.z);
        initialPose.pose.orientation.y = asin(accAverage.x);

        ROS_INFO(" initial roll = %f , pitch = %f.",initialPose.pose.orientation.x*180/PI, initialPose.pose.orientation.y*180/PI);

        return true;
    }

    targetStates.linear.x = 0;
    targetStates.angular.z = 0;

    return false;
}

bool initialRobotPose()
{
    static int motionStep = 0;
    bool taskFinish = false;

    static vector<PonitXYZ> pathPoint(10);

    if(motionStep == 0)
    {
        double startAngle = robotPose.z;
        double moveX = 2.0;
        double moveY = 6.0;

        pathPoint[0].x = robotPose.x;
        pathPoint[0].y = robotPose.y;
        pathPoint[0].z = robotPose.z;

        pathPoint[1].x = pathPoint[0].x + moveY*cos(robotPose.z);
        pathPoint[1].y = pathPoint[0].y + moveY*sin(robotPose.z);
        pathPoint[1].z = constrainAngle(robotPose.z);

        pathPoint[2].x = pathPoint[1].x + moveX*cos(robotPose.z - PI/2);
        pathPoint[2].y = pathPoint[1].y + moveX*sin(robotPose.z - PI/2);
        pathPoint[2].z = constrainAngle(robotPose.z - PI/2);

        pathPoint[3].x = pathPoint[2].x + moveY*cos(robotPose.z - PI);
        pathPoint[3].y = pathPoint[2].y + moveY*sin(robotPose.z - PI);
        pathPoint[3].z = constrainAngle(robotPose.z - PI);

        motionStep = 1;

        ROS_INFO(" startPoint: x = %f , y = %f , theta = %f.",robotPose.x,robotPose.y,robotPose.z);
    }

    if(motionStep == 1)
    {
        if(motionControl(pathPoint,4))
        {
            motionStep = 2; // finish initial yaw and position(x,y)
        }
    }

    if(motionStep == 2)
    {
        if(initialRillPitch())
        {
            motionStep = 3; // finish initial roll and pitch 
        }
    }

    if(motionStep == 3)
    {
        initialPose.header.stamp = ros::Time::now();
        initialPose.header.frame_id = "InitialState";
        initial_pub.publish(initialPose);

        initialFinish = true;

        taskFinish = true;
        motionStep = 0;
    }

    vel_pub.publish(targetStates);

    return taskFinish;
}

void poseCallback(geometry_msgs::PoseArray msg_alf)
{
    ros::Time currentTime=ros::Time::now();
    odomData.header.stamp = currentTime;
    
    odomData.pose.position.x = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y); // mileage
    odomData.pose.position.y = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w); // velocity
    odomData.pose.position.z = (msg_alf.poses[1].orientation.w - msg_alf.poses[1].orientation.z)/0.32;

    odomData.pose.orientation.x = msg_alf.poses[1].orientation.x; // left wheel mileage
    odomData.pose.orientation.y = msg_alf.poses[1].orientation.y; // right wheel mileage
    odomData.pose.orientation.z = msg_alf.poses[1].orientation.z; // left wheel velocity
    odomData.pose.orientation.w = msg_alf.poses[1].orientation.w; // right wheel velocity

    imuData.header.stamp = currentTime;

    imuData.pose.position.x = msg_alf.poses[1].position.x; // acceleration data
    imuData.pose.position.y = msg_alf.poses[1].position.y;
    imuData.pose.position.z = msg_alf.poses[1].position.z;

    imuData.pose.orientation.x = msg_alf.poses[0].orientation.x; // gyro data
    imuData.pose.orientation.y = msg_alf.poses[0].orientation.y;
    imuData.pose.orientation.z = msg_alf.poses[0].orientation.z;
    imuData.pose.orientation.w = msg_alf.poses[0].orientation.w; // temperature

    eulerAngle.header.stamp = currentTime;
    eulerAngle.point.x =  msg_alf.poses[0].position.x; //euler angle
    eulerAngle.point.y =  msg_alf.poses[0].position.y;
    eulerAngle.point.z =  msg_alf.poses[0].position.z;

    locationDR();
    robotPoseCallback();

    refeshIMU = true;
}

void uwbPositionCallback(geometry_msgs::PointStamped msg_point)
{
    PositionUWB(0) = msg_point.point.x;
    PositionUWB(1) = msg_point.point.y;
    PositionUWB(2) = msg_point.point.z; 

    if( ( fabs(msg_point.point.z) > 0.01 ) && ( fabs(msg_point.point.z) < 0.5 ) )
    {
        gotPositionUWB = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial sensor");
    ROS_INFO_STREAM("initial sensor is initialing...");

    auto status_pointger = spdlog::rotating_logger_mt("robot_status",file_status,1048576*5,3);
    status_pointger->info("\n\n\n---------------- The initial sensor is starting-----------------------");

    ros::NodeHandle nh;

    //Publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    initial_pub = nh.advertise<geometry_msgs::PoseStamped>("/init_sensor", 1);

    //Subcriber
    ros::Subscriber pose_sub  = nh.subscribe("/alf001_dis", 1,poseCallback);
    ros::Subscriber uwb_sub = nh.subscribe("/uwb_pose", 1, uwbPositionCallback);

    ros::Rate loop_rate(10);
    ros::Time prev_whileloop = ros::Time::now();

    while(ros::ok())
    {
        if((!initialFinish) && refeshIMU )
        {
            initialRobotPose();
        }

        ros::spinOnce();
        // loop_rate.sleep();

        if (pauseFlag)
        {
            ros::Duration(1.0).sleep(); // sleep for 1.0 second
            pauseFlag = false;
        }
        else
        {
            ros::Duration(0.1).sleep(); // sleep for 0.1 second
        }

        double dt = ros::Time::now().toSec() - prev_whileloop.toSec();
        prev_whileloop = ros::Time::now();
    }

    return 0;
}

std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}
