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

using namespace std;

//navigation frame: x to the right side, y to the up, anti-clockwise as positive

std::string getDate();
//point file
std::string point_path = "/home/raspberry002/catkin_ws/src/uwb_zigzag/data/";
std::string file_status = point_path + getDate() + "log.txt";

enum targetPointType
{
    MAP,
    PATHT,
    PATHV
};

struct PathDataType
{
    float x;
    float y;
    float theta;
    float v;
    float w;
};

typedef  struct
{
	double x; 
	double y;
    double z;
} PonitXYZ; 

//read map file
std::string file_map = point_path +"map_edge.txt";
vector<PonitXYZ> mapPoint(10000);
int mapPointNum = 0;

int pathType=0;

std::string file_pathT = point_path +"pathT.txt";
std::string file_pathV = point_path +"pathV.txt";
vector<PonitXYZ> pathPoint(10000);
int pathPointNum = 0;

bool isDigitalString(char *data)
{
    return (strspn(data, "0123456789.-") == strlen(data));
}

/******************* ros publisher ****************************/

ros::Publisher vel_pub_;
ros::Publisher arrive_point_pub;

bool init_uwb=false;
Eigen::Vector2d initial_uwb(0,0);

nav_msgs::Path path;    //robot path
bool pauseFlag=false;
geometry_msgs::PoseStamped odomData;

//robot 
geometry_msgs::Twist targetStates;

/******************* controller parametre ****************************/
bool refeshIMU = false;

geometry_msgs::Point robotPose;

/****************************************************************************************/

// deal with the margin -180 <=> 180
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


bool turnedTarget(Eigen::Vector3d targetPoint, Eigen::Vector3d lastPoint,float turnDoneAngle)
{
    double a = targetPoint(0)-lastPoint(0);
    double b = targetPoint(1)-lastPoint(1);

    double targetAngle = atan2( b , a );

    bool turnedFlag = ( fabs(deltaAngle( targetAngle , robotPose.z )) < turnDoneAngle )?true:false;

    return turnedFlag;
}

bool arrivedTarget(PathDataType targetPoint, PathDataType lastPoint,float moveDoneLength)
{
    float pathDistance,deltaDistance;
    Eigen::Vector2d pathVector(0,0);
    Eigen::Vector2d targetVector(0,0);
    
    double a = targetPoint.x - lastPoint.x;
    double b = targetPoint.y - lastPoint.y;
    pathDistance = sqrt( a*a + b*b );

    pathVector(0) = a / pathDistance;
    pathVector(1) = b / pathDistance;
    
    targetVector(0) = targetPoint.x - robotPose.x;
    targetVector(1) = targetPoint.y - robotPose.y;

    deltaDistance = ( pathVector.transpose() )*targetVector;

    bool arrivedFlag = ( deltaDistance < moveDoneLength )?true:false;

    return arrivedFlag;
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

    if(refeshIMU)
    {
        last_controlObject = controlObject;
        conObjectInt = 0;
        control_V_start = 0;

        refeshIMU = false;
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


bool waitTimes(int waitNum)
{
    static int num=0;
    
    num++;
    
    if(num >= waitNum)
    {
        num=0;
        return true;
    }
    else
    {
        return false;
    } 
}

bool motionControl(geometry_msgs::Point nextPoint,geometry_msgs::Point lastPoint)
{
    static int motionStep = 0;

    ROS_INFO("the next point : x = %f, y = %f",nextPoint.x,nextPoint.y);
    ROS_INFO("the last point : x = %f, y = %f",lastPoint.x,lastPoint.y);

    // judge the motion type
    if( motionStep == 0 )
    {
        motionStep = 1; // straightMotion
    }

    // turn to the last point angle
    if( motionStep == 1 )
    {
        // spdlog::get("robot_status")->info("turning to the target...");
        if( zeroTurning(nextPoint, lastPoint) )  
        {
            motionStep = 2;
            // spdlog::get("robot_status")->info("turning done.");
        }
    }

    // wait for one second 
    if( motionStep == 2 )
    {
        if( waitTimes(10) )
        {
            motionStep = 3;
        }
        else
        {
            targetStates.linear.x = 0;
            targetStates.angular.z = 0;
        }
    }

    // straightMotion and curveMotion
    if( motionStep == 3 )
    {
        // spdlog::get("robot_status")->info("going straight motion to the target...");
        if( straightMotion(nextPoint,lastPoint) )
        {
            motionStep = 4;
        }
    }

    // judge
    if( motionStep == 4 )
    {
        if( waitTimes(10) )
        {
            motionStep = 1;
            return true; // finish motion control
        }
        else
        {
            targetStates.linear.x = 0;
            targetStates.angular.z = 0;
        }
    } 
 
    return false;
}

bool zigzagMotion()
{
    static geometry_msgs::Point nextpoint;
    static geometry_msgs::Point lastpoint;
    static geometry_msgs::Point robotBegainPoint;

    static bool onPathFlag=false;
    static bool startFlag=false;

    static int pointNum = 0;

    // record the robot start point
    if(!startFlag)
    {
        robotBegainPoint.x = robotPose.x;
        robotBegainPoint.y = robotPose.y;
        robotBegainPoint.z = atan2(pathPoint[pointNum].y - robotPose.y, pathPoint[pointNum].x - robotPose.x);

        lastpoint = robotBegainPoint; 
        nextpoint.x = pathPoint[pointNum].x;
        nextpoint.y = pathPoint[pointNum].y;
        nextpoint.z = atan2(nextpoint.y - lastpoint.y, nextpoint.x - lastpoint.x);
        startFlag = true;   
    }
    
    // zigzag motion control
    if ( motionControl(nextpoint , lastpoint) )
    {
        //move done, refresh a new point
        // spdlog::get("robot_status")->info("waiting for a new target...");
        
        ROS_INFO("zigzag motion : arrive one path point ...");

        onPathFlag = true;

        pointNum++;

        if(pointNum < pathPointNum)
        {
            // refesh the target point
            lastpoint = nextpoint;
            nextpoint.x = pathPoint[pointNum].x;
            nextpoint.y = pathPoint[pointNum].y;
            nextpoint.z = atan2(nextpoint.y - lastpoint.y, nextpoint.x - lastpoint.x);
        }
        else
        {
            targetStates.linear.x = 0;
            targetStates.angular.z = 0;
            vel_pub_.publish(targetStates);

            pauseFlag = true;
            return true;
        }
    }

    vel_pub_.publish(targetStates);
    return false;
}


void pathRead(targetPointType fileIndex)
{
    std::string line;
    char *dataString;
    std::string file_path;


    char mapString[] = "map";
    char pathTString[] = "pathT";
    char pathVString[] = "pathV";

    bool isPathData = false;

    switch (fileIndex)
    {
        case MAP:
            dataString = mapString;
            file_path = file_map;
            isPathData = false;
            break;
        case PATHT:
            dataString = pathTString;
            file_path = file_pathT;
            isPathData = true;
            break;
        case PATHV:
            dataString = pathVString;
            file_path = file_pathV;
            isPathData = true;
            break;
        default:
            ROS_ERROR("Error file read index .");
    }

    std::ifstream pathStream(file_path.c_str(), ios::in);

    //path data loading
    if(!pathStream.is_open())
            ROS_INFO("Can not open %s.txt.",dataString);

    vector<float> digitalData(5);

    while(!pathStream.eof() && getline(pathStream, line))
    {
        char *stringData,*startLine;
        // for(int i=0; i < line.size();i++)
        // {
        //     startLine[i] = line[i];
        // }
        startLine = &(line[0]);
        
        int dataNum = 0;
        while ((stringData = strtok(startLine," ")) != NULL && dataNum < 5)
        {
            if ( strspn(stringData, "0123456789.-") == strlen(stringData) )
            {
                digitalData[dataNum] = atof(stringData);
            }
            else
            {
                break;
            }

            startLine = NULL;
            dataNum++;
        }

        if (dataNum == 2)
        {
            if(isPathData)
            {
                pathPoint[pathPointNum].x = digitalData[0];
                pathPoint[pathPointNum].y = digitalData[1];

                pathPointNum++;
            }
            else
            {
                mapPoint[mapPointNum].x = digitalData[0];
                mapPoint[mapPointNum].y = digitalData[1];     
                mapPointNum++;
            }
            ROS_INFO("%s points : x= %f, y= %f", dataString, digitalData[0],  digitalData[1]);
        }   
    }
    ROS_INFO("%s data read end.",dataString);
}

void pathCallback()//geometry_msgs::Point msg
{

    ROS_INFO("map point reader is starting...");
    pathRead(MAP);
    switch (pathType)
    {
        case 0:
            pathRead(PATHT);
            break;
        case 1:
            pathRead(PATHV);
            break;
        case 2:
            pathRead(PATHT);
            pathRead(PATHV);
            break;
        case 3:
            pathRead(PATHV);
            pathRead(PATHT);
            break;
        default:
            ROS_ERROR("Error path type.");
            break;
    }
    ROS_INFO("map point reader is ending...");
}

void ekfPositionCallback(geometry_msgs::PointStamped msg_ekf)
{
    robotPose.x = msg_ekf.point.x;
    robotPose.y = msg_ekf.point.y;
    robotPose.z = msg_ekf.point.z; //(-180,180]
}

void initialCallback(geometry_msgs::PoseStamped msg_initial)
{
    robotPose.x =  msg_initial.pose.position.x;
    robotPose.y =  msg_initial.pose.position.y;
    robotPose.z =  msg_initial.pose.orientation.z;

    ROS_INFO("motion get initial point: x = %f, y = %f , theta = %f",robotPose.x, robotPose.y, robotPose.z*180/PI);

    init_uwb = true;
}

void ros_get_params(ros::NodeHandle nh)
{
    if(nh.getParam("/motion_uwb/pathType",pathType))
    {
        ROS_INFO("Got pathType:%d!",pathType);
    }
    else
    {
        ROS_ERROR("Failed to get param 'pathType'.");
        pathType = 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion control");
    ROS_INFO_STREAM("motion control is starting...");

    ros::NodeHandle nh;
    
/********* get PID controler parametre from mower_param.yaml ********/
    system("pwd");
    ros_get_params(nh);
/********       end of getting PID controler parametre      *********/

    auto status_pointger = spdlog::rotating_logger_mt("robot_status",file_status,1048576*5,3);
    status_pointger->info("\n\n\n---------------- The zigzag mower is starting-----------------------");

    //Publisher
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //Subcriber
    ros::Subscriber uwb_sub = nh.subscribe("/ekf_pose", 1, ekfPositionCallback);
    ros::Subscriber initial_sub  = nh.subscribe("/init_sensor", 1,initialCallback);

    //Synchronizer
    static bool fenceDoneFlag=false;
    static bool zigzagDoneFlag=false;

    pathCallback();

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        // motion control
        if(init_uwb)
        {
            if(zigzagDoneFlag)
            {
                init_uwb = false;
                ROS_INFO("fencing and zigzag motion has finished.");
            }
            else
            {
                if(zigzagMotion())
                {
                    zigzagDoneFlag = true;
                    ROS_INFO("zigzag motion has finished.");
                }
            }
        }

        ros::spinOnce();
        
        // puse one second
        if (pauseFlag)
        {
            for(int i=0;i<0.6667*10e7;i++)
            ;
            pauseFlag=false;
        }

        loop_rate.sleep();
    }

    status_pointger->flush();
    spdlog::drop_all();
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
