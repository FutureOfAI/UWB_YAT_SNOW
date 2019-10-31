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
#include <vector>

//spdlog
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

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

#define PI 3.14159265358979323846

//typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, geometry_msgs::PoseStamped> sync_policy_classifiction;
//get the current date //date -s "20180929 08:00:00"

#define  NumUWB 4
#define  NumStation 6
#define yawFilterNum 20
#define indexOffset 0.05

using namespace std;

std::string getDate();
std::string point_path = "/home/raspberry002/catkin_ws/src/uwb_zigzag/data/";
std::string file_location = point_path + getDate() + "location.txt";
std::ofstream fout_point(file_location.c_str());

ros::Publisher ekf_pub;
ros::Publisher uwb_pub;

geometry_msgs::PointStamped eulerData;
geometry_msgs::PoseStamped odomData;
geometry_msgs::PoseStamped imuData;
geometry_msgs::PoseStamped uwbData;

bool flag_imucb=false;
bool flag_uwbcb=false;
bool flag_odomcb=false;

float rob_theta0;            //face to the right north

bool gotPositionUWB;


float DeltaHigh=1.4;
float TagHigh=0.57;
float PrecisionThresholdUWB=1;
float PrecisionUWB=100;
float enableVelUWB=0.2;
float biasThreshold = 0.1;
float biasFilterIndex = 0.1;
Eigen::Matrix3d GeometricMatix(3,3);
Eigen::Matrix3d PrecisionIndex(3,3);

bool sensorInitialed = false;

/******* UWB  parametre*********/
bool enableYawUWB = false;
float orientationUWB = 0;
int yawOffsetPeriod = 30;
Eigen::Vector4d distanceBias(0,0,0,0);

struct PointTypeXYZ
{
    float x;
    float y;
    float z;
};

vector<PointTypeXYZ> BaseStationPoint(NumStation);
float lengthBaseStation[ (NumUWB-1)*NumUWB ];

int BaseStationType; // # description : 0 is no BS information, 1 is use position, 2 is used length

/******* UWB location parametre*********/
float alphaParametre=0.05;

Eigen::Vector3d PositionUWB(0,0,0);
Eigen::Vector3d FilteredPositionUWB(0,0,0);

/******* dead recking location parametre*********/
Eigen::Vector3d PositionDR(0,0,PI/2);

/******* Kalman filter parametre*********/
bool InitFlagKF=false;

Eigen::Vector2d PositionKF(0,0);
Eigen::Matrix2d P(2,2);
Eigen::Matrix2d Q(2,2);  // system noise covariance
Eigen::Matrix2d R(2,2);  // observation noise covariance
Eigen::Matrix2d A(2,2);
Eigen::Matrix2d H(2,2);
Eigen::Matrix2d K(2,2);

float constrainAngleRad(float angle)
{
    float trueAngle;
    if (angle > PI)
    {
        trueAngle = angle - 2*PI;
    }
    else if (angle < -PI)
    {
        trueAngle = angle + 2*PI;
    }
    else
    {
        trueAngle =angle;
    }

    return trueAngle;
}

// deal with the margin -180 <=> 180
float calculateDeltaAngle(float TargetAngle,float CurrentAngle)
{
    float DeltaAngle;
    DeltaAngle = TargetAngle - CurrentAngle; 
    if ( fabs(DeltaAngle) > 2*PI)
    {
        if( (TargetAngle>0) && (CurrentAngle<0) )
        {
            DeltaAngle = TargetAngle - CurrentAngle -2*PI;
        }
        else if( (TargetAngle<0) && (CurrentAngle>0) )
        {
            DeltaAngle = 2*PI - CurrentAngle + TargetAngle;
        }
    } 
    return DeltaAngle;
}

float calculateYawUWB(double pointForYaw[yawFilterNum][2])
{
    float yawUWB; 
    Eigen::Vector2d sumA(0,0);
    Eigen::Vector2d sumB(0,0);

    for(int m = 0 ; m < 5 ; m++) 
    {
        sumA(0) = sumA(0) + pointForYaw[m][0];
        sumA(1) = sumA(1) + pointForYaw[m][1];
    }

    for(int n = yawFilterNum - 5; n < yawFilterNum ; n++) 
    {
        sumB(0) = sumB(0) + pointForYaw[n][0];
        sumB(1) = sumB(1) + pointForYaw[n][1];
    }

    double delta_x = ( sumB(0) - sumA(0) ) / 5;
    double delta_y = ( sumB(1) - sumA(1) ) / 5;

    yawUWB=atan2( delta_y, delta_x);

    // ROS_INFO("Yaw from UWB location is %f.",yawUWB*180/PI);

   return yawUWB;
} 

double compensationYaw()
{
    static double yawOffsetUWB = 0;
    static double yawIMU[yawFilterNum];

    static double pointForYaw[yawFilterNum][2];
    static int numberPoint = 0;
    static int timesLowPass = 0;

    bool gotYawUWB = false;
    float differenceAngle;
    
    // record the yaw from IMU
    for(int i=0;i<(yawFilterNum-1);i++)
    {
        yawIMU[i]=yawIMU[i+1];
    }
    yawIMU[yawFilterNum-1]=PositionDR(2);

    float deltaVelocity = 0.32*fabs(odomData.pose.position.z);
    float averageVelocity = fabs(odomData.pose.position.y);
    //
    if( (deltaVelocity < 0.05) && (averageVelocity > 0.2) )
    {
        // record the yaw from UWB
        for(int i=0;i<yawFilterNum-1;i++)
        {
            pointForYaw[i][0]=pointForYaw[i+1][0];
            pointForYaw[i][1]=pointForYaw[i+1][1];
        }
        pointForYaw[yawFilterNum-1][0]=FilteredPositionUWB(0);
        pointForYaw[yawFilterNum-1][1]=FilteredPositionUWB(1);

        numberPoint++;

        if(numberPoint>=yawFilterNum)
        {
            orientationUWB = calculateYawUWB(pointForYaw);
            gotYawUWB = true;
        }
    }
    else
    {
        numberPoint=0;
    }

    if(gotYawUWB)
    {
       differenceAngle = calculateDeltaAngle( orientationUWB , yawIMU[yawFilterNum/2-1] );
       yawOffsetUWB = (1-indexOffset)*yawOffsetUWB + indexOffset*differenceAngle;
       timesLowPass++;
    }

    if(timesLowPass > yawOffsetPeriod*10)
    {
        enableYawUWB = true;
        // PositionDR(2)=PositionDR(2)+yawOffsetUWB;
        // PositionDR(2)=constrainAngleRad(PositionDR(2));
        yawOffsetUWB = 0;
        timesLowPass = 0;
    }
    else
    {
        enableYawUWB = false;
    }
    
    /***********************************************************************************************************************************/ 
    return yawOffsetUWB;
}

float distanceR(float X0,float Y0,float Z0,float X1,float Y1,float Z1)
{
    float r = sqrt( pow(X0-X1,2) + pow(Y0-Y1,2) + pow(Z0-Z1,2) );
    return r;
}

bool gotBaseStationPosition()
{
    //get base stations position parametre from baseStationPosition.yaml
    bool GotBaseType=false;
    double AB,BC,CA,AD,BD,CD;
    double costheta1,theta1,costheta2,theta2;
    AB = lengthBaseStation[0];
    BC = lengthBaseStation[1];
    CA = lengthBaseStation[2];
    AD = lengthBaseStation[3];
    BD = lengthBaseStation[4];
    CD = lengthBaseStation[5];
    
    switch(BaseStationType)
    {
        case 0:
            // Fail to UWB base stations position
            GotBaseType=false;
            break;

        case 1:
            // got UWB base stations position
            GotBaseType=true;
            break;

        case 2:
            {
                BaseStationPoint[0].x = 0;
                BaseStationPoint[0].y = 0;

                BaseStationPoint[1].x = AB;
                BaseStationPoint[1].y = 0;

                costheta1 = (AB*AB+CA*CA-BC*BC)/(2*AB*CA);
                theta1 = acos(costheta1);

                BaseStationPoint[2].x = CA*cos(theta1);
                BaseStationPoint[2].y = CA*sin(theta1);

                costheta2 = (AB*AB+AD*AD-BD*BD)/(2*AB*AD);
                theta2 = acos(costheta2);

                BaseStationPoint[3].x = AD*cos(theta2);
                BaseStationPoint[3].y = AD*sin(theta2);

                GotBaseType=true;
                break;
            }
        default:
            GotBaseType=false;
            // Fail to UWB base stations position
            break;
    }

    if(GotBaseType)
    {
        // print the initial base station position result
        ROS_INFO("initial %d UWB base stations position: ", 4);
        for (int i = 0; i < 4; i++)
        {
            ROS_INFO("NO.%d UWB base station position: X=%f ,Y=%f ",i,BaseStationPoint[i].x,BaseStationPoint[i].y);
        }
        return true;
    }
    else
    {
        ROS_INFO("Fail to UWB base stations position.");
        return false;
    }

}

bool locationQuadUWB(float dataUWB[NumStation],int enableNum[NumStation])
{
    Eigen::Vector3d deltaX(0,0,0);
    Eigen::Vector4d errorVector(0,0,0,0);
    Eigen::MatrixXd MatrixG(4,3);

    int NumBS = enableNum[NumStation-1];

    float X[5], Y[5], Z[5], R[5];

    X[4] = PositionUWB(0);
    Y[4] = PositionUWB(1);
    Z[4] = TagHigh;

    for(int n = 0; n < NumBS ; n++ )
    {
        X[n] = BaseStationPoint[ enableNum[n] ].x;
        Y[n] = BaseStationPoint[ enableNum[n] ].y;
        Z[n] = BaseStationPoint[ enableNum[n] ].z;
    }

    int NumIteration=0;

    while( NumIteration < 50 )
    {
        for(int i = 0; i < NumBS ; i++ )
        {
            R[i] = distanceR(X[4],Y[4],Z[4],X[i],Y[i],Z[i]);
            if ( R[i] < 0.01 )
            {
                R[i] = 0.01;
            }
            errorVector(i) = dataUWB[i] - R[i];

            MatrixG(i,0) = (X[4]-X[i])/R[i];
            MatrixG(i,1) = (Y[4]-Y[i])/R[i];
            MatrixG(i,2) = 1;
            
            // ROS_INFO(" NO.%d UWB data is : %f",i,dataUWB[i]);
        }

        deltaX = (((MatrixG.transpose())*MatrixG).inverse())*(MatrixG.transpose())*errorVector;

        X[4] = X[4] + 0.5*deltaX(0);
        Y[4] = Y[4] + 0.5*deltaX(1);
        //Z[4] = Z[4] + 0.2*deltaX(2);

        if ( (fabs(deltaX(0))<0.005) && (fabs(deltaX(1))<0.005) && (fabs(deltaX(2))<0.005) )
        {
            break;
        }
        NumIteration++;
    }

    PrecisionUWB = sqrt((errorVector.transpose())*errorVector)/2;

    /******************  record the precision parametre *********************/

    GeometricMatix(0,0) = deltaX(0);
    GeometricMatix(0,1) = deltaX(1);
    GeometricMatix(0,2) = deltaX(2);
    // GeometricMatix(1,0) = deltaX(3);
    GeometricMatix(1,1) = errorVector(0);
    GeometricMatix(1,2) = errorVector(1);
    GeometricMatix(2,0) = errorVector(2);
    GeometricMatix(2,1) = errorVector(3);
    GeometricMatix(2,2) = Z[4];

    Eigen::MatrixXd MatrixH(3,3);
    MatrixH =((MatrixG.transpose())*MatrixG).inverse();

    PrecisionIndex(0,0) = MatrixH(0,0);
    PrecisionIndex(0,1) = MatrixH(1,1);
    PrecisionIndex(0,2) = MatrixH(2,2);
    // PrecisionIndex(1,0) = MatrixH(3,3);
    PrecisionIndex(1,1) = 0;
    // PrecisionIndex(1,2) = sqrt(MatrixH(0,0) + MatrixH(1,1) + MatrixH(2,2) + MatrixH(3,3));
    PrecisionIndex(2,0) = sqrt(MatrixH(0,0) + MatrixH(1,1) + MatrixH(2,2));
    PrecisionIndex(2,1) = sqrt(MatrixH(0,0) + MatrixH(1,1));
    // PrecisionIndex(2,2) = sqrt(MatrixH(3,3));

    /****************************  end record ******************************/

    // ROS_INFO("UWB data Precision is : %f",PrecisionUWB);

    if (PrecisionUWB < PrecisionThresholdUWB)
    {
        PositionUWB(0) = X[4];
        PositionUWB(1) = Y[4];
        PositionUWB(2) = deltaX(2);
        
        // ROS_INFO("UWB data error is : %f",Z[4]);
        gotPositionUWB = true;
        
        return true;
    }
    else
    {
        return false;
    }
}

bool locationTripleUWB(float dataUWB[NumStation],int enableNum[NumStation])
{
    float prevPositionX = PositionUWB(0);
    float prevPositionY = PositionUWB(1);

    Eigen::Vector3d deltaX(0,0,0);
    Eigen::Vector3d errorVector(0,0,0);
    Eigen::Matrix3d MatrixG(3,3);

    float X[4],Y[4],Z[4];
    float R[4];

    X[3] = prevPositionX;
    Y[3] = prevPositionY;
    Z[3] = TagHigh;

    for(int n = 0; n < 3 ; n++ )
    {
        X[n] = BaseStationPoint[ enableNum[n] ].x;
        Y[n] = BaseStationPoint[ enableNum[n] ].y;
        Z[n] = BaseStationPoint[ enableNum[n] ].z;
    }

    int NumIteration=0;

    while(NumIteration<20)
    {

        for(int i = 0; i < 3 ; i++ )
        {
            R[i]=distanceR(X[3],Y[3],Z[3],X[i],Y[i],Z[i]);
            if ( R[i] < 0.01 )
            {
                R[i] = 0.01;
            }
            errorVector(i) = dataUWB[i] - R[i];

            MatrixG(i,0) = (X[3]-X[i])/R[i];
            MatrixG(i,1) = (Y[3]-Y[i])/R[i];
            MatrixG(i,2) = 1;
        }

        deltaX=(((MatrixG.transpose())*MatrixG).inverse())*(MatrixG.transpose())*errorVector;

        X[3] = X[3] + 0.5*deltaX(0);
        Y[3] = Y[3] + 0.5*deltaX(1);

        if ((fabs(deltaX(0))<0.005)&&(fabs(deltaX(1))<0.005))
        {
            break; 
        }

        NumIteration++;
    }

    PrecisionUWB=sqrt((errorVector.transpose())*errorVector)/sqrt(3);

    /******************  record the precision parametre *********************/

    GeometricMatix = MatrixG;

    PrecisionIndex(0,0) = errorVector(0);
    PrecisionIndex(0,1) = errorVector(1);
    PrecisionIndex(0,2) = errorVector(2);

    PrecisionIndex(1,0) = deltaX(0);
    PrecisionIndex(1,1) = deltaX(1);
    PrecisionIndex(1,2) = deltaX(2);

    Eigen::Matrix3d MatrixH(3,3);
    MatrixH =((MatrixG.transpose())*MatrixG).inverse();

    PrecisionIndex(2,0) = sqrt(MatrixH(0,0) + MatrixH(1,1) + MatrixH(2,2));
    PrecisionIndex(2,1) = sqrt(MatrixH(0,0) + MatrixH(1,1));
    PrecisionIndex(2,2) = sqrt(MatrixH(2,2));

    /****************************  end record ******************************/

    if (PrecisionUWB<PrecisionThresholdUWB)
    {
        PositionUWB(0) = X[3];
        PositionUWB(1) = Y[3];
        PositionUWB(2) = deltaX(2);

        gotPositionUWB = true;
        
        return true;
    }
    else
    {
        return false;
    }
}

bool locationDualUWB(float dataUWB[NumStation],int enableNum[NumStation])
{
    float prevPositionX = PositionUWB(0);
    float prevPositionY = PositionUWB(1);

    float X1,Y1,Z1,X2,Y2,Z2;
    float R1,R2;

    X1 = BaseStationPoint[ enableNum[0] ].x;
    Y1 = BaseStationPoint[ enableNum[0] ].y;
    Z1 = BaseStationPoint[ enableNum[0] ].z;
    R1 = sqrt( fabs(pow(dataUWB[ enableNum[0] ],2)-pow((Z1-TagHigh),2)) );

    X2 = BaseStationPoint[ enableNum[1] ].x;
    Y2 = BaseStationPoint[ enableNum[1] ].y;
    Z2 = BaseStationPoint[ enableNum[1] ].z;
    R2 = sqrt( fabs(pow(dataUWB[ enableNum[1] ],2)-pow((Z2-TagHigh),2)) );


     if (R1<0.01)
     {
        R1=0.01;
     }
    
     if (R2<0.01)
     {
        R2=0.01;
     }
     /*****************************************************************/
     Eigen::Vector2d vectorAB(X2-X1,Y2-Y1);
     Eigen::Vector2d vectorAC(0,0);
     Eigen::Vector2d vectorN(0,0);
     Eigen::Vector2d vectorCD(0,0);
     Eigen::Vector2d vectorCE(0,0);
     
     Eigen::Vector2d PointA(X1,Y1);
     Eigen::Vector2d PointD(0,0);
     Eigen::Vector2d PointE(0,0);

     double costheta2,theta2;
     
     // calculate the vector AC (point C coordinates)

     double lengthAB=sqrt(vectorAB.transpose()*vectorAB);
     if (lengthAB<0.01)
     {
        lengthAB=0.01;
     }

     costheta2=(lengthAB*lengthAB+R1*R1-R2*R2)/(2*R1*lengthAB);
     if (fabs(costheta2)>1)
     {
        costheta2=costheta2/fabs(costheta2);
     }
     
     theta2=acos(costheta2);
     vectorAC=costheta2*R1*vectorAB/lengthAB;

     // calculate the AB vertical unit vector
     vectorN(0)=-(Y2-Y1)/lengthAB;
     vectorN(1)=(X2-X1)/lengthAB;

     // calculate the vector CD and vector CE (point D and E coordinates)
     double lengthAC=sqrt(vectorAC.transpose()*vectorAC);
     vectorCD=sin(theta2)*R1*vectorN;
     vectorCE=-sin(theta2)*R1*vectorN;

     PointD=PointA+vectorAC+vectorCD;
     PointE=PointA+vectorAC+vectorCE;
     /*****************************************************************/     
     
     // respectively calculate distance from two possible point to PositionKF 
     float D1,D2;
     D1=sqrt(((PositionKF-PointD).transpose())*(PositionKF-PointD));
     D2=sqrt(((PositionKF-PointE).transpose())*(PositionKF-PointE));

     //ROS_INFO("use 2 base stations for location:");
     //ROS_INFO("point D  is: x=%f, y=%f.",PointD(0),PointD(1));
     //ROS_INFO("point E  is: x=%f, y=%f.",PointE(0),PointE(1));

     // judge if point D or point E can be used
     if ((D1<0.5)||(D2<0.5))
     {
         // judge which one can be used
         if (D1<D2)
         {     
            PositionUWB(0)=PointD(0);
            PositionUWB(1)=PointD(1);
            PositionUWB(2)=D1;
         }
         else
         {
            PositionUWB(0)=PointE(0);
            PositionUWB(1)=PointE(1);
            PositionUWB(2)=D2;
         }

        //ROS_INFO("the result is x=%f, y=%f.",PositionUWB(0),PositionUWB(1));

        gotPositionUWB=true;
        
        return true;

     }
     else
     {
         // neither point D nor point E can be used
         //ROS_INFO("neither point D nor point E can be used.");

         return false;
     }
}

bool checkDataUWB(float dataUWB[NumStation],int enableNum[NumStation])
{
    bool goodFlag = true;

    if( enableNum[ NumStation-1 ] < 1 )
    {
        return false;
    }

    double deltaHigh = 0;

    for(int i = 0 ; i < enableNum[ NumStation-1 ] ; i++)
    {
        deltaHigh = BaseStationPoint[ enableNum[i] ].z - TagHigh ;
        if( dataUWB[ enableNum[i] ] <  fabs(deltaHigh) )
        {
            goodFlag = false;
            break;
        }
    }
    
    return goodFlag;
}

Eigen::Vector3d locationFilterUWB()
{

    static bool FilterInitialed=false;
    static Eigen::Vector3d FilteredPosition(0,0,0);

    if(FilterInitialed)
    {
        if (fabs(odomData.pose.position.y)>0.1)
        {
            // robot is moving
             FilteredPosition=PositionUWB;
        }
        else
        {
            FilteredPosition=(1-alphaParametre)*FilteredPosition+alphaParametre*PositionUWB;
        }
    }
    else
    {
        // initial the filter data
       FilteredPosition=PositionUWB;
       FilterInitialed=true;
    }

    return FilteredPosition;
}

void correctDataUWB(float dataUWB[NumStation],int enableNum[NumStation])
{
    static ros::Time prev_time=ros::Time::now();
        // calculate delta time
    double dt=ros::Time::now().toSec()-prev_time.toSec();
    double a,b,c,d,distanceError;

    for(int n = 0; n < enableNum[NumStation-1] ; n++)
    {
        a = BaseStationPoint[ enableNum[n] ].x - ( PositionDR(0) + 0.22*cos(PositionDR(2)) );
        b = BaseStationPoint[ enableNum[n] ].y - ( PositionDR(1) + 0.22*sin(PositionDR(2)) );
        c = BaseStationPoint[ enableNum[n] ].z - TagHigh;
        d = sqrt(a*a+b*b+c*c);

        distanceError = dataUWB[ enableNum[n] ] - d;

        // distanceBias(n) = (1-biasFilterIndex)*distanceBias(n) + biasFilterIndex*distanceError(n);
        distanceBias( enableNum[n] ) = 0.5 * distanceError;
        // ROS_INFO(" UWB %d distance baise is %f",n, distanceBias(n));
    }

    prev_time=ros::Time::now(); 
}

void uwbPositionPubish()
{
    geometry_msgs::PointStamped PositionPub;

    static ros::Time current_time;
    current_time=ros::Time::now();

    PositionPub.header.stamp = current_time;
    PositionPub.header.frame_id = "PositionUWB";
    PositionPub.point.x=PositionUWB(0);
    PositionPub.point.y=PositionUWB(1);
    PositionPub.point.z=PositionUWB(2);

    uwb_pub.publish(PositionPub);
}

void uwbPositionCallback()
{
    static ros::Time prev_time=ros::Time::now();

    float dataUWB[NumStation];
    float scale = 1/1.001;
    float error = 0.38;
    
    dataUWB[0] = scale*uwbData.pose.orientation.x - error - distanceBias(0);
    dataUWB[1] = scale*uwbData.pose.orientation.y - error - distanceBias(1);
    dataUWB[2] = scale*uwbData.pose.orientation.z - error - distanceBias(2);
    dataUWB[3] = scale*uwbData.pose.orientation.w - error - distanceBias(3);
    dataUWB[4] = 0;
    dataUWB[5] = uwbData.pose.position.z;

    // judge the available base station 
    int enableIndex = (int)(dataUWB[5]);
    int enableNum[NumStation] = {0,0,0,0,0,0};
   
    for(int i = 0 ; i < (NumStation-1) ; i++)
    {
        if( ( (enableIndex >> i) % 2 ) == 1 )
        {
            enableNum[ enableNum[NumStation-1] ] = i;
            enableNum[NumStation-1] ++;
        }
    }

    static bool InitFlagUWB = false;
    static bool getBaseStation = false;
    bool goodDataUWB = false;

    // get base station position
    if (!getBaseStation)
    {
        getBaseStation = gotBaseStationPosition();
    }

    // check the UWB data good or bad
    if( ( checkDataUWB(dataUWB,enableNum) ) && getBaseStation )
    {
        goodDataUWB = true;
    }

    // get UWB location result
    if ( goodDataUWB )
    {
        // location by different situation
        switch (enableNum[NumStation-1])
        {
            case 4:
                locationQuadUWB(dataUWB,enableNum);
                break;
            case 3:
                locationTripleUWB(dataUWB,enableNum);
                break;
            case 2:
                locationDualUWB(dataUWB,enableNum);
                break;
            default:
                ROS_INFO("get the UWB data ,but no location.");
                break;
        }

        // filter the UWB Position
        FilteredPositionUWB = locationFilterUWB();
    }
    else
    {
        //ROS_INFO("get the UWB data ,but no location.");
    }

    // initial the UWB position ,distance bias and DR position
    if( gotPositionUWB && ( !InitFlagUWB ) )
    {    
        ROS_INFO("UWB position initialization complete:X=%f, Y=%f.",PositionUWB(0),PositionUWB(1));

        InitFlagUWB=true;
    }

    // correct the UWB data by DR position
    if( sensorInitialed && gotPositionUWB)
    {
        correctDataUWB(dataUWB,enableNum);

        PositionUWB(0) = PositionUWB(0) - 0.22*cos( PositionDR(2) );  // 0.22(m) is bias from center 
        PositionUWB(1) = PositionUWB(1) - 0.22*sin( PositionDR(2) );
    }

    uwbPositionPubish();
}

void locationDR()
{

    double currentYaw = eulerData.point.z * PI/180;
    double currentMileage = odomData.pose.position.x;

    static float yaw_prev = currentYaw; 
    static float mileage_prev = currentMileage; 


    float yaw_delta;   // delta yaw measured by imu
    float mileage_delta;  // current mileage measured by odom
    
    mileage_delta = currentMileage-mileage_prev;
    yaw_delta = calculateDeltaAngle(currentYaw, yaw_prev);

    /***********************************************  evaluate PositionDR by IMU and mileage *************************************************/ 
    Eigen::Vector3d PositionDR_delta(mileage_delta*cos(PositionDR(2)+yaw_delta/2),mileage_delta*sin(PositionDR(2)+yaw_delta/2),yaw_delta);
    PositionDR = PositionDR + PositionDR_delta;
    /***********************************************************************************************************************************/ 
    PositionDR(2) = constrainAngleRad(PositionDR(2));

    //update the prev data
    yaw_prev = currentYaw; 
    mileage_prev = currentMileage;

}

void initKalmanFilter()
{

    P<< 1, 0,
        0, 1;

    Q<< 0.01*0.01,           0,
                  0, 0.01*0.01;

    R<< 0.05*0.05,         0,
                0, 0.05*0.05;

    A<< 1, 0,
        0, 1;

    H<< 1, 0,
        0, 1;
    
    K<< 1, 0,
        0, 1;

    InitFlagKF=true;
}

Eigen::Vector2d KalmanFilter()
{
    Eigen::Vector2d X_kf(0,0); // final evaluation of Kalman Filter 
    Eigen::Vector2d X_k(0,0);  // prediction of Kalman Filter

    Eigen::Vector2d Y_m(FilteredPositionUWB(0),FilteredPositionUWB(1)); // observed value of system states

    // prediction 
    X_k(0) = PositionDR(0);
    X_k(1) = PositionDR(1);

    P=A*P*A.transpose()+Q;

    // evaluation
    K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
    X_kf = X_k +  K*(Y_m - H*X_k);
    P=P-P*K*H;

    return X_kf;
}

void ekfPositionPubish()
{

    geometry_msgs::PointStamped PositionPub;

    static ros::Time current_time;
    current_time=ros::Time::now();

    PositionPub.header.stamp = current_time;
    PositionPub.header.frame_id = "PositionEKF";
    PositionPub.point.x=PositionKF(0);
    PositionPub.point.y=PositionKF(1);
    PositionPub.point.z=PositionDR(2);

    ekf_pub.publish(PositionPub);

}

void ekfPositionCallback()
{

    //the navigation framework in kf: x to the right, y to the heading, z upwards
    
  /*************************************************************************************************/
    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(10);

    // line 0
    fout_point << "  0  " << "    " << PositionKF(0) << "   " << PositionKF(1) << "   ";
    fout_point << distanceBias(0) << "   "  << distanceBias(1) << "   " << distanceBias(2) << "   ";
    fout_point << "  0  " << "   " << "  0  " << "   " << "  0  " <<"\n";
   /*************************************************************************************************/

    static ros::Time prev_time=ros::Time::now();
    prev_time=ros::Time::now();
    double yawCompensationUWB;

    static bool initialFlag = false;

    enableYawUWB=false;
    gotPositionUWB=false;

    // calculate current point by Dead Recking
    locationDR();
    yawCompensationUWB = compensationYaw();

    // if UWB data is refreshed, process the location program; if not, location flag is false.
    if (flag_uwbcb)
    {
        // calculate current point by UWB
        uwbPositionCallback();

        if(gotPositionUWB)
        {
            initialFlag = true;
        }
    }

    // check Kalman filter initialization flag
    if (!InitFlagKF)
    {
        initKalmanFilter();
    }

    static int disableNum = 0;
    static bool infoPub=true;
    // only used DR for location
    if(fabs(odomData.pose.position.y) < 2*enableVelUWB)
    {
        if (infoPub)
        {       
            ROS_INFO("low Velocity, disable the UWB position No. %d times.",++disableNum);
            infoPub=false;
        }
        gotPositionUWB = false;
    }
    else
    {
        infoPub=true;
    }

    // if precision of UWB location is to low, only used DR for location
    if( fabs(PrecisionUWB) > biasThreshold )
    {
        //ROS_INFO("UWB measurement deviates from DR.");
        gotPositionUWB = false;
    }

    if (gotPositionUWB)
    {            
       // if UWB can be used, get current position by Kalman filter 
       PositionKF = KalmanFilter();
    }
    else
    {
       // if UWB can not be used, use DR position as Kalman filter position
       PositionKF(0) = PositionDR(0);
       PositionKF(1) = PositionDR(1);
    }

    // calculate delta time
    double dt=ros::Time::now().toSec()-prev_time.toSec();
    //ROS_INFO("positioncallback time duration is %f.",dt);

    /*****************************************************  record in fusion log *******************************************************/ 
    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(10);

    // line 1
    fout_point << PositionUWB(0) << "    " << PositionUWB(1) << "   " << PositionUWB(2) << "   ";
    fout_point << PositionDR(0) << "   "  << PositionDR(1) << "   " << PositionDR(2) << "   ";
    fout_point << PositionKF(0) << "   " << PositionKF(1) << "   " << "  0  " <<"\n";

    // line 2
    fout_point << P(0,0) << "   " << P(0,1) << "   " << P(1,0) << "   ";
    fout_point << P(1,1) << "   " << K(0,0) << "   " << K(0,1) << "   "; 
    fout_point << K(1,0) << "    " << K(1,1)  << "    " <<  "  0  "  <<"\n";

    // line 3
    fout_point << uwbData.pose.position.x << "   " << uwbData.pose.position.y << "   " << uwbData.pose.position.z << "   ";
    fout_point << uwbData.pose.orientation.x << "   " << uwbData.pose.orientation.y << "   " << uwbData.pose.orientation.z << "   "; 
    fout_point << PrecisionUWB << "    " << gotPositionUWB  << "    " <<  "  0  "  <<  std::endl;
    
    // line 4
    fout_point << -imuData.pose.position.x << "   " << imuData.pose.position.y << "   " << -imuData.pose.position.z << "   ";
    fout_point << imuData.pose.orientation.x << "   " << imuData.pose.orientation.y << "   " << prev_time << "   "; 
    fout_point << enableYawUWB << "    " << orientationUWB  << "    " <<  yawCompensationUWB  <<  std::endl;

    // line 5
    fout_point << odomData.pose.position.x << "   "  << odomData.pose.position.y << "   " << odomData.pose.position.z << "   ";
    fout_point << odomData.pose.orientation.x << "   " << odomData.pose.orientation.y << "   " << odomData.pose.orientation.z << "   ";
    fout_point <<  FilteredPositionUWB(0)  << "    " <<  FilteredPositionUWB(1)  << "    " <<  FilteredPositionUWB(2)  <<  std::endl;

    // line 6
    fout_point << GeometricMatix(0,0) << "   " << GeometricMatix(0,1) << "   " << GeometricMatix(0,2) << "   ";
    fout_point << GeometricMatix(1,0) << "   " << GeometricMatix(1,1) << "   " << GeometricMatix(1,2) << "   ";
    fout_point << GeometricMatix(2,0) << "   " << GeometricMatix(2,1) << "   " << GeometricMatix(2,2) <<  std::endl;

    // line 7
    fout_point << PrecisionIndex(0,0) << "   " << PrecisionIndex(0,1) << "   " << PrecisionIndex(0,2) << "   ";
    fout_point << PrecisionIndex(1,0) << "   " << PrecisionIndex(1,1) << "   " << PrecisionIndex(1,2) << "   ";
    fout_point << PrecisionIndex(2,0) << "   " << PrecisionIndex(2,1) << "   " << PrecisionIndex(2,2) <<  std::endl;

    fout_point<< "\n";
    /***********************************************************************************************************************************/

    // feedback to Dead Recking
    PositionDR(0) = PositionKF(0);
    PositionDR(1) = PositionKF(1);

    if(initialFlag)
    {
        ekfPositionPubish();
    }

    flag_uwbcb = false;
}

void poseCallback(geometry_msgs::PoseArray msg_alf)
{
    ros::Time currentTime=ros::Time::now();
    imuData.header.stamp = currentTime;

    imuData.pose.position.x = msg_alf.poses[1].position.x;
    imuData.pose.position.y = msg_alf.poses[1].position.y;
    imuData.pose.position.z = msg_alf.poses[1].position.z;

    imuData.pose.orientation.x = msg_alf.poses[0].orientation.x;
    imuData.pose.orientation.y = msg_alf.poses[0].orientation.y;
    imuData.pose.orientation.z = msg_alf.poses[0].orientation.z;
    imuData.pose.orientation.w = msg_alf.poses[0].orientation.w;
    flag_imucb=true;

    eulerData.point.x = msg_alf.poses[0].position.x;
    eulerData.point.y = msg_alf.poses[0].position.y;
    eulerData.point.z = (msg_alf.poses[1].orientation.y - msg_alf.poses[1].orientation.x)*180/PI/0.32;

    odomData.pose.position.x = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y);
    odomData.pose.position.y = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w);
    odomData.pose.position.z = 180/PI*(msg_alf.poses[1].orientation.y - msg_alf.poses[1].orientation.x)/0.32;

    odomData.pose.orientation.x = msg_alf.poses[1].orientation.x;
    odomData.pose.orientation.y = msg_alf.poses[1].orientation.y;
    odomData.pose.orientation.z = msg_alf.poses[1].orientation.z;
    odomData.pose.orientation.w = msg_alf.poses[1].orientation.w;
    flag_odomcb=true;
}

void uwbCallback(geometry_msgs::PoseStamped msg_uwb)
{
    ros::Time currentTime=ros::Time::now();
    uwbData.header.stamp = currentTime;

    uwbData.pose.position.x = msg_uwb.pose.position.x;
    uwbData.pose.position.y = msg_uwb.pose.position.y;
    uwbData.pose.position.z = msg_uwb.pose.position.z;

    uwbData.pose.orientation.x = msg_uwb.pose.orientation.x;
    uwbData.pose.orientation.y = msg_uwb.pose.orientation.y;
    uwbData.pose.orientation.z = msg_uwb.pose.orientation.z;
    uwbData.pose.orientation.w = msg_uwb.pose.orientation.w;
    flag_uwbcb=true;
}

void initialCallback(geometry_msgs::PoseStamped msg_initial)
{   
    PositionDR(0) =  msg_initial.pose.position.x;
    PositionDR(1) =  msg_initial.pose.position.y;
    PositionDR(2) =  msg_initial.pose.orientation.z;

    sensorInitialed = true;

    ROS_INFO(" location get the initial data: x = %f , y = %f , theta = %f.",PositionDR(0),PositionDR(1),PositionDR(2)*180/PI);
}

void synchronizeMessage()
{

    double t1,t2,t3,dt;
    bool syncFlag=false;
    t1=(imuData.header.stamp).toSec();
    t2=(uwbData.header.stamp).toSec();
    t3=(odomData.header.stamp).toSec();
    dt=(t1-t2)*(t1-t3)*(t2-t3);

    syncFlag=flag_imucb && flag_uwbcb && flag_odomcb;

    if (syncFlag)
    {
        ekfPositionCallback();
    }
}

void ros_get_params(ros::NodeHandle& nl)
{

    if(nl.getParam("/location_uwb/BaseStationA_x",BaseStationPoint[0].x))
    {
        ROS_INFO("Got BaseStationA_x:%f!",BaseStationPoint[0].x);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationA_x'.");
        BaseStationPoint[0].x = 0;
    }

    if(nl.getParam("/location_uwb/BaseStationA_y",BaseStationPoint[0].y))
    {
        ROS_INFO("Got BaseStationA_y:%f!",BaseStationPoint[0].y);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationA_y'.");
        BaseStationPoint[0].y = 0;
    }

    if(nl.getParam("/location_uwb/High_BS1",BaseStationPoint[0].z))
    {
        ROS_INFO("Got High_BS1:%f!",BaseStationPoint[0].z);
    }
    else
    {
        ROS_ERROR("Failed to get param 'High_BS1'.");
        BaseStationPoint[0].z = 1.83;
    }

    /*****************************************************************/

    if(nl.getParam("/location_uwb/BaseStationB_x",BaseStationPoint[1].x))
    {
        ROS_INFO("Got BaseStationB_x:%f!",BaseStationPoint[1].x);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationB_x'.");
        BaseStationPoint[1].x = 0;
    }

    if(nl.getParam("/location_uwb/BaseStationB_y",BaseStationPoint[1].y))
    {
        ROS_INFO("Got BaseStationB_y:%f!",BaseStationPoint[1].y);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationB_y'.");
        BaseStationPoint[1].y = 10.51;
    }

        if(nl.getParam("/location_uwb/High_BS2",BaseStationPoint[1].z))
    {
        ROS_INFO("Got High_BS2:%f!",BaseStationPoint[1].z);
    }
    else
    {
        ROS_ERROR("Failed to get param 'High_BS2'.");
        BaseStationPoint[1].z = 1.83;
    }

    /*****************************************************************/

    if(nl.getParam("/location_uwb/BaseStationC_x",BaseStationPoint[2].x))
    {
        ROS_INFO("Got BaseStationC_x:%f!",BaseStationPoint[2].x);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationC_x'.");
        BaseStationPoint[2].x = 10.60;
    }

    if(nl.getParam("/location_uwb/BaseStationC_y",BaseStationPoint[2].y))
    {
        ROS_INFO("Got BaseStationC_y:%f!",BaseStationPoint[2].y);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationC_y'.");
        BaseStationPoint[2].y = 0;
    }

    if(nl.getParam("/location_uwb/High_BS3",BaseStationPoint[2].z))
    {
        ROS_INFO("Got High_BS3:%f!",BaseStationPoint[2].z);
    }
    else
    {
        ROS_ERROR("Failed to get param 'High_BS3'.");
        BaseStationPoint[2].z = 1.83;
    }

    /*****************************************************************/

    if(nl.getParam("/location_uwb/BaseStationD_x",BaseStationPoint[3].x))
    {
        ROS_INFO("Got BaseStationD_x:%f!",BaseStationPoint[3].x);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationD_x'.");
        BaseStationPoint[3].x = 10.60;
    }

    if(nl.getParam("/location_uwb/BaseStationD_y",BaseStationPoint[3].y))
    {
        ROS_INFO("Got BaseStationD_y:%f!",BaseStationPoint[3].y);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationD_y'.");
        BaseStationPoint[3].y = 0;
    }

    if(nl.getParam("/location_uwb/High_BS4",BaseStationPoint[3].z))
    {
        ROS_INFO("Got High_BS4:%f!",BaseStationPoint[3].z);
    }
    else
    {
        ROS_ERROR("Failed to get param 'High_BS4'.");
        BaseStationPoint[3].z = 1.83;
    }

    /*****************************************************************/

    if(nl.getParam("/location_uwb/BaseStationAB",lengthBaseStation[0]))
    {
        ROS_INFO("Got BaseStationAB:%f!",lengthBaseStation[0]);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationAB'.");
        lengthBaseStation[0] = 10.66;
    }

    if(nl.getParam("/location_uwb/BaseStationBC",lengthBaseStation[1]))
    {
        ROS_INFO("Got BaseStationBC:%f!",lengthBaseStation[1]);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationBC'.");
        lengthBaseStation[1] = 15.09;
    }

    if(nl.getParam("/location_uwb/BaseStationCA",lengthBaseStation[2]))
    {
        ROS_INFO("Got BaseStationCA:%f!",lengthBaseStation[2]);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationCA'.");
        lengthBaseStation[2] = 10.54;
    }

    if(nl.getParam("/location_uwb/BaseStationAD",lengthBaseStation[3]))
    {
        ROS_INFO("Got BaseStationAD:%f!",lengthBaseStation[3]);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationAD'.");
        lengthBaseStation[3] = 10.66;
    }

    if(nl.getParam("/location_uwb/BaseStationBD",lengthBaseStation[4]))
    {
        ROS_INFO("Got BaseStationBD:%f!",lengthBaseStation[4]);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationBD'.");
        lengthBaseStation[4] = 15.09;
    }

    if(nl.getParam("/location_uwb/BaseStationCD",lengthBaseStation[5]))
    {
        ROS_INFO("Got BaseStationCD:%f!",lengthBaseStation[5]);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationCD'.");
        lengthBaseStation[5] = 10.54;
    }
    /*****************************************************************/

    if(nl.getParam("/location_uwb/BaseStationType",BaseStationType))
    {
        ROS_INFO("Got BaseStationType:%d!",BaseStationType);
    }
    else
    {
        ROS_ERROR("Failed to get param 'BaseStationType'.");
        BaseStationType = 0;
    }

    if(nl.getParam("/location_uwb/DeltaHigh",DeltaHigh))
    {
        ROS_INFO("Got DeltaHigh:%f!",DeltaHigh);
    }
    else
    {
        ROS_ERROR("Failed to get param 'DeltaHigh'.");
        DeltaHigh = 1.26;
    }


    if(nl.getParam("/location_uwb/High_Tag",TagHigh))
    {
        ROS_INFO("Got High_Tag:%f!",TagHigh);
    }
    else
    {
        ROS_ERROR("Failed to get param 'High_Tag'.");
        TagHigh = 0.57;
    }

    /*****************************************************************/

    if(nl.getParam("/location_uwb/PrecisionThresholdUWB",PrecisionThresholdUWB))
    {
        ROS_INFO("Got PrecisionThresholdUWB:%f!",PrecisionThresholdUWB);
    }
    else
    {
        ROS_ERROR("Failed to get param 'PrecisionThresholdUWB'.");
        PrecisionThresholdUWB = 0.12;
    }

    if(nl.getParam("/location_uwb/biasFilterIndex",biasFilterIndex))
    {
        ROS_INFO("Got biasFilterIndex:%f!",biasFilterIndex);
    }
    else
    {
        ROS_ERROR("Failed to get param 'biasFilterIndex'.");
        biasFilterIndex = 0.1;
    }

    if(nl.getParam("/location_uwb/enableVelUWB",enableVelUWB))
    {
        ROS_INFO("Got enableVelUWB:%f!",enableVelUWB);
    }
    else
    {
        ROS_ERROR("Failed to get param 'enableVelUWB'.");
        enableVelUWB = 0.2;
    }
    
    if(nl.getParam("/location_uwb/biasThreshold",biasThreshold))
    {
        ROS_INFO("Got biasThreshold:%f!",biasThreshold);
    }
    else
    {
        ROS_ERROR("Failed to get param 'biasThreshold'.");
        biasThreshold = 0.1;
    }

    if(nl.getParam("/location_uwb/alphaParametre",alphaParametre))
    {
        ROS_INFO("Got alphaParametre:%f!",alphaParametre);
    }
    else
    {
        ROS_ERROR("Failed to get param 'alphaParametre'.");
        alphaParametre = 0.05;
    }


    if(nl.getParam("/location_uwb/yawOffsetPeriod",yawOffsetPeriod))
    {
        ROS_INFO("Got yawOffsetPeriod:%d!",yawOffsetPeriod);
    }
    else
    {
        ROS_ERROR("Failed to get param 'yawOffsetPeriod'.");
        yawOffsetPeriod = 30;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "location");
    ROS_INFO_STREAM("location program is starting...");

    ros::NodeHandle nl;

/********* get base stations position parametre from baseStationPosition.yaml ********/
    system("pwd");
    ros_get_params(nl);
/********       end of getting base stations position parametre      *********/
    ekf_pub = nl.advertise<geometry_msgs::PointStamped>("/ekf_pose", 10);
    uwb_pub = nl.advertise<geometry_msgs::PointStamped>("/uwb_pose", 10);

    ros::Subscriber pose_sub  = nl.subscribe("/alf001_dis", 1,poseCallback);
    ros::Subscriber uwb_sub  = nl.subscribe("/myuwb", 1,uwbCallback);
    ros::Subscriber initial_sub  = nl.subscribe("/init_sensor", 1,initialCallback);

    //message_filters::Subscriber<geometry_msgs::PoseArray> alf_sub(nl,"alf001_dis",1);
    //message_filters::Subscriber<geometry_msgs::PoseStamped> odom_sub(nl,"myodom",1);
    //message_filters::Synchronizer<sync_policy_classifiction> sync(sync_policy_classifiction(10), alf_sub, odom_sub);
    //sync.registerCallback(boost::bind(&positionCallback, _1, _2));

    ros::Time prev_time;
    double dt;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        prev_time=ros::Time::now();

        if(sensorInitialed)
        {
            synchronizeMessage();   
        }
        else
        {
            uwbPositionCallback();
        }

        ros::spinOnce();

        dt=ros::Time::now().toSec()-prev_time.toSec();
        //ROS_INFO("main time duration is %f.",dt);

        loop_rate.sleep();
    }

    fout_point.close();
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















