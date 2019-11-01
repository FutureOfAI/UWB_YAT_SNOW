#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#define PI 3.14159265358979323846

geometry_msgs::PointStamped eulerData;
// geometry_msgs::PoseStamped odomData;
geometry_msgs::PoseStamped imuData;
geometry_msgs::PoseStamped uwbData;

void poseCallback(geometry_msgs::PoseArray msg_alf)
{
    // ros::Time currentTime=ros::Time::now();
    // imuData.header.stamp = currentTime;

    imuData.pose.position.x = msg_alf.poses[1].position.x;
    imuData.pose.position.y = msg_alf.poses[1].position.y;
    imuData.pose.position.z = msg_alf.poses[1].position.z;

    imuData.pose.orientation.x = msg_alf.poses[0].orientation.x;
    imuData.pose.orientation.y = msg_alf.poses[0].orientation.y;
    imuData.pose.orientation.z = msg_alf.poses[0].orientation.z;
    imuData.pose.orientation.w = msg_alf.poses[0].orientation.w;
    // flag_imucb=true;

    eulerData.point.x = msg_alf.poses[0].position.x;
    eulerData.point.y = msg_alf.poses[0].position.y;
    eulerData.point.z = (msg_alf.poses[1].orientation.y - msg_alf.poses[1].orientation.x)*180/PI/0.32;

    ROS_INFO("%f %f %f\n", imuData.pose.orientation.x, imuData.pose.orientation.y, imuData.pose.orientation.z);
    
    // odomData.pose.position.x = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y);
    // odomData.pose.position.y = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w);
    // odomData.pose.position.z = 180/PI*(msg_alf.poses[1].orientation.y - msg_alf.poses[1].orientation.x)/0.32;

    // odomData.pose.orientation.x = msg_alf.poses[1].orientation.x;
    // odomData.pose.orientation.y = msg_alf.poses[1].orientation.y;
    // odomData.pose.orientation.z = msg_alf.poses[1].orientation.z;
    // odomData.pose.orientation.w = msg_alf.poses[1].orientation.w;
    // flag_odomcb=true;
}

void uwbCallback(geometry_msgs::PoseStamped msg_uwb)
{
    // ros::Time currentTime=ros::Time::now();
    // uwbData.header.stamp = currentTime;

    uwbData.pose.position.x = msg_uwb.pose.position.x;
    uwbData.pose.position.y = msg_uwb.pose.position.y;
    uwbData.pose.position.z = msg_uwb.pose.position.z;

    uwbData.pose.orientation.x = msg_uwb.pose.orientation.x;
    uwbData.pose.orientation.y = msg_uwb.pose.orientation.y;
    uwbData.pose.orientation.z = msg_uwb.pose.orientation.z;
    uwbData.pose.orientation.w = msg_uwb.pose.orientation.w;
    // flag_uwbcb=true;
    ROS_INFO("%f %f %f %f\n", uwbData.pose.orientation.x, uwbData.pose.orientation.y, uwbData.pose.orientation.z, uwbData.pose.orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Topic_Test");
    ROS_INFO_STREAM("UWB and IMU topic is starting...");

    ros::NodeHandle nl;

    ros::Subscriber pose_sub  = nl.subscribe("/alf001_dis", 1000,poseCallback);
    ros::Subscriber uwb_sub  = nl.subscribe("/myuwb", 1000,uwbCallback);

    ros::Rate loop_rate(10); //frequency

    while(ros::ok())
    {
//	ROS_INFO("%f\n", imuData.pose.orientation.x);
	ros::spinOnce();
    	loop_rate.sleep();
    }
    return 0;
}
