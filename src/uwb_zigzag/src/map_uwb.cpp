//ros
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
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

using namespace std;
#define PI 3.14159265358979323846

//typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, geometry_msgs::PoseStamped> sync_policy_classifiction;
//get the current date //date -s "20180929 08:00:00"


bool flag_file_close=false;
bool flag_log = false;
double l_scale_ = 1;
geometry_msgs::Twist js_twist;

float robot_pose_x, robot_pose_y;
//float robot_angle_;

std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

//file
std::string getTime();
string map_path = "/home/raspberry002/catkin_ws/src/uwb_zigzag/data/boundary/";
string filename_map_save = map_path + getTime() + "map.txt";
string filename_map = map_path + "map.txt";
ofstream fout_map(filename_map.c_str());          //save map

void positionCallback(geometry_msgs::PointStamped msg_uwb)
{
    //get the robot position
    robot_pose_x = msg_uwb.point.x;
    robot_pose_y = msg_uwb.point.y;
    if(flag_log)
    {
        //log the map
        fout_map.setf(std::ios_base::showpoint);
        fout_map.precision(6);
        fout_map <<  robot_pose_x << "    " <<  robot_pose_y  << "  0  0  0 " <<  std::endl;
    }
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
   if(joy->buttons[0])  // Key 'A'
   {
      ROS_INFO("%s","fencing starting...");
      flag_log=true;
   }

   if(joy->buttons[1])  // Key 'B'
   {
      ROS_INFO("%s","fencing finish...");        
      flag_log=false;
      flag_file_close=true;
   }
   if(joy->buttons[6] || joy->buttons[7])
   {
       js_twist.angular.z=0;
       js_twist.linear.x=0;
   }
   

   js_twist.angular.z = 0.35 *  (joy->axes[2]); //angle
   js_twist.linear.x = 1 * (joy->axes[3]) /2.5 ; //

   //arrow
    if(joy->buttons[13])			//left
        js_twist.angular.z = 0.35;
    if(joy->buttons[14])			//right
        js_twist.angular.z = -0.35;
    if(joy->buttons[15])			//up
        js_twist.linear.x = 0.4;
    if(joy->buttons[16])			//down
        js_twist.linear.x = -0.4;
}
void file_cp(string* file_r, string* file_w)
{
    FILE *op,*inp;
    op = fopen(file_r->c_str(),"rb");
    inp = fopen(file_w->c_str(),"wb");

    void *buf;
    char c;

    while(!feof(op))
    {
        fread(&buf,1,1,op);
        fwrite(&buf,1,1,inp);
    }
    fclose(inp);
    fclose(op);
    printf("Map copied done.\n");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mapping");
    ROS_INFO_STREAM("mapping program is starting...");

    ros::NodeHandle nh;

    //ros::Subscriber pose_sub  = nh.subscribe("/alf001_dis", 1,poseCallback);
    ros::Subscriber position_sub  = nh.subscribe("/uwb_pose", 1,positionCallback);


    ros::Subscriber sub_joy = nh.subscribe("joy",100, joy_callback);              //joy control
    ros::Publisher  pub_joy = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        if (flag_file_close)
        {
            fout_map.close();
            flag_file_close=false;
            ROS_INFO("%s","fencing done.");
            ROS_INFO("%s","save a map copy to....");
            file_cp(&filename_map, &filename_map_save);
        }
        pub_joy.publish(js_twist);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



