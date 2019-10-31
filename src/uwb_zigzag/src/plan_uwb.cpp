//ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"

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

//map
#include <path.h>
#include <calib.h>
#include <contour.h>
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

#define PI 3.1415926

//get the current date
std::string getDate();
//log file
std::string log_path = "/home/raspberry002/catkin_ws/src/uwb_zigzag/data/";
std::string file_edge = log_path + "map_edge.txt";
std::ofstream fout_edge(file_edge.c_str());
//ros publisher
ros::Publisher path_pub;
ros::Publisher dst_pub_;
ros::Publisher calib_pub_;
ros::Publisher marker_dst_pub_;
//ros_msg
geometry_msgs::Point robot_pose;
visualization_msgs::Marker marker_dst;  //robot visualization
visualization_msgs::Marker markercorner;  //robot visualization
//yat_data
YAT_PATH mappath;  //path
YAT_POINTF worldStart;
_YAT_POINTF_THETA base_pose; 
_YAT_POINTF_THETA calib_point;
_YAT_POINTF_THETA origin_pose;
std::vector<YAT_POINT> calibPath;  //map calibration points
//c_data
float robot_theta;
int pathPointNum = 1;   //number of finished path points 
bool pathDone = false;
//create map
std::string map_file_ = "/home/raspberry002/catkin_ws/src/uwb_zigzag/data/boundary/map.txt";
POLYGON smoothpolygon;
Map map_ = createGridMap(map_file_.c_str(), smoothpolygon);

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    return q;
}

/*void pub_calibration(_YAT_POINTF_THETA pos_theta)
{
    geometry_msgs::Point point;
    point.x = pos_theta.x;
    point.y = pos_theta.y;
    point.z = pos_theta.theta;

    ROS_INFO("adjusting calibration_pose to %f, %f, %f", point.x, point.y, point.z);
    dst_pub_.publish(point);
}

void pub_next_point(YAT_POINTF nextpoint)
{
    geometry_msgs::Point point;
    point.x = nextpoint.x;
    point.y = nextpoint.y;
    point.z = 0;

    ROS_INFO("next point x:%f , y:%f", point.x, point.y);
    dst_pub_.publish(point);
}

YAT_POINTF last_point;
YAT_POINT last_point_map;
bool calibed = false;

void pointCallback(std_msgs::Bool msg)
{  
    //mission check
    if (cornerPath.size() > 0)
    {    
        ROS_INFO("arrived point.");        
        if(pathPointNum >= cornerPath.size()) 
        {
            pathDone = true;
            ROS_INFO("MAP IS COVERED");
            return;
        }

        YAT_POINT point = cornerPath[pathPointNum];
        YAT_POINTF nextpoint = map_to_world(&map_, point);
        pub_next_point(nextpoint);
        //update last_point info
        last_point = nextpoint;
        last_point_map = cornerPath[pathPointNum];
        pathPointNum++;

        ROS_INFO_STREAM("callback first point" << nextpoint.x << " " << nextpoint.y);
    }
    else
        ROS_INFO("Path no points.");
}*/

void poseCallback(geometry_msgs::PointStamped pose)
{
    origin_pose.x = pose.point.x;
    origin_pose.y = pose.point.y;
}

//show map in rviz
nav_msgs::OccupancyGrid get_occ_map(Map &map)
{
    ROS_INFO("Creating map rviz");
    nav_msgs::OccupancyGrid ocmap;
    ocmap.header.frame_id = "my_frame";
    ocmap.header.stamp = ros::Time::now();

    ocmap.info.height = map.height;
    ocmap.info.width = map.width;
    ocmap.info.resolution = 1.0 / map.scale;
    ocmap.info.origin.position.x = -map.xoffset;
    ocmap.info.origin.position.y = -map.yoffset;

    ocmap.data.resize(ocmap.info.width * ocmap.info.height);

    for(int i=0; i<map.height; i++)
    {
        for(int j=0; j<map.width; j++)
        {
            ocmap.data[i * ocmap.info.width + j] = map.data[i * ocmap.info.width + j];
        }
    }
    return ocmap;
}

void showBoundary()
{
    nav_msgs::Path path;    //robot path
    ROS_INFO("Showing boundary!");
    //show robot path
    geometry_msgs::PoseStamped this_pose_stamped;
    path.header.frame_id = "my_frame";
    path.header.stamp = ros::Time::now();

    for(int i = 0; i < smoothpolygon.num; ++i)
    {
        this_pose_stamped.pose.position.x = smoothpolygon.points[i].x;
        this_pose_stamped.pose.position.y = smoothpolygon.points[i].y;

        // this_pose_stamped.pose.orientation.x = quat.x();
        // this_pose_stamped.pose.orientation.y = quat.y();
        // this_pose_stamped.pose.orientation.z = quat.z();
        // this_pose_stamped.pose.orientation.w = quat.w();
        
        fout_edge << smoothpolygon.points[i].x << "  " << smoothpolygon.points[i].y << std::endl;
        // ROS_INFO("%f ,%f",smoothpolygon.points[i].x,smoothpolygon.points[i].y);
        path.poses.push_back(this_pose_stamped);
    }
    //pub robot map_boundary
    path_pub.publish(path);
    ROS_INFO("Boundary showed!");
}

vector<string> split_str(const string &s, const string &seperator)
{
    vector<string> result;
    typedef string::size_type string_size;
    string_size i = 0;

    while (i != s.size()) {
        int flag = 0;
        while (i != s.size() && flag == 0) {
            flag = 1;
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[i] == seperator[x]) {
                    ++i;
                    flag = 0;
                    break;
                }
        }

        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0) {
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[j] == seperator[x]) {
                    flag = 1;
                    break;
                }
            if (flag == 0)
                ++j;
        }
        if (i != j) {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zigzagMove");
    ROS_INFO_STREAM("zigzag planner is starting...");

    ros::NodeHandle nh_;
    //random seed
    srand((int)time(0));
    //Visual map
    nav_msgs::OccupancyGrid ocmap = get_occ_map(map_);
    ROS_INFO("Map rviz created!");
    //Publisher
    path_pub = nh_.advertise<nav_msgs::Path>("/map_edge", 1, true);
    marker_dst_pub_ = nh_.advertise<visualization_msgs::Marker>("/dest_marker", 1, true);
    ros::Publisher occmap_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    // dst_pub_ = nh_.advertise<geometry_msgs::Point>("/destination", 100, true);
    // calib_pub_ = nh_.advertise<geometry_msgs::Point>("/calib_pose", 100, true);

    //Subcriber
    // ros::Subscriber point_sub = nh_.subscribe("/arrive_point", 10, pointCallback);
    ros::Subscriber robot_pose = nh_.subscribe("/uwb_pose", 10, poseCallback);

    //Get mode param
    int mode = 0;
    if(nh_.getParam("/plan_uwb/mode",mode))
    {
        if(mode == 1)
            ROS_INFO("Launching Map-Mode!");
        else if(mode == 2)
            ROS_INFO("Launching Testing-Mode!");
        else
        {
            ROS_INFO("Bad Mode Input...Launching Map-Mode!");
            mode = 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to get param 'Mode'. Launching Map-Mode");
        mode = 1;
    }

    //pub grid map
    occmap_pub.publish(ocmap);
    showBoundary();

    //get target point
    ros::Rate wait_rate(5);
    while(fabs(origin_pose.x)<0.0001||fabs(origin_pose.y)<0.0001)
    {
        ROS_INFO("Waiting uwb_data......");
        ROS_INFO("uwb_x:%f, uwb_y:%f",origin_pose.x, origin_pose.y); 
        ros::spinOnce();
        wait_rate.sleep();
    }

	worldStart.x = origin_pose.x;
    worldStart.y = origin_pose.y;

    //get path from map
    std::vector<YAT_POINT> cornerPathV;  //all mission points
    std::vector<YAT_POINT> cornerPathT;  //all mission points
    if(mode == 1)
    {    
        mappath = create_path(&map_);
        YAT_POINT mapStart = world_to_map(&map_, worldStart);
        // get_path_all(&map_, &mappath, mapStart, cornerPathV/*alibPath*/);
        get_path(&map_, &mappath, mapStart, cornerPathV);
        getPathV(&map_, &mappath, mapStart, cornerPathT);
    }
    else if(mode == 2)
    {
        std::ifstream fin_corner("/home/raspberry002/catkin_ws/src/uwb_zigzag/data/boundary/test_points.txt", ios::in);
        if(!fin_corner.is_open())
            ROS_INFO("Can not open test_points.txt.");
        // if(!fin_calib.is_open())
        //     ROS_INFO("Can not open test_calib_points.txt.");

        int char_count = 1;
        string line;
        while(!fin_corner.eof() && getline(fin_corner, line))
        {
            vector<string> tmp = split_str(line, " ");  //load whole line

            if (tmp.size() == 2)
            {
                YAT_POINTF corner_tmp;
                corner_tmp.x = atof(tmp[0].c_str());
                corner_tmp.y = atof(tmp[1].c_str());
                cornerPathV.push_back(world_to_map(&map_, corner_tmp));
            }   
        }
        ROS_INFO_STREAM("Test path has " << cornerPathV.size() << " points." );
        //calibPath.push_back(cornerPath[0]);
    }
    
    //init calib_pose
    calib_point.x = worldStart.x;
    calib_point.y = worldStart.y;
    calib_point.theta = -120.0;
    /*Path record*/
    std::string file_pathV = log_path + "pathV.txt";
    std::ofstream fout_pathV(file_pathV.c_str());
    for(int i = 1; i <= cornerPathV.size() - 1; i++)
    {
        _YAT_POINTF temp;
        temp = map_to_world(&map_, cornerPathV[i]);
        fout_pathV << temp.x << "  " << temp.y << std::endl;
    }
    
    std::string file_pathT = log_path + "pathT.txt";
    std::ofstream fout_pathT(file_pathT.c_str());
    for(int i = 1; i <= cornerPathT.size() - 1; i++)
    {
        _YAT_POINTF temp;
        temp = map_to_world(&map_, cornerPathT[i]);
        fout_pathT << temp.x << "  " << temp.y << std::endl;
    }

    ROS_INFO("Vertical path has %lu points", cornerPathV.size());
    ROS_INFO("Transverse path has %lu points", cornerPathT.size());
    ROS_INFO("Planner compeleted!");

    release_path(&mappath);
    releaseMap(map_);
    fout_pathV.close();
    fout_pathT.close();
    fout_edge.close();

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
