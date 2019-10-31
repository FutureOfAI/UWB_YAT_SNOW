/*********************************************************************
** this project used to test ROS read IMU data from serial ***********
** copyright leo 2019-10-29 lei.yao@yat.com  *************************
** serial_imu.cpp ****************************************************
*********************************************************************/

#include "ros/ros.h"
//#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include <iostream>
#include <cstdio>
//#include <string>
// define variable
ros::Publisher uwb_pub;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "uwbRawData");
  ros::NodeHandle n;

  uwb_pub = n.advertise<std_msgs::String>("uwbRawData", 1000);
  ros::Rate loop_rate(10);

  // uart port, baudrate, timeout in ms
  std::string port("/dev/uwb");
  unsigned long baud = 115200;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  if(my_serial.isOpen())
  {
    ROS_INFO("serial open sucess!");
  }
  else
  {
    ROS_INFO("serial open failed.");
  }
  ROS_INFO("starting to publish imu topic...\n");

  while(ros::ok())
  {
    if(my_serial.available())
    {
      std_msgs::String result;
      result.data = my_serial.read(my_serial.available());
      int i = 0;
      ROS_INFO("%d %d %d %d %d %d\n", result.data[i+0], result.data[i+1], result.data[i+2], result.data[i+3], result.data[i+4], result.data[i+5]);
      //std::cout<<result.data[0]<<'\n';
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
