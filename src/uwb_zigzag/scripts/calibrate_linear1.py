#!/usr/bin/env python


import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

dist_sum=0

def odom_cb(odom_msg):
    global dist_sum
    dist_sum=odom_msg.pose.pose.position.z
    
if __name__ == '__main__':
    try:
        rospy.init_node('cali_odom', anonymous=True)
        rospy.Subscriber("/odom", Odometry, odom_cb)
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        vel_robot=Twist()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
	    rospy.loginfo("distance traveled is %f", dist_sum)
            if dist_sum>4.0:
                vel_robot.linear.x= 0;
                vel_robot.angular.z= 0;
                vel_pub.publish(vel_robot)
                break  
	    else:
		vel_robot.linear.x= 0.2;
                vel_robot.angular.z= 0;    
	        vel_pub.publish(vel_robot)
            rate.sleep()
       
            
    except rospy.ROSInterruptException:
        pass
