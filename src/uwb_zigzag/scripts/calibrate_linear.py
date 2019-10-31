#!/usr/bin/env python


import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

dist_sum = 0
theta_sum = 0


def odom_cb(odom_msg):
    global dist_sum
    global theta_sum
    dist_sum = odom_msg.pose.position.z
    #theta_sum = (odom_msg.pose.orientation.y-odom_msg.pose.orientation.x)/0.4
    theta_sum = odom_msg.pose.orientation.z

if __name__ == '__main__':
    dist_thr = 10
    theta_thr = 3.1416
    linear_v_1 = 0.2  # for straight movement
    angular_v_1 = 0

    linear_v_2 = 0  # for curve movement
    angular_v_2 = 0.2

    dist_sum_pre = 0
    theta_sum_pre = 0
    try:
        rospy.init_node('cali_odom', anonymous=True)
        rospy.Subscriber("/myodom", PoseStamped, odom_cb)
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        vel_robot = Twist()
        while not rospy.is_shutdown():
            dist_sum_pre = dist_sum
            theta_sum_pre = theta_sum
            print "choose your test: 1 --- line; 2 --- turning; 3 --- end \n"
            the_choice = input("your choice: ")
            if the_choice==3: break
            rate = rospy.Rate(10)  # 10hz
            while True:
                if the_choice == 1:
                    if(abs(dist_sum-dist_sum_pre) > dist_thr):
                        vel_robot.linear.x = 0
                        vel_robot.angular.z = 0
                        vel_pub.publish(vel_robot)
                        break
                    else:
                        rospy.loginfo("distance traveled is %f", dist_sum)
                        vel_robot.linear.x = linear_v_1
                        vel_robot.angular.z = angular_v_1
                        vel_pub.publish(vel_robot)
                        rate.sleep()
                elif the_choice == 2:
                    if(abs(theta_sum-theta_sum_pre) > theta_thr):
                        vel_robot.linear.x = 0
                        vel_robot.angular.z = 0
                        vel_pub.publish(vel_robot)
                        break
                    else:
                        rospy.loginfo("angle turned is %f", theta_sum)
                        vel_robot.linear.x = linear_v_2
                        vel_robot.angular.z = angular_v_2
                        vel_pub.publish(vel_robot)
                        rate.sleep()

    except rospy.ROSInterruptException:
        pass
