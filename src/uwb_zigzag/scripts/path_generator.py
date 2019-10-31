#!/usr/bin/env python


import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

arrivalFlag=False

def arrival_ck(data):
    global arrivalFlag
    rospy.loginfo(rospy.get_caller_id() + "  I heard %s", data.data)
    if data.data:
        arrivalFlag=True
    
if __name__ == '__main__':
    #global arrivalFlag
    i=0
    try:
        arrivalFlag=False
        #thepath=[(0,20,0),(0.5,20,0),(0.5,0,0),(1.0,0,0),(1.0,20,0),(1.5,20,0),(1.5,0,0),(2.0,0.5,0),(2.0,20,0),(2.5,20,0),(2.5,0.5,0),(3.0,0.5,0),(3.0,20,0),(3.5,20,0),(3.5,0.5,0)]
        #thepath=[(0,5,0),(0.5,5,0),(0.5,0,0),(0,0,0),(0,5,0),(0.5,5,0),(0.5,0,0),(0,0,0),(0,5,0),(0.5,5,0),(0.5,0,0),(0,0,0),(0,5,0),(0.5,5,0),(0.5,0,0),(0,0,0)]
        #thepath=[(0,8,0),(0.2,8,0),(0.2,0,0),(0.4,0,0),(0.4,8,0),(0.6,8,0),(0.6,0,0),(0.8,0,0),(0.8,8,0),(1.0,8,0),(1.0,0,0),(1.2,0,0),(1.2,8,0),(1.4,8,0),(1.4,0,0),(1.8,0,0),(1.8,8,0),(2.0,8,0),(2.0,0,0),(2.2,0,0),(2.2,8,0),(2.4,8,0),(2.4,0,0),(2.8,0,0),(2.8,8,0),(3.0,8,0),(3.0,0,0),(0,0,0)]
        #thepath=[(0,8,0),(0.5,8,0),(0.5,0,0),(1,0,0),(1,8,0),(1.5,8,0),(1.5,0,0),(2,0,0),(2,8,0),(2.5,8,0),(2.5,0,0),(3,0,0),(3,8,0),(3.5,8,0),(3.5,0,0),(0,0,0)]
        thepath=[(3,5.2,0),(-2.2,8.2,0),(0,0,0),(0,0,0)]
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/arrive_point", Bool, arrival_ck)
        #rospy.Subscriber("/arrive_flag", Bool, arrival_ck)
        pub = rospy.Publisher('/destination', Point, queue_size=10,latch=True)
        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if arrivalFlag:
                thetarget=Point()        
                if i<len(thepath):
                    thetarget.x=thepath[i][0]
                    thetarget.y=thepath[i][1]
                    thetarget.z=thepath[i][2]
                    rospy.loginfo(thetarget)
                    pub.publish(thetarget)
                    arrivalFlag=False
                    i=i+1
                    rate.sleep()
       
            
    except rospy.ROSInterruptException:
        pass
