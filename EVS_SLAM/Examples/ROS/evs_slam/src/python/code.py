#!/usr/bin/env python3
from orb_slam2.msg import Num
from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import MultiArrayDimension
import rospy


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub2 = rospy.Publisher('custom_talker',Num, queue_size=1)
    pub3 = rospy.Publisher("array_talker", Int8MultiArray, queue_size=10)

    rospy.init_node('talker', anonymous=True)


    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        msg_mask=Int8MultiArray
        msg_mask.dim=8
        msg_mask.data=[1,2,1]
        
        pub3.publish(msg_mask)

        
        #msg=Num
        #msg.name="Andres"
        #msg.age= int(5)
        #msg.serialize(msg)
        #rospy.loginfo(msg_mask)
        #pub2.publish(msg)
        
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass