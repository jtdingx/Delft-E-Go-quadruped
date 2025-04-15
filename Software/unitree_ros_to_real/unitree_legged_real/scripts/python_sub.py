#! /usr/bin/env python

import rospy
from std_msgs.msg import String ###data type
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

data_receive = String() 

####callback
def receive_data(msg):
    rospy.loginfo("received data:%s",msg.data)
    data_receive = msg.data
    print("data_receive:%s",data_receive)


if __name__ == "__main__":
    

    rospy.init_node("Landing_motion_TOx")    ###node name

    sub = rospy.Subscriber("che",String,receive_data, queue_size=10)
    

    msg = String()

    rate = rospy.Rate(1) ####publish data in 1Hz
    count = 0

    # while not rospy.is_shutdown():
    #     count += 1
    #     pub.publish(msg)
    #     print("data_receive:%s",data_receive.data)
    #     rate.sleep()
    
    rospy.spin()