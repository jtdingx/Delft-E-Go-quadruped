#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header





if __name__ == "__main__":
    

    rospy.init_node("Landing_motion_TO")    ###node name

    pub = rospy.Publisher("che",String, queue_size=10)
    pub2 = rospy.Publisher("landing_motion_2rtctrl",JointState, queue_size=10)
    
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    # hello_str.name = ['joint0', 'joint1', 'joint2', 'joint3']

    # hello_str.position.resize(2)
    hello_str.position = [0.1]
    hello_str.velocity = []
    hello_str.effort = []    

    msg = String()

    rate = rospy.Rate(1) ####publish data in 1Hz
    count = 0

    while not rospy.is_shutdown():
        count += 1
        ###publish data
        # msg.data = "hello" + str(count)
        # pub.publish(msg)

        hello_str.position = [count]
        pub2.publish(hello_str)
        rospy.loginfo("pub data:%s",hello_str.position)

        rate.sleep()