#!/usr/bin/env python
import os

import rospy
from geometry_msgs.msg import Twist
from time import sleep


def go1():
    rospy.init_node('crazypi_client')
    print "node initialized"

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    
    twist = Twist()
    twist.linear.x = 2.0
    twist.linear.y = 2.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    for i in range(0, 1):
        pub.publish(twist)
        sleep(0.05)

if __name__ == '__main__':
    os.environ["ROS_MASTER_URI"] = "http://192.168.18.16:11311"
    os.environ["ROS_IP"] = "192.168.18.78"

    print "sending message..."
    try:
        go1()
    except rospy.ROSInterruptException:
        pass


