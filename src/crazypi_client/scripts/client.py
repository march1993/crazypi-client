#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import sys
import os

from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import *

import rospy
from geometry_msgs.msg import Twist
from time import sleep

def h1():
    os.environ["ROS_MASTER_URI"] = "http://192.168.99.1:11311"

    print "sending message..."
    try:
        go1()
    except rospy.ROSInterruptException:
        pass




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

    for i in range(0, 10):
        pub.publish(twist)
        sleep(0.05)


class MainWindow(QtGui.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        layout = QtGui.QVBoxLayout()
        layout.addWidget(QtGui.QLabel('SB_GZX'))
        forwardButton = QtGui.QPushButton('Forward')
        layout.addWidget(forwardButton)
        forwardButton.clicked.connect(self.forward)

        backButton = QtGui.QPushButton('Back')
        layout.addWidget(backButton)
        backButton.clicked.connect(self.back)

        leftRotButton = QtGui.QPushButton('LeftRot')
        layout.addWidget(leftRotButton)
        leftRotButton.clicked.connect(self.leftRot)

        rightRotButton = QtGui.QPushButton('RightRot')
        layout.addWidget(rightRotButton)
        rightRotButton.clicked.connect(self.rightRot)

        self.setLayout(layout)

        rospy.init_node('crazypi_client')
        print "node initialized"

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

        print 'ok'

    def forward(self):
        print('forward')

        twist = WLCTwist()
        for i in range(0, 10):
            self.pub.publish(twist)
            sleep(0.05)

    def back(self):
        print('back')
        twist = WLCTwist()
        for i in range(0, 10):
            self.pub.publish(twist)
            sleep(0.05)


    def leftRot(self):
        print('leftRot')
        twist = WLCTwist()
        for i in range(0, 10):
            self.pub.publish(twist)
            sleep(0.05)

    def rightRot(self):
        print('rightRot')
        twist = WLCTwist()
        for i in range(0, 10):
            self.pub.publish(twist)
            sleep(0.05)

    def WLCTwist(self, linearX = 0.0, linearY = 0.0, angularZ = 0.0):
        twist = Twist()
        twist.linear.x = linearX
        twist.linear.y = linearY
        twist.angular.z = angularZ
        return twist



class ConfigWindow(QtGui.QWidget):
    def __init__(self):
        super(ConfigWindow, self).__init__()

        layout = QtGui.QVBoxLayout()

        h0 = QtGui.QHBoxLayout()
        l0 = QtGui.QLabel('Local IP: ')
        l0.setStyleSheet("font-family: Monospace;")
        self.t0 = t0 = QtGui.QLineEdit()
        h0.addWidget(l0)
        h0.addWidget(t0)
        layout.addLayout(h0)

        h1 = QtGui.QHBoxLayout()
        l1 = QtGui.QLabel('Robot IP: ')
        l1.setStyleSheet("font-family: Monospace;")
        self.t1 = t1 = QtGui.QLineEdit()
        h1.addWidget(l1)
        h1.addWidget(t1)
        layout.addLayout(h1)


        button = QtGui.QPushButton('Connect')
        layout.addWidget(button)
        button.clicked.connect(self.connect)

        self.setLayout(layout)


    def connect(self):
        os.environ["ROS_MASTER_URI"] = "http://" + str(self.t1.text()) + ":11311"
        os.environ["ROS_IP"] = str(self.t0.text())
        print os.environ["ROS_MASTER_URI"]
        print os.environ["ROS_IP"]

        self.mainWindow = MainWindow()
        self.mainWindow.show()
        self.close()





if __name__ == '__main__':

    # Create an PyQT4 application object.
    app = QApplication(sys.argv)

    # The QWidget widget is the base class of all user interface objects in PyQt4.
    configWindow = ConfigWindow()

    # Show window
    configWindow.show()

    sys.exit(app.exec_())


