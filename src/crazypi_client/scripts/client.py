#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import sys
import os
import json

from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import QThread, pyqtSignal, QString

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from PIL import Image as ImagePP
from base64 import standard_b64encode, standard_b64decode
from StringIO import StringIO
from math import floor, ceil, sqrt
#from PIL.ImageQt import ImageQt

def decode(string):
    """ b64 decode the string, then PNG-decompress """
    decoded = standard_b64decode(string)
    buff = StringIO(decoded)
    i = ImagePP.open(buff)
    qim = ImageQt(i)
    return qim


# crazypi-client


def center(self):
    frameGm = self.frameGeometry()
    screen = QtGui.QApplication.desktop().screenNumber(QtGui.QApplication.desktop().cursor().pos())
    centerPoint = QtGui.QApplication.desktop().screenGeometry(screen).center()
    frameGm.moveCenter(centerPoint)
    self.move(frameGm.topLeft())



class SpinThread(QThread):
    def __init__(self):
        QThread.__init__(self)

    def __del__(self):
        self.wait()

    def run(self):
        # your logic here
        rospy.spin()

def image_callback(msg):
    image = QImage(msg.data, msg.width, msg.height, QImage.Format_RGB888);
    global mainWindow
    mainWindow.imageUpdateSlot.emit(image)

def map_callback(msg):
    # bytearray = QtCore.QByteArray.fromBase64(decode(json.loads(msg.data)['data']))
    bytearray = QtCore.QByteArray.fromBase64(json.loads(msg.data)['data'])
    # bytearray = decode(json.loads(msg.data)['data'])
    image = QtGui.QImage.fromData(bytearray, 'PNG')
    # image = decode(json.loads(msg.data)['data'])
    mainWindow.mapUpdateSlot.emit(image)

mainWindow = None

class MainWindow(QtGui.QWidget):
    imageUpdateSlot = pyqtSignal(QImage, name = 'imageUpdateSlot')
    mapUpdateSlot = pyqtSignal(QImage, name = 'mapUpdateSlot')

    def __init__(self):
        super(MainWindow, self).__init__()
        layout = QtGui.QVBoxLayout()

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

        self.imageLabel = QtGui.QLabel()
        layout.addWidget(self.imageLabel)

        self.mapLabel = QtGui.QLabel()
        layout.addWidget(self.mapLabel)

        self.setLayout(layout)

        rospy.init_node('crazypi_client')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

        rospy.Subscriber('/camera/image_raw', Image, image_callback)
        rospy.Subscriber('/crazy_map', String, map_callback)

        self.spinThread = SpinThread()
        self.spinThread.start()

        self.imageUpdateSlot.connect(self.imageUpdate)
        self.mapUpdateSlot.connect(self.mapUpdate)

        print "node initialized"

    def imageUpdate(self, qImage):
        pm = QPixmap(qImage)
        self.imageLabel.setPixmap(pm)

    def mapUpdate(self, qImage):
        pm = QPixmap(qImage)
        self.mapLabel.setPixmap(pm)

    def forward(self):
        print('forward')

        twist = self.WLCTwist(2,0,0)
        try:
            for i in range(0, 10):
                self.pub.publish(twist)
                sleep(0.05)
        except rospy.ROSInterruptException:
            pass

    def back(self):
        print('back')
        twist = self.WLCTwist(-2,0,0)
        try:
            for i in range(0, 10):
                self.pub.publish(twist)
                sleep(0.05)
        except rospy.ROSInterruptException:
            pass


    def leftRot(self):
        print('leftRot')
        twist = self.WLCTwist(0,0,2)
        try:
            for i in range(0, 10):
                self.pub.publish(twist)
                sleep(0.05)
        except rospy.ROSInterruptException:
            pass

    def rightRot(self):
        print('rightRot')
        twist = self.WLCTwist(0,0,-2)
        try:
            for i in range(0, 10):
                self.pub.publish(twist)
                sleep(0.05)
        except rospy.ROSInterruptException:
            pass

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
        center(self.mainWindow)
        global mainWindow
        mainWindow = self.mainWindow

        self.close()





if __name__ == '__main__':

    # Create an PyQT4 application object.
    app = QApplication(sys.argv)

    # The QWidget widget is the base class of all user interface objects in PyQt4.
    configWindow = ConfigWindow()

    # Show window
    configWindow.show()
    center(configWindow)

    sys.exit(app.exec_())


