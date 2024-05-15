#!/usr/bin/env python3
import rospy
from rplidar import RPLidar
from geometry_msgs.msg import Pose2D
from math import sin, cos, pi
import cv2 as cv
import numpy as np

class Map:
    
    def __init__(self, width, height):
        assert width > 0 
        assert height > 0
        # make sure the width and height are odd numbers
        if width % 2 == 0:
            width += 1
        if height % 2 == 0:
            height += 1
        self.center = (width // 2, height // 2)
        self.width = width
        self.height = height
        self.goal = None
        self.currentPosition = None
        self.mapData = np.ones([height, width], dtype=np.uint8) * 255
    
    def setCoord(self, coord, value):
        [x, y] = coord
        self.mapData[self.center[1] + y][self.center[0] + x] = value
    
    def getCoord(self, coord):
        [x, y] = coord
        return self.mapData[self.center[1] + y][self.center[0] + x]
    
    def show(self):
        img = cv.cvtColor(np.array(self.mapData), cv.COLOR_GRAY2BGR)
        if self.goal:
            cv.circle(img, self.goal, 2, (0, 0, 255), -1)
        if self.currentPosition:
            cv.circle(img, self.currentPosition, 2, (0, 255, 0), -1)
        cv.imshow('map', img)

    def addObject(self, coord):
        [x, y] = coord
        self.mapData[self.center[1] + y][self.center[0] + x] = 0

    def setGoal(self, coord):
        [x, y] = coord
        self.goal = (self.center[0] + x, self.center[1] + y)

    def setCurrentPosition(self, coord):
        [x, y] = coord
        self.currentPosition = (self.center[0] + x, self.center[1] + y)

class LiDAR():
    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSBLidar', baudrate=115200)
        self.lidar.start_motor()
        self.lidar.connect()
        self.currentPose = Pose2D()
        rospy.init_node('lidar', anonymous=True)
        rospy.Subscriber('/pose', Pose2D, self.pose_callback)
        self.map = Map(300, 300)

    def pose_callback(self, msg):
        x = msg.x * 100
        y = msg.y * 100
        self.currentPose = Pose2D(x=x, y=y,theta= msg.theta)
        self.map.setCurrentPosition((int(round(x)), int(round(y))))

      
    def run(self):
        while not rospy.is_shutdown():
            for scan in self.lidar.iter_measures():
                [new_scan, quality, angle, distance] = scan
                distance = distance / 10 # convert to cm
                relativeX = distance * cos(angle * pi / 180)
                relativeY = distance * sin(angle * pi / 180)
                x = relativeX * cos(self.currentPose.theta) - relativeY * sin(self.currentPose.theta) + self.currentPose.x
                y = relativeX * sin(self.currentPose.theta) + relativeY * cos(self.currentPose.theta) + self.currentPose.y  
                if abs(x) < self.map.width // 2 and abs(y) < self.map.height // 2:
                    x = int(round(x))
                    y = int(round(y))
                    self.map.addObject((x, y))
                    print("added")
                key = cv.waitKey(1) 
                self.map.show()

    

if __name__ == '__main__':
    node = LiDAR()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.lidar.disconnect()