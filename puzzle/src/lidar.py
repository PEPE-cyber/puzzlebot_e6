#!/usr/bin/env python3
import rospy
from rplidar import RPLidar, RPLidarException
from geometry_msgs.msg import Pose2D
from math import sin, cos, pi
import cv2 as cv
import numpy as np
import zlib
import time

import grpc

# import the generated classes
import image_msg_pb2
import image_msg_pb2_grpc

# open a gRPC channel
channel = grpc.insecure_channel('192.168.0.103:50051')

# create a stub (client)
stub = image_msg_pb2_grpc.PointsProcessorStub(channel)

class PointCloud:
    
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
        self.cloudData = np.ones([height, width], dtype=np.uint8) * 255
        self.pose = Pose2D()
        self.lidar = RPLidar('/dev/ttyUSBLiDAR', baudrate=115200)
        rospy.init_node('PointCloud', anonymous=False)
        rospy.Subscriber('/pose', Pose2D, self.pose_callback)
        


    def pose_callback(self, msg):
        self.pose = msg

    def clean(self):
        self.cloudData = self.cloudData * 0 + 255
       

    def addPoint(self, coord, size=1):
        [x, y] = coord
        relativeY = self.center[1] + y
        relativeX = self.center[0] + x
        if size == 1:
            self.cloudData[relativeY][relativeX] = 0
        else:
            for i in range(-size // 2, size // 2 + 1):
                for j in range(-size // 2, size // 2 + 1):
                    # make sure the object is within the map and not lower than 0
                    if abs(relativeY + i) < self.height and abs(relativeX + j) < self.width and relativeY + i >= 0 and relativeX + j >= 0:
                        self.cloudData[relativeY + i][relativeX + j] = 0

    def addLidarMeasure(self, angle, distance):
        absAngle = angle + self.pose.theta - 90
        # make the center of the image as the origin
        x = int(distance * cos(absAngle * pi / 180) + self.pose.x)
        y = int(distance * sin(absAngle * pi / 180) + self.pose.y)
        if abs(x) < self.width and abs(y) < self.height // 2:
            # determine the size of the point based on the distance
            size = int(distance * 0.1) + 2
            self.addPoint([x, y], size=size)

    def sendTofindBorders(self,angles, ranges):
        # Find the lines in the map, use Hough transform
        data = self.cloudData
        data = zlib.compress(data)
        # create a valid request message
        cp = image_msg_pb2.CloudPoints(data=data, width=self.width, height=self.height)
        sc = image_msg_pb2.LidarScan(angles=angles, ranges=ranges)
        p2 = image_msg_pb2.Pose2D(x=self.pose.x, y=self.pose.y, theta=self.pose.theta)
        request = image_msg_pb2.LidarData(points=cp, scan=sc, pose=p2)
        # make the call
        response = stub.getLocation(request)

    def run(self):
        iterator = self.lidar.iter_scans()
        rate = rospy.Rate(10)
        while True:
            try:
                for scan in iterator:
                    start_time = time.time()
                    angles = []
                    distances = []
                
                    for (_, angle, distance) in scan:
                        angles.append(angle)
                        distances.append(distance)
                        self.addLidarMeasure(angle, distance)
                    self.sendTofindBorders(angles, distances)
                    self.clean()
                    print(time.time() - start_time)
                    
            except RPLidarException as e:
                print("error")
                self.lidar.clean_input()
                pass
            except:
                print("hola")
                self.lidar.clean_input()
            rate.sleep()


if __name__ == '__main__':
    node = PointCloud(600,600)
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.lidar.disconnect()