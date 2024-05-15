#!/usr/bin/env python3
import rospy
from rplidar import RPLidar
import cv2
import nanocamera as nano


    

def listener():
    # lidar = RPLidar('/dev/ttyUSB1', baudrate=115200)
    camera = nano.Camera(flip=0, width=640, height=480, fps=30)
    if(camera.isReady()):
        print("Camera is ready")
        frame = camera.read()
        print(frame.shape)
    rospy.init_node('listener', anonymous=True)
    # sleep for 10 seconds to allow the lidar to start up

if __name__ == '__main__':
    listener()