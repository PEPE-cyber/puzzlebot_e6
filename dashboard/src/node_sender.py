#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import numpy as np

import zlib
import grpc

# import the generated classes
import data_pb2
import data_pb2_grpc

# open a gRPC channel
channel = grpc.insecure_channel('192.168.0.123:50051')

# create a stub (client)
stub = data_pb2_grpc.MyServiceStub(channel)

import nanocamera as nano
import cv2

class Sender:
    def __init__(self):
        rospy.init_node('SenderToDashboard', anonymous=False)
        
    
        # Subscribers to update real wheel speeds
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)



        self.wl = 0
        self.wr = 0


        self.rate = rospy.Rate(30)  # 10Hz


    
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data

    def run(self):
        camera = nano.Camera(flip=0, width=1280, height=720, fps=30)
        while not rospy.is_shutdown():
            try:
                # discard old frames
                for _ in range(10):
                    frame = camera.read()
                frame = camera.read()
                # create a valid request message
                vls = data_pb2.Vels(
                    v_r=self.wr,
                    v_l=self.wl
                )
                frame = cv2.resize(frame, (400, 400))
                img = data_pb2.Image(
                    data = zlib.compress(frame),
                    width = 400,
                    height = 400
                )

                data = data_pb2.Data(
                    img = img,
                    v = vls
                )
                print("sending data")
                # make the call
                response = stub.sendData(data)
                self.rate.sleep()
                
            except Exception as e:
                print("Error: ")
                print(e)
        
        camera.release()
if __name__ == '__main__':
    try:
        loc = Sender()
        loc.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass