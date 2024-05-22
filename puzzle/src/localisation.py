#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Pose2D, Twist

ra = .055
b = 0.1725 / 2

MAX_ANGULAR_SPEED = 0.1
MAX_LINEAR_SPEED = 0.2

class Localisation:
    def __init__(self):
        rospy.init_node('Localisation', anonymous=False)
        
    
        # Subscribers to update real wheel speeds
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)


        # Publisher to send pose
        self.pose_pub = rospy.Publisher('/pose', Pose2D, queue_size=10)



        self.wl = 0
        self.wr = 0

        self.currentPose = Pose2D()
        self.currentPose.x = 0
        self.currentPose.y = 0
        self.currentPose.theta = 0

        self.rate = rospy.Rate(10)  # 10Hz


    
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data

    def run(self):
        dt = 0.1
        matriz =np.array([[ra/2, ra/2], [ra/(2*b), -ra/(2*b)]])
        while not rospy.is_shutdown():
            
            # Get the current pose
            [vel_lin, vel_ang]= np.matmul(matriz, np.array([self.wr, self.wl]))
            y_dot =   sin(self.currentPose.theta) * vel_lin
            x_dot =  cos(self.currentPose.theta) * vel_lin
            self.currentPose.x += x_dot * dt
            self.currentPose.y += y_dot * dt
            self.currentPose.theta += vel_ang * dt
            self.currentPose.theta = self.currentPose.theta 
            self.pose_pub.publish(self.currentPose)
            self.rate.sleep()
            

if __name__ == '__main__':
    try:
        loc = Localisation()
        loc.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass