#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Pose2D, Twist
from kalman import EKF

wheelR = .055
l = 0.1725 / 2

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

        self.ekf = EKF()

        self.rate = rospy.Rate(10)  # 10Hz


    
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data

    def run(self):
        dt = 0.1
        matriz =np.array([[wheelR/2, wheelR/2], [wheelR/(2*l), -wheelR/(2*l)]])
        while not rospy.is_shutdown():
            
            # Get the current pose
            u = np.matmul(matriz, np.array([self.wr, self.wl]))
            est_state = self.ekf.stateTrasitionModel(u)
            self.ekf.jacobian(u)

            
            

if __name__ == '__main__':
    try:
        loc = Localisation()
        loc.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass