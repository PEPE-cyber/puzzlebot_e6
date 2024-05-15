#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Pose2D, Twist

ra = .05
b = 0.191 / 2

MAX_ANGULAR_SPEED = 0.1
MAX_LINEAR_SPEED = 0.1

class Navigation:
    def __init__(self):
        rospy.init_node('Navigation', anonymous=False)
        
        # Publishers to send wheel speeds
        # self.wl_pub = rospy.Publisher('/cmd_wL', Float32, queue_size=10)
        # self.wr_pub = rospy.Publisher('/cmd_wR', Float32, queue_size=10)  

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers to update real wheel speeds
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)


        # Publisher to send pose
        self.pose_pub = rospy.Publisher('/pose', Pose2D, queue_size=10)

        # Subscribers to update setpoint
        self.setpoint_sub = rospy.Subscriber('/setpoint', Pose2D, self.setpoint_callback)

        self.wl = 0
        self.wr = 0

        self.currentPose = Pose2D()
        self.currentPose.x = 0
        self.currentPose.y = 0
        self.currentPose.theta = 0
        self.setpoint = Pose2D()


        self.rate = rospy.Rate(10)  # 10Hz

    def speeds_2_wheels(self, angular_vel, linear_vel):
        # mat = np.array([[ra/2, ra/2], [ra/(2*b), -ra/(2*b)]])
        # inv_mat = np.linalg.inv(mat)
        # input = np.array([linear_vel, angular_vel])
        # resultado = np.matmul(inv_mat, input)
        # print("Sending speeds: ", resultado)
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)
    
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data

    def setpoint_callback(self, msg):
        self.setpoint = msg

    def run(self):
        dt = 0.1
        while not rospy.is_shutdown():
            # Get the current pose
            s_theta = sin(self.currentPose.theta)
            c_theta = cos(self.currentPose.theta)
            jacobiano = np.array([[ra*c_theta/2, ra*c_theta/2], [ra*s_theta/2, ra*s_theta/2], [ra/(2*b), -ra/(2*b)]])
            [x_dot,y_dot,theta_dot] = np.matmul(jacobiano, np.array([self.wl, self.wr]))
            self.currentPose.x += x_dot * dt
            self.currentPose.y += y_dot * dt
            self.currentPose.theta += theta_dot * dt
           
            self.pose_pub.publish(self.currentPose)
            
            # Control
            error_y = self.setpoint.y - self.currentPose.y
            error_x =  self.setpoint.x - self.currentPose.x
            ang = atan2(error_y, error_x)
            error_distance = sqrt(error_x * error_x + error_y*error_y)
            error_theta = ang - self.currentPose.theta
            angular_speed = error_theta * 0.2
            linear_speed = error_distance * 0.4
            if linear_speed > MAX_LINEAR_SPEED:
                linear_speed = MAX_LINEAR_SPEED
            elif linear_speed < -MAX_LINEAR_SPEED:
                linear_speed = -MAX_LINEAR_SPEED
            if angular_speed > MAX_ANGULAR_SPEED:
                angular_speed = MAX_ANGULAR_SPEED
            self.speeds_2_wheels(angular_speed, linear_speed)
            
            # Sleep
            self.rate.sleep()
            

if __name__ == '__main__':
    try:
        nav = Navigation()
        nav.run()
    except rospy.ROSInterruptException:
        pass