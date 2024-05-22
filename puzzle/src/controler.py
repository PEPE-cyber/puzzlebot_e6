#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import numpy as np
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Pose2D, Twist

ra = .055
b = 0.1725 / 2

MAX_ANGULAR_SPEED = 0.1
MAX_LINEAR_SPEED = 0.2

class Controler:
    def __init__(self):
        rospy.init_node('Controller', anonymous=False)
        

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers to send update
        self.pose_sub = rospy.Subscriber('/pose', Pose2D, self.setpoint_callback)

        # Subscribers to update setpoint
        self.setpoint_sub = rospy.Subscriber('/setpoint', Pose2D, self.setpoint_callback)
        
        # Subscribers to u
        self.setpoint_sub = rospy.Subscriber('/controllerActive', Bool, self.active_callback)

        self.currentPose = Pose2D()
        self.setpoint = Pose2D()
        self.setpoint.x = 0
        self.setpoint.y = 0
        self.setpoint.theta = 0
        
        self.active = True


        self.rate = rospy.Rate(10)  # 10Hz

    def speeds_2_wheels(self, angular_vel, linear_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        rospy.loginfo(msg)
        self.cmd_vel_pub.publish(msg)


    def setpoint_callback(self, msg):
        self.setpoint = msg

    def pose_callback(self, msg):
        self.currentPose = msg

    def active_callback(self, msg):
        self.active = msg.data

    def stop(self):
        self.speeds_2_wheels(0, 0)
        print("Stopping motors")


    def run(self):
        while not rospy.is_shutdown():
            if self.active:
                # Control
                error_y = self.setpoint.y - self.currentPose.y
                error_x =  self.setpoint.x - self.currentPose.x
                error_distance = sqrt(error_x * error_x + error_y * error_y)
                if (error_distance < 0.01):
                    self.stop()
                else:
                    ang = atan2(error_y, error_x)
                    error_theta = ang - self.currentPose.theta
                    angular_speed = error_theta * 0.13
                    linear_speed = error_distance * 0.15

                    # Make sure it is not too fast
                    if linear_speed > MAX_LINEAR_SPEED:
                        linear_speed = MAX_LINEAR_SPEED
                    elif linear_speed < -MAX_LINEAR_SPEED:
                        linear_speed = -MAX_LINEAR_SPEED
                    if angular_speed > MAX_ANGULAR_SPEED:
                        angular_speed = MAX_ANGULAR_SPEED
                    elif angular_speed < -MAX_ANGULAR_SPEED:
                        angular_speed = -MAX_ANGULAR_SPEED
                    # send speed
                    self.speeds_2_wheels(angular_speed, linear_speed)
            # Sleep
            self.rate.sleep()
            

if __name__ == '__main__':
    try:
        nav = Controler()
        nav.run()
    except rospy.ROSInterruptException:
        nav.stop()
    except KeyboardInterrupt:
        nav.stop()