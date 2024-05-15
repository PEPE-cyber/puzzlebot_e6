#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt

MAX_ANGULAR_SPEED = 0.5
MAX_LINEAR_SPEED = 0.2

class talker():
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.set_x = 10
        self.set_y = 10
        self.x = 0
        self.y = 0

        vel_msg = Twist()
        vel_msg.linear.x =  0
        vel_msg.angular.z = 0
        rospy.loginfo(vel_msg)
        self.pub.publish(vel_msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.th = msg.pose.pose.orientation.z


    def run(self):
        while not rospy.is_shutdown():
            ey = self.set_y - self.y
            ex = self.set_x - self.x
            ang = atan2(ey, ex)
            ed = sqrt(ex * ex + ey*ey)
            eth = ang - self.th
            vel_msg = Twist()
            vel_msg.linear.x =  ed * 0.1
            vel_msg.angular.z = eth * 0.1 # Change this value to what you want
            if vel_msg.linear.x > MAX_LINEAR_SPEED:
                vel_msg.linear.x = MAX_LINEAR_SPEED
            elif vel_msg.linear.x < -MAX_LINEAR_SPEED:
                vel_msg.linear.x = -MAX_LINEAR_SPEED
            if (vel_msg.angular.z > MAX_ANGULAR_SPEED):
                vel_msg.angular.z = MAX_ANGULAR_SPEED
            elif (vel_msg.angular.z < -MAX_ANGULAR_SPEED):
                vel_msg.angular.z = -MAX_ANGULAR_SPEED
            rospy.loginfo(vel_msg)
            self.pub.publish(vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = talker()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass