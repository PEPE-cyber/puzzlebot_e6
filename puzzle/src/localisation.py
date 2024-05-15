#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
from tf import transformations
from math import sin, cos

ra = .05
b = 0.191/2

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]


class Localisation:
    def __init__(self):
        rospy.init_node('localisation', anonymous=True)
        
        self.pose_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)


        # Suscribe to the robot's motor nodes
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        
    
        self.rate = rospy.Rate(10)  # 10Hz
        self.wr = 0
        self.wl = 0
        

    def wr_callback(self, msg):
        # Save the message in the variable named wr
        print("update wr", msg.data)
        self.wr = msg.data

    def wl_callback(self, msg):
        # Save the message in the variable named wl
        self.wl = msg.data

    
    def run(self):
        x = 0
        y = 0
        o = 0
        while not rospy.is_shutdown():
            # Update the timestamp of the pose
            dt = 0.1
            
            matA = np.array([[ra/2, ra/2], [ra/(2*b), -ra/(2*b)]])
            self.arr = np.array([self.wr, self.wl])
            Vw = np.matmul(matA, self.arr)
            y_dot =   sin(o) * Vw[0] 
            y +=  y_dot * dt
            x_dot =  1 * (Vw[0] * cos(o))
            x += x_dot * dt
            o += dt * Vw[1]
            quat = transformations.quaternion_from_euler(0,0,o)
            quatNice = Quaternion(quat[0],quat[1],quat[2],quat[3])
            #Creamos el msg
            current_time = rospy.Time.now()
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation = quatNice
            odom.child_frame_id = "base_link"
            odom.twist.twist.angular.z = Vw[1]
            odom.twist.twist.linear.x = x_dot
            odom.twist.twist.linear.y = y_dot
            # Publish the pose
            self.pose_publisher.publish(odom)
            # Sleep to maintain the specified rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_sim = Localisation()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass
