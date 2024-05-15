#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D


pub = rospy.Publisher('/setpoint', Pose2D, queue_size=10)

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    pub.publish(Pose2D(x=msg.data, y=0, theta=0))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("sx", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()