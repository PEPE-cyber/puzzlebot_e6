#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool, String
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Twist, Pose2D

#Manda el sp
#Buscan aruco 
ra = .055
b = 0.1725 / 2

class Master:
    def __init__(self):
        rospy.init_node('Master', anonymous=False)
        
	# Publishers
        self.controllermode_pub = rospy.Publisher('/controller_mode', String, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/setpoint', Pose2D, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/found', Bool, self.found_callback)
        
        self.rate = rospy.Rate(10)  # 10Hz

        self.state = 'Turning'
        self.found = False
        print("Start")

    def found_callback(self, msg):
        self.found = msg.data
        
    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        print("Stopping motors")
        self.cmd_vel_pub.publish(self.vel)


    def run(self):
        while not rospy.is_shutdown():
	        #Creamos el msj String()
            mode = String()
            vel = Twist()
            if self.state == 'Coords':
                mode.data = 'Coords'
                self.controllermode_pub.publish(mode)
                pose = Pose2D()
                pose.x = 2
                pose.y = 1
                pose.theta = 0
                self.setpoint_pub.publish(pose)
            elif self.state == 'Aruco':
                mode.data = 'Aruco'
                self.controllermode_pub.publish(mode)
                if self.found == False:
                    self.state = 'Take arUco'
                # Si no es True me quedo en el mismo estado
                # ya que lo sigo viendo
            elif self.state == 'Turning':
                mode.data = 'Turning'
                self.controllermode_pub.publish(mode)
                if self.found == True:
                    self.state = 'Aruco'
                # Si no es False me quedo en el mismo estado
            elif self.state == 'Take arUco':
                # Apago el ocntrolador
                mode.data = 'Off'
                self.controllermode_pub.publish(mode)
                # Me acerco al arUco
                vel.linear.x = 0.1
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(1)
                # Comienzo a tomar el arUco
                vel.linear.x = 0
                self.cmd_vel_pub.publish(vel)
                #TODO: Comienzo a publicar para que el servo lo agarre
                # Me hago hacia atras
                vel.linear.x = -0.1
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(1)
                # Me detengo
                vel.linear.x = 0
                self.cmd_vel_pub.publish(vel)
                # Me voy a la coordenada
                # self.state = 'Coords'
                self.state = "Coords"
            elif self.state == "Stop":
                vel.linear.x = 0
                vel.angular.z = 0
                print("Stopping motors")
                self.cmd_vel_pub.publish(vel)


            print(self.state)
            # Sleep
            self.rate.sleep()

            

if __name__ == '__main__':
    try:
        mast = Master()
        mast.run()
    except rospy.ROSInterruptException:
        mast.stop()
    except KeyboardInterrupt:
        mast.stop()
