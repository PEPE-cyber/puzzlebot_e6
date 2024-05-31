#!/usr/bin/env python

#! La flag que cambia el controlador comienza la sequencia de buscar el aruco donde lo vamos a dejar
#! y a dejar el aruco y detenerse
import rospy
import numpy as np
from std_msgs.msg import Bool, String, Float32
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
        self.servo_pub = rospy.Publisher('/ServoAngle', Float32, queue_size=10)
        self.aruco_pub = rospy.Publisher('/idAruco', int, queue_size=10)

        # Subscribers
        rospy.Subscriber('/found', Bool, self.found_callback)
        rospy.Subscriber('/controller_done', Bool, self.phase_callback)
        
        self.rate = rospy.Rate(10)  # 10Hz

        self.state = 'Turning'
        self.found = False
        self.phase = False
        self.controller_mode = String()
        print("Start")

    def phase_callback(self, msg):
        self.phase = msg.data

    def found_callback(self, msg):
        self.found = msg.data
        
    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        print("Stopping motors")
        self.cmd_vel_pub.publish(self.vel)

    def controller_callback(self, msg):
        if self.state == "going_box":
            self.state = "searching_Box_ArUco"
        if self.state == "reach_Box_ArUco":
            self.state = "take_ArUco"
        if self.state == "going_goal":
            self.state = "searching_goal_ArUco"
        if self.state == "reach_goal_ArUco":
            self.state = "Stop"
        

    def sendControlerMode(self, mode):
        if mode != self.controller_mode.data:
            self.controller_mode.data = mode
            self.controllermode_pub.publish(self.controller_mode)

    def run(self):
        while not rospy.is_shutdown():
	        #Creamos el msj String()
            
            vel = Twist()
            if self.state == 'going_box':
                self.sendControlerMode('Coords')
                pose = Pose2D()
                # coordinates of box
                pose.x = 1 
                pose.y = 1
                pose.theta = 0
                self.setpoint_pub.publish(pose)
            elif self.state == 'searching_Box_ArUco': # aqui nomas giro
                self.sendControlerMode('Turning')
                self.aruco = 1 #! Numero ArUco que se va a agarrar
                self.aruco_pub.publish(self.aruco)
                if self.found == True:
                    self.state = 'reach_Box_ArUco'
                # Si no es False me quedo en el mismo estado
            elif self.state == 'reach_Box_ArUco':
                self.sendControlerMode('Aruco')
            elif self.state == 'take_ArUco':
                # Apago el controlador
                self.sendControlerMode('Off')
                # Me acerco al arUco
                vel.linear.x = 0.1
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(1)
                # Comienzo a tomar el arUco
                vel.linear.x = 0
                self.cmd_vel_pub.publish(vel)
                #TODO: Comienzo a publicar para que el servo lo agarre
                self.servo_pub.publish(1) 
                # Me hago hacia atras
                vel.linear.x = -0.1
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(1)
                # Me detengo
                vel.linear.x = 0
                self.cmd_vel_pub.publish(vel)
                # Me voy a la coordenada de la base
                # self.state = 'Coords'
                self.state = "going_goal"
            elif self.state == 'going_goal':
                self.sendControlerMode('Coords')
                pose = Pose2D()
                # coordinates of base
                pose.x = 1 
                pose.y = 1
                pose.theta = 0
                self.setpoint_pub.publish(pose)
            elif self.state == 'searching_goal_ArUco':
                self.sendControlerMode('Turning')
                self.aruco = 3
                self.aruco_pub.publish(self.aruco)
                if self.found == True:
                    self.state = 'reach_goal_ArUco'
            elif self.state == 'reach_goal_ArUco':
                self.sendControlerMode('Aruco')
            elif self.state == "Stop":
                self.sendControlerMode('Off')
                vel.linear.x = 0
                vel.angular.z = 0
                print("Stopping motors")
                self.cmd_vel_pub.publish(vel)
                self.servo_pub.publish(1) 
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
