#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64, String, Int16
import numpy as np
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Pose2D, Twist
from puzzle.msg import Obstacles



MAX_ANGULAR_SPEED = 0.1
MAX_LINEAR_SPEED = 0.2

class Controler:
    def __init__(self):
        rospy.init_node('Controller', anonymous=False)
        

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cotroller_done_pub = rospy.Publisher('/controller_done', Bool, queue_size=10)

        # Subscribers to send update
        rospy.Subscriber('/pose', Pose2D, self.pose_callback)
        # Subscribers to update setpoint
        rospy.Subscriber('/setpoint', Pose2D, self.setpoint_callback)

        # Subscriber to get the current pose of arUco
        rospy.Subscriber('/marker_x', Float64, self.x_callback)
        rospy.Subscriber('/marker_z', Float64, self.z_callback)
        rospy.Subscriber('/controller_mode', String, self.mode_callback)
        rospy.Subscriber('/obstacle', Obstacles, self.obstacle_callback)
        rospy.Subscriber('/idAruco', Int16, self.arucoid_callback)



        self.currentPose = Pose2D()
        self.setpoint = Pose2D()
        self.obstacles = Obstacles()
        self.setpoint.x = 0
        self.setpoint.y = 0
        self.setpoint.theta = 0
        self.distance_aruco = 0   
        self.bug_state = "goal_seeking"
        
        self.mode = "None"
        self.done = False


        self.rate = rospy.Rate(10)  # 10Hz

    def speeds_2_wheels(self, angular_vel, linear_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)


    def setpoint_callback(self, msg):
        self.setpoint = msg

    def pose_callback(self, msg):
        self.currentPose = msg

    def stop(self):
        self.speeds_2_wheels(0, 0)
        print("Stopping motors")

    def z_callback(self, msg):
        self.z = msg.data

    def x_callback(self, msg):
        self.x = msg.data

    def mode_callback(self, msg):
        self.done = False
        self.mode = msg.data

    def obstacle_callback(self, msg):
        self.obstacles = msg

    def publish_done(self, value):
        if value != self.done:
            self.cotroller_done_pub.publish(value)
            self.done = value

    def arucoid_callback(self, msg):
        id = msg.data
        if id == 1:
            self.distance_aruco = .15
        elif id == 3:
            self.distance_aruco = .40

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == "Coords":
                 # Control
                if self.bug_state == "goal_seeking":
                    if self.obstacles.front:
                        self.bug_state = "boundary_following"
                        # self.speeds_2_wheels(0.1, 0)
                        angular_speed = -0.1
                        linear_speed = 0
                        print("Obstacle found")
                        turn = True
                    else:
                        error_y = self.setpoint.y - self.currentPose.y
                        error_x =  self.setpoint.x - self.currentPose.x
                        error_distance = sqrt(error_x * error_x + error_y * error_y)
                        if (error_distance < 0.2):
                            self.stop()
                            self.publish_done(True)
                        else:
                            # TODO: ADD BUG0 ALGORITHM
                            print("Straight")
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

                elif self.bug_state == "boundary_following":
                    if not self.obstacles.left and not self.obstacles.front and not turn:
                        self.bug_state = "goal_seeking"
                        print("No obstacle found")
                    else:
                        # Follow the boundary (simple left-hand rule)
                        if turn:
                            angular_speed = -0.1
                            linear_speed = 0
                        else:  # Move forward along the boundary
                            angular_speed = 0
                            linear_speed = 0.2
                            print("Following boundary")
                        self.speeds_2_wheels(angular_speed, linear_speed)
                        if self.obstacles.left:
                            turn = False
            elif  self.mode == "Off":
                pass
            elif self.mode == "Aruco":
                #Tenemos la posicion del Aruco
                
                diff_z = .12 * self.z
                # Giramos hasta que tenga el cubo en el centro
                print(self.x, self.z)
                if abs(self.x) > 50 and abs(self.x) < 370:
                    print("close enough")
                    diff_z = .12 * self.z
                    diff_x = -0.00015 * self.x
                elif abs(self.x) < 50:
                    diff_z = .15 * self.z
                    diff_x = 0
                    print("Avanzando")
                    if self.z < self.distance_aruco:
                        self.publish_done(True)
                else:
                    diff_x = -0.0002 * self.x
                    diff_z = 0
                    print("Vuelta")
                self.speeds_2_wheels(diff_x, diff_z)

            elif self.mode == 'Turning':
                self.speeds_2_wheels(0.15, 0)

            print(self.mode)

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