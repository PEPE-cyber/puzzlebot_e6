#!/usr/bin/env python3
import cv2
import numpy as np
import nanocamera as nano
from cv2 import aruco
import rospy
from std_msgs.msg import Bool, Float64, Int16

fps = 120

class ArucoDetector:
    def __init__(self):
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[811.48683216, 0.0, 572.46668181], 
                                       [0.0, 817.31903688, 319.25222192], 
                                       [0.0, 0.0, 1.0]])
        self.camera_coeffs = np.array([-1.24901247e-01, -1.52331073e+00, 
                                       -5.75586430e-04, -1.05189455e-02, 3.59173875e+00])
        self.camera = nano.Camera(flip=0, width=1280, height=720, fps=fps)
        rospy.init_node('ArucoDetector', anonymous=False)
        self.poseMode = True
        rospy.Subscriber('/ArucoDetectorPoseMode', Bool, self.poseModeCb)
        rospy.Subscriber('/idAruco', Int16, self.arucoid)

        self.marker_id_pub = rospy.Publisher('/marker_id', Int16, queue_size = 10)
        self.marker_x_pub = rospy.Publisher('/marker_x', Float64, queue_size = 10)
        self.marker_z_pub = rospy.Publisher('/marker_z', Float64, queue_size = 10)
        self.found_pub = rospy.Publisher('/found', Bool, queue_size = 10)
        self.search = 1


    def poseModeCb(self, msg):
        self.poseMode = msg.data

    def arucoid(self, msg):
        self.search = msg.data

    def run(self):
        rate = rospy.Rate(fps)
        while not rospy.is_shutdown():
            # Capture a frame from the camera
            frame = self.camera.read()
            msg = Bool()
            # Detect the ArUco markers in the frame
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)

            # If ArUco markers are detected
            if ids is not None:
                # Estimate the pose of the first detected marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.037, self.camera_matrix, self.camera_coeffs)

                # Draw the detected markers and their pose on the frame
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Print the marker ID, rotation vector, and translation vector
                detectedArUcos = []
                for i in range(len(ids)):
                    if ids[i][0] == self.search:
                        detectedArUcos.append({
                            'id': ids[i][0],
                            'x':  sum([coord[0] for coord in corners[i][0]]) / len( corners[i][0] ) - 670,
                            'z': tvec[i][0][2]})
                        msg.data = True
                
                if detectedArUcos :
                    self.marker_x_pub.publish(detectedArUcos[0]['x'])
                    self.marker_z_pub.publish(detectedArUcos[0]['z'])

                
                self.found_pub.publish(msg)
                print(f"State : {msg.data}")
                msg.data = False
                if detectedArUcos :
                    print(f"Marker ID: {detectedArUcos[0]['id']}")
                    print(f"X: {detectedArUcos[0]['x']}")
                    print(f"Z: {detectedArUcos[0]['z']}")
            else:
                print('No markers detected')

                msg.data = False
                self.found_pub.publish(msg)
            # show frame
            # print("yyeyyey")
            # cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()
        self.camera.release()

# Release the camera and close all windows

if __name__ == '__main__':
    try:
        arucoDetector = ArucoDetector()
        arucoDetector.run()
    except:
        arucoDetector.camera.release()


