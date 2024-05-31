import numpy as np
from math import atan2


class EKF:
    def __init__(self, initial_state=np.zeros(3), dt=0.1 ):
        # sampling time
        self.dt = dt
        # estimated state
        self.state = initial_state
        self.prev_state = initial_state
        # Covariance matrix
        self.P = np.eye(3)
        # Nondeterministic error 
        # TODO: Tune this values
        self.Q = [[0.5, 0.01, 0.01],
                  [0.01, 0.5, 0.01],
                  [0.01, 0.01, 0.2]]
        # Initialize state
        self.state = np.zeros((3,1))
        # Jacobian of the system
        self.H = np.eye(3)

    def stateTrasitionModel(self, u):
       self.prev_state = self.state
       self.state = np.array([self.state[0] + u[0] * self.dt * np.cos(self.state[2]),
                            self.state[1] + u[0] * self.dt * np.sin(self.state[2]),
                            self.state[2] + u[1] * self.dt])
        # Calculate the propagation of the uncertainty
        # update the jacobian
        self.updateJacobian(u)
        # Update the covariance matrix
        self.P = np.matmul(np.matmul(self.H, self.P), self.H.T) + self.Q

    def correction(self, Z, landmark):
        # Location of landmarks
        l1 = np.array([0, 0])
        l2 = np.array([0, 1])
        # Calculate the observation model
        z1 = np.array([np.sqrt((self.state[0] - l1[0])**2 + (self.state[1] - l1[1])**2), 
                       atan2(self.state[1] - l1[1], self.state[0] - l1[0]) - self.state[2]])
        z2 = np.array([np.sqrt((self.state[0] - l2[0])**2 + (self.state[1] - l2[1])**2),
                          atan2(self.state[1] - l2[1], self.state[0] - l2[0]) - self.state[2]])
        z = np.array([z1, z2])

        # Calculate the jacobian of the observation model
        H = np.array([[-(self.state[0] - l1[0]) / z1[0], -(self.state[1] - l1[1]) / z1[0], 0],
                        [(self.state[1] - l1[1]) / (z1[0]**2), -(self.state[0] - l1[0]) / (z1[0]**2), -1],
                        [-(self.state[0] - l2[0]) / z2[0], -(self.state[1] - l2[1]) / z2[0], 0],
                        [(self.state[1] - l2[1]) / (z2[0]**2), -(self.state[0] - l2[0]) / (z2[0]**2), -1]])
        # uncertainty propagation of the observation model
        R = np.array([[0.1, 0],
                        [0, 0.02]]) # convert to 0
        S = np.matmul(np.matmul(H, self.P), H.T) + R
        # Calculate the Kalman gain
        K = np.matmul(np.matmul(self.P, H.T), np.linalg.inv(S))
        # Calculate the state 
        self.ekfState = self.state + np.matmul(K, (Z - z))

        
    def updateJacobian(self, u):
        self.H = np.array([[1, 0, -u[0] * self.dt * np.sin(self.state[2])],
                           [0, 1, u[0] * self.dt * np.cos(self.state[2])],
                           [0, 0, 1]])
        
    def update(self, u):
        state = self.stateTrasitionModel(u)
        self.updateJacobian(u)
        self.P = np.matmul(np.matmul(self.H, self.P), self.H.T) + self.Q
        self.state = state
    
    def get_state(self):
        


if __name__ == '__main__':
    try:
        ekn = ExtendedKalmanNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass