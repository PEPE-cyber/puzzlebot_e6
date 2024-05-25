import numpy as np

class EKF:
    def __init__(self, initial_state=np.zeros(3), dt=0.1 ):
        # sampling time
        self.dt = dt
        # estimated state
        self.state = initial_state
        # Covariance matrix
        self.P = np.eye(3)
        # Nondeterministic error
        self.Q = [[0.5, 0.01, 0.01],
                  [0.01, 0.5, 0.01],
                  [0.01, 0.01, 0.2]]
        # Initialize state
        self.state = np.zeros((3,1))
        # Jacobian of the system
        self.H = np.eye(3)

    def stateTrasitionModel(self, u):
       return np.array([self.state[0] + u[0] * self.dt * np.cos(self.state[2]),
                            self.state[1] + u[0] * self.dt * np.sin(self.state[2]),
                            self.state[2] + u[1] * self.dt])
        
    def jacobian(self, u):
        self.H = np.array([[1, 0, -u[0] * self.dt * np.sin(self.state[2])],
                           [0, 1, u[0] * self.dt * np.cos(self.state[2])],
                           [0, 0, 1]])
        
    def update(self, u):
        state = self.stateTrasitionModel(u)
        self.jacobian(u)
        self.P = np.matmul(np.matmul(self.H, self.P), self.H.T) + self.Q
        self.state = state
    
    def get_state(self):
        


if __name__ == '__main__':
    try:
        ekn = ExtendedKalmanNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass