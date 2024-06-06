from . import BaseController
import numpy as np
from scipy.linalg import solve_continuous_are

class Controller(BaseController):
    """
    An LQR controller
    """
    def __init__(self):
        self.A = np.array([[0, 1, 0, 0],
              [0, 0, -2, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

        self.B = np.array([[1],
              [0],
              [0],
              [1]])
        raise NotImplementedError


        # Cost matrices (Q and R should be tuned based on the specific application)
        self.Q = np.diag([0.001, 0.001, 0.001, 0.001])  # State cost matrix
        self.R = np.array([[1]])        # Control cost matrix

    def system_dynamics(self):
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R
        # Solve the continuous-time algebraic Riccati equation (CARE)
        P = solve_continuous_are(A, B, Q, R)
        # Compute the LQR gain
        K = np.linalg.inv(R) @ B.T @ P
        return K
    
    def update(self, target_lataccel, current_lataccel, state, future_plan):
        # Compute the LQR gain matrix
        K = self.system_dynamics()

        lat_error = (target_lataccel - current_lataccel)**2
        
        # Define the state vector [lateral_error, state]
        x = np.array([state.roll_lataccel, state.v_ego, state.a_ego, lat_error])

        # Compute steering angle
        u = -K @ x
        return u[0] * 0.1