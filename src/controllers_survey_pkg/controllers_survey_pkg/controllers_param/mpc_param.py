import numpy as np

class MPCParams:
    def __init__(self):
        self.HORIZON = 30
        self.Q = np.diag([20.0, 20.0, 0.0, 0.01])
        self.R = np.diag([0.01, 45.01])
        self.Qf = np.diag([20.0, 20.0, 0.0, 0.01])
        self.Rd = np.diag([0.01, 1e-9])
        self.model_type = "continuous"
        self.NO_STATES = 4 # number of states x, y, v , yaw
        self.NO_INPUTS = 2 # number of inputs  a, delta
        self.LOOKAHEAD_DISTANCE_MPC = 8.5