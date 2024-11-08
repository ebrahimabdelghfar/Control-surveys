import numpy as np
class GeneralParam:
    def __init__(self):
        self.wheelbase = 4.5
        self.DT = 0.01
        self.MIN_SPEED = 0
        self.MAX_SPEED = 22
        self.MAX_STEER = np.degrees(1.1)
        self.MAX_ACCEL = 1.0
        self.controller = "pure_pursuit" # (mpc , pure_pursuit , stanley_control)
