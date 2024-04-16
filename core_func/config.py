import numpy as np
import cv2

class config_gazebo(object):
    def __init__(self):
        self.n_gyro = np.array([0.01, 0.01, 0.01])
        self.n_acc = np.array([0.01, 0.01, 0.01])

        self.b_gyro = np.array([0.001, 0.001, 0.001])
        self.b_acc = np.array([0.001, 0.001, 0.001])

        self.gravity = np.array([0, 0, 9.81])

