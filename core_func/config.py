import numpy as np
import cv2

class config_gazebo():
    def __init__(self):
        self.n_gyro = np.array([0.01, 0.01, 0.01])
        self.n_acc = np.array([0.01, 0.01, 0.01])
        self.gravity = np.array([0, 0, 9.81])