import math
from math_func.quat_func import *
import numpy as np
from core_func.config import config_gazebo


class IMU_Propagation():
    def __init__(self, state_server, config):
        self.position = state_server.imu_state.position # Position vector in the global frame
        self.velocity = state_server.imu_state.velocity  # Velocity vector in the global frame
        self.orientation = state_server.imu_state.orientation  # Initial orientation as a quaternion

        # parameter configutation
        self.config = config

        self.n_gyro = self.config.n_gyro
        self.n_acc = self.config.n_acc
        self.gravity = self.config.gravity



    def update_orientation(self, gyro, dt):
        """
        Updates the orientation based on gyroscope data according to JPL quaternion convention.
        """
        omega_magnitude = math.sqrt(sum(g**2 for g in gyro))
        if omega_magnitude > 0:
            delta_theta = omega_magnitude * dt
            delta_q = quat_integral(gyro,dt)
            # Quaternion multiplication for orientation update
            self.orientation = delta_q @ self.orientation
            self.orientation = quat_normalize(self.orientation)

    def update_velocity_and_position(self, accel, dt):
        """
        Updates the velocity and position based on accelerometer data.
        Acceleration is in the body frame and is converted to the global frame.
        """
        self.accel_G = np.dot(quattorot(self.orientation), accel) - self.gravity
        self.position = self.position + self.velocity * dt + 0.5 * self.accel_G * dt * dt
        self.velocity = self.velocity + self.accel_G * dt




    def update_state(self, gyro, accel, dt):
        """
        Combined update function for convenience.
        """
        self.update_velocity_and_position(accel, dt)
        self.update_orientation(gyro, dt)

        return self.position, self.velocity, self.orientation

    def update_cov(self, gyro, accel, dt):
        F = np.zeros((21, 21))
        G = np.zeros((21, 12))

        # compute the transition matrix
        F[:3,:3] = -skew(gyro)
        F[:3,3:6] = -np.identity(3)
        F[6:9,:3] = -quattorot(self.orientation).T @ skew(accel)

        F[6:9,9:12] = -quattorot(self.orientation).T
        F[12:15,6:9] = np.identity(3)

        G[:3,:3] = -np.identity(3)
        G[3:6,3:6] = np.identity(3)
        G[6:9,6:9] = -quattorot(self.orientation).T
        G[9:12, 9:12] = np.identity(3)

        Fdt = F * dt
        Fdt_square = Fdt @ Fdt
        Fdt_cube = Fdt_square @ Fdt

        Phi = np.identity(21)  + Fdt + 0.5*Fdt_square + 1/6*Fdt_cube

        # propagation the covariance matrix
        Q = Phi @ G @ G.T @ Phi.T * dt






