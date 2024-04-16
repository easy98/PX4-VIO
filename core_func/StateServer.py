import numpy as np

class State_Server():
    def __init__(self):

        self.imu_state = IMU_state()

        # State covariance matrix
        self.state_cov = np.zeros((21, 21))
        self.continuous_noise_cov = np.zeros((12, 12))

class IMU_state():

    def __init__(self):
        # Time when the state is recorded
        self.timestamp = None

        # Orientation
        # Take a vector from the world frame to the IMU (body) frame.
        self.orientation = np.array([0., 0., 0., 1.])

        # Position of the IMU (body) frame in the world frame.
        self.position = np.zeros(3)
        # Velocity of the IMU (body) frame in the world frame.
        self.velocity = np.zeros(3)

        # IMU acc and gyro
        self.acc = np.zeros(3)
        self.gyro = np.zeros(3)

        # IMU timesteps label (current and next timesteps)
        self.next_time = 0

        self.dt = self.next_time - self.current_time
        self.current_time = self.next_time