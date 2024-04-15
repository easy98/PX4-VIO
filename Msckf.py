import numpy as np
from core_func.IMUPropagation import IMUPropagation
from core_func.config import config_gazebo
from math_func.quat_func import *
import time


class MSCKF():
    def __init__(self):
        self.imu_msg_buffer = []
        self.gt_msg_buffer = []
        self.__Is_init = False


    def state_init(self):
        self.position = np.array(self.gt_msg_buffer[0]['position_gt'])
        self.orientation = np.array(self.gt_msg_buffer[0]['orientation_gt'])
        self.velocity = np.array(self.gt_msg_buffer[0]['velocity_gt'])

        self.current_time = self.gt_msg_buffer[0]['Timestep']
        print("State Initilization Done!")
        print(self.current_time)

    def gt_cb(self, msg):
        self.gt_msg_buffer.append(msg)

        if self.__Is_init == False:
            self.state_init()
            self.__Is_init == True

    def imu_cb(self, msg):
        self.imu_msg_buffer.append(msg)
        # print(self.imu_msg_buffer)

    def imu_process(self):

        imu_msg_ct = 0
        # imu propagation
        for msg in self.imu_msg_buffer:
            IMU = IMUPropagation(self.position, self.velocity, self.orientation)

            self.acc = msg['acc']
            self.gyro = msg['gyro']
            self.next_time = msg['Timestep']
            self.dt = self.next_time - self.current_time
            self.current_time = self.next_time

            self.position, self.velocity, self.orientation = IMU.update_state(gyro=self.gyro, accel=self.acc, dt=self.dt)

            imu_msg_ct = imu_msg_ct + 1

    def vio_process(self, gt_data, imu_data):
        self.gt_cb(gt_data)
        self.imu_cb(imu_data)

        self.imu_process()