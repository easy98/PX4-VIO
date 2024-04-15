#!/usr/bin/env python
import numpy as np

import rospy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from math_func.quat_func import *
from core_func.IMUPropagation import IMUPropagation
from queue import Queue
from threading import Thread
from Msckf import MSCKF

# define global variable

class VIO_msckf():
    def __init__(self):
        self.ros_init()
        self.vio_init()

    def ros_init(self):
        ### ROS initialize ###
        # Initialize ros node
        rospy.init_node('vio_msckf', anonymous=True)
        rospy.loginfo("ROS Initilization Finished.")

        # Subscriber
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.pose_callback)
        self.mg_sub = rospy.Subscriber("/iris_depth_camera/camera/depth/image_raw", Image, self.img_cb)

        # Publisher
        self.pose_est_pub = rospy.Publisher('/pose_est', Path, queue_size=10)

        self.dt = 0.02
        self.rate = rospy.Rate(50)  # 50Hz

        rospy.sleep(2.0)

    def vio_init(self):

        ### vio initialize ###
        self.img_queue = Queue()
        self.imu_queue = Queue()
        self.gd_truth_queue = Queue()

        self.vio_thread = Thread(target=self.msg_process)
        self.vio_thread.start()

        self.msckf = MSCKF()
        rospy.loginfo("VIO Initilization Finished.")


    def pose_callback(self, msg):
        global position_gt, orientation_gt, velocity_gt
        # This function is called when a new PoseStamped message is received
        position_gt = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        orientation_gt = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        velocity_gt = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]

        self.gt_time = msg.header.stamp.to_sec()

        self.pose_data = {'Timestep': self.gt_time, 'position_gt': position_gt,
                          'orientation_gt': orientation_gt, 'velocity_gt': velocity_gt}

        self.gd_truth_queue.put(self.pose_data)
        # self.gd_truth_queue.task_done()


    def imu_callback(self, msg):
        # Extract the angular velocity and linear acceleration data
        self.imu_time = msg.header.stamp.to_sec()

        self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        self.linear_velocity = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        self.imu_data = {'Timestep': self.imu_time, 'acc': self.linear_velocity, 'gyro': self.angular_velocity}
        self.imu_queue.put(self.imu_data)
        # self.imu_queue.task_done()

        print(self.imu_data)


    def img_cb(self, msg):
        self.img_time = msg.header.stamp.to_sec()
        self.img_data = {'Timestep': self.img_time,'img': msg}
        self.img_queue.put(self.img_data)
        self.img_queue.task_done()

    def msg_process(self):
        imu_data = self.imu_queue.get()
        img_data = self.img_queue.get()
        gt_data = self.gd_truth_queue.get()

        if imu_data is None or img_data is None or gt_data is None:
            return

        rospy.loginfo("IMU data: accel:", imu_data['acc'] , "gyro:", imu_data['gyro'])

        self.msckf.vio_process(gt_data, imu_data)



    def VIO_Propagation(self):
        # IMU propagation
        IMU = IMUPropagation(self.position, self.velocity, self.orientation)
        self.position, self.velocity, self.orientation = IMU.update_state(gyro=self.angular_velocity, accel=self.velocity, dt=self.dt)


    def pose_est_pub(self):
        pose_est = PoseStamped()
        pose_est.header.stamp = rospy.Time.now()
        pose_est.header.frame_id = 'base_footprint'

        pose_est.pose.position.x = self.position.x
        pose_est.pose.position.y = self.position.y
        pose_est.pose.position.z = self.position.z

        pose_est.pose.orientation.x = self.orientation.x
        pose_est.pose.orientation.y = self.orientation.y
        pose_est.pose.orientation.z = self.orientation.z
        pose_est.pose.orientation.w = self.orientation.w

        pose_est.publisher(pose_est)

    def spin(self):

        # Main loop
        while not rospy.is_shutdown():

            # pulish pose estimate
            # self.VIO_Propagation()
            # self.pose_pub.publish()
            self.msg_process()

            self.rate.sleep()


if __name__ == '__main__':
    node = VIO_msckf()
    node.spin()
