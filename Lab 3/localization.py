import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

Q_val = 0.9  # (from readme start from 0.5)
R_val = 0.5  # (from readme start with 0.5)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","x", "y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        # My working assumption is that the state we get from the sensors and use them to get the value
        # we know that x = [x, y, theta, v, w, vdot]
        # since the init is run first we know that v, w, vdot area all 0
        x= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # from odom
        # initial covariance of process
    
        Q = np.array(
        [[0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.3, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.3, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.25, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.25]]
        )
        Q = np.diag([Q_val, Q_val, Q_val, Q_val, Q_val, Q_val])
        # from the imu/exteroceptive sensor
        #R= R_val* np.eye(4) # initial covariance of the observation noise
        R = np.diag([R_val, R_val, R_val, R_val])
        '''
        R = R_val * np.array([
        [0.25, 0.0, 0.0, 0.0],
        [0.0, 0.25, 0.0, 0.0],
        [0.0, 0.0, 0.20, 0.0],
        [0.0, 0.0, 0.0, 0.25]]
        )
        '''

        P= 0.5 * np.eye(6) # initial guess covariance
        
        self.kf=kalman_filter(P,Q,R, x, dt)

        # Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        #stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        stamp = odom_msg.header.stamp
        # Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay

        # zk = h(xk) + vk , vk ~ N(0, Rk) https://en.wikipedia.org/wiki/Extended_Kalman_filter
        measurements = np.array([odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z,
                                 imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y])
        #z =  np.dot(self.kf.jacobian_H(), measurements.T) + self.kf.R
        z = measurements
        # Implement the two steps for estimation
        self.kf.predict()
        self.kf.update(z)
        
        # Get the estimate
        xhat=self.kf.get_states()
        print(xhat)

        # Update the pose estimate to be returned by getPose
        self.pose = np.array([xhat["kf_x"], xhat["kf_y"], xhat["theta"], stamp])

        # Part 4: log your data
        # we need to log loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):
        log_data = [xhat["kf_ax"], xhat["kf_ay"], xhat["kf_vx"], xhat["kf_w"], xhat["kf_x"], xhat["kf_y"]]
        self.loc_logger.log_values([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y]+log_data+[Time.from_msg(stamp).nanoseconds])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
