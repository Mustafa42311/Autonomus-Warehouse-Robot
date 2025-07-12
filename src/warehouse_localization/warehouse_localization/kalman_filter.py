#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")
        
        # Use a more reliable QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_sub_ = self.create_subscription(
            Odometry, 
            "warehouse_controller/odom_noisy", 
            self.odomCallback, 
            qos_profile
        )
        
        self.imu_sub_ = self.create_subscription(
            Imu, 
            "imu/out", 
            self.imuCallback, 
            qos_profile
        )
        
        self.odom_pub_ = self.create_publisher(
            Odometry, 
            "warehouse_controller/odom_kalman", 
            qos_profile
        )
        
        # Initialize state variables - for 1D Kalman filter on angular velocity
        self.x = 0.0  # State estimate (angular velocity)
        self.P = 1.0  # Estimate uncertainty/covariance
        
        # System noise parameters - critical for filter performance
        self.Q = 0.1   # Process noise - how much we expect the system to change
        self.R = 0.1  # Measurement noise - how noisy are our sensors
        
        # IMU measurement storage
        self.imu_measurement = None
        self.imu_received = False
        
        # Debug parameters
        self.debug = True
        self.measurement_count = 0
        
        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()
        
        self.get_logger().info("Enhanced Kalman Filter initialized")
    
    def odomCallback(self, odom):
        # Store the original message for republishing with filtered values
        self.kalman_odom_ = odom
        
        # Extract noisy angular velocity from odometry
        z_odom = odom.twist.twist.angular.z
        
        # Prediction step (time update)
        # For angular velocity without a motion model, we assume it remains constant
        # x = x (no change in prediction step)
        # Increase uncertainty based on process noise
        self.P = self.P + self.Q
        
        # Update step (measurement update)
        # First update using odometry measurement
        K = self.P / (self.P + self.R)  # Kalman gain
        self.x = self.x + K * (z_odom - self.x)  # Update state estimate
        self.P = (1 - K) * self.P  # Update covariance
        
        # If we have a recent IMU measurement, incorporate it too
        if self.imu_received:
            # Use a slightly higher trust for IMU (smaller R value)
            R_imu = 0.5  
            K_imu = self.P / (self.P + R_imu)
            self.x = self.x + K_imu * (self.imu_measurement - self.x)
            self.P = (1 - K_imu) * self.P
            self.imu_received = False  # Reset flag
        
        # Apply heavy exponential smoothing to further reduce noise
        alpha = 0.1  # Smoothing factor (0 < alpha < 1), smaller = more smoothing
        self.x = alpha * z_odom + (1 - alpha) * self.x
        
        # Update the filtered message
        self.kalman_odom_.twist.twist.angular.z = self.x
        
        # Publish the filtered odometry
        self.odom_pub_.publish(self.kalman_odom_)
        
        if self.debug and self.measurement_count % 100 == 0:
            self.get_logger().info(f"Noisy: {z_odom:.4f}, Filtered: {self.x:.4f}, P: {self.P:.4f}")
        
        self.measurement_count += 1
    
    def imuCallback(self, imu):
        # Store IMU measurement (with sign correction if needed)
        self.imu_measurement = imu.angular_velocity.x
        self.imu_received = True

def main():
    rclpy.init()
    kalman_filter = KalmanFilter()
    
    try:
        rclpy.spin(kalman_filter)
    except KeyboardInterrupt:
        pass
    finally:
        kalman_filter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
