#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')

        # Subscribe to raw IMU data from Gazebo (without covariance)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)

        # Publish IMU data with proper covariance
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            10)

        self.get_logger().info('IMU Republisher started')
        self.get_logger().info('Subscribing to: /imu/data_raw')
        self.get_logger().info('Publishing to: /imu/data')

    def imu_callback(self, msg):
        """
        Add covariance values to IMU data
        Covariance matrix is 3x3, stored as a row-major array of 9 elements
        """
        # Create new message (copy original)
        imu_msg = Imu()

        # Copy header
        imu_msg.header = msg.header

        # Copy orientation
        imu_msg.orientation = msg.orientation

        # Set orientation covariance (roll, pitch, yaw) in rad^2
        # Good orientation accuracy: 0.001 rad^2 (~1.8 degrees std dev)
        imu_msg.orientation_covariance = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.001
        ]

        # Copy angular velocity
        imu_msg.angular_velocity = msg.angular_velocity

        # Set angular velocity covariance (x, y, z) in (rad/s)^2
        # Good gyroscope accuracy: 0.0004 (rad/s)^2 (~0.02 rad/s std dev)
        imu_msg.angular_velocity_covariance = [
            0.0004, 0.0,    0.0,
            0.0,    0.0004, 0.0,
            0.0,    0.0,    0.0004
        ]

        # Copy linear acceleration
        imu_msg.linear_acceleration = msg.linear_acceleration

        # Set linear acceleration covariance (x, y, z) in (m/s^2)^2
        # Good accelerometer accuracy: 0.001 (m/s^2)^2 (~0.03 m/s^2 std dev)
        imu_msg.linear_acceleration_covariance = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.001
        ]

        # Publish corrected IMU data
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
