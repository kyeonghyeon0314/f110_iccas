#!/usr/bin/env python3
"""
AckermannDrive to Twist Converter Node for F1/10
Converts ackermann_msgs/AckermannDrive to geometry_msgs/Twist
for compatibility with gazebo_ros_ackermann_drive plugin.

Author: Dr. Hyun (Principal Robotics Engineer)
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Twist
import math


class AckermannToTwist(Node):
    """
    Converts AckermannDrive messages to Twist messages using Ackermann geometry.

    Subscribed Topics:
        /drive (ackermann_msgs/AckermannDrive): Desired speed and steering angle

    Published Topics:
        /cmd_vel (geometry_msgs/Twist): Converted linear and angular velocity
    """

    def __init__(self):
        super().__init__('ackermann_to_twist')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.3302)  # F1/10 wheelbase (meters)
        self.declare_parameter('input_topic', '/drive')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('use_stamped', False)

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        use_stamped = self.get_parameter('use_stamped').value

        # Subscribe to AckermannDrive or AckermannDriveStamped
        if use_stamped:
            self.subscription = self.create_subscription(
                AckermannDriveStamped,
                input_topic,
                self.ackermann_stamped_callback,
                10
            )
            self.get_logger().info(f'Subscribed to {input_topic} (AckermannDriveStamped)')
        else:
            self.subscription = self.create_subscription(
                AckermannDrive,
                input_topic,
                self.ackermann_callback,
                10
            )
            self.get_logger().info(f'Subscribed to {input_topic} (AckermannDrive)')

        # Publisher for Twist
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.get_logger().info(f'Publishing to {output_topic} (Twist)')

        self.get_logger().info(f'Wheelbase: {self.wheelbase} m')
        self.get_logger().info('AckermannToTwist node initialized')

    def ackermann_stamped_callback(self, msg):
        """Handle AckermannDriveStamped messages"""
        self.convert_and_publish(msg.drive)

    def ackermann_callback(self, msg):
        """Handle AckermannDrive messages"""
        self.convert_and_publish(msg)

    def convert_and_publish(self, ackermann_msg):
        """
        Convert AckermannDrive to Twist using bicycle model kinematics.

        Ackermann Geometry:
            angular_velocity = (velocity * tan(steering_angle)) / wheelbase

        Args:
            ackermann_msg: AckermannDrive message
        """
        twist = Twist()

        # Linear velocity (forward/backward)
        twist.linear.x = ackermann_msg.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        # Angular velocity (calculated from steering angle)
        if abs(ackermann_msg.steering_angle) < 1e-6:
            # Straight driving
            twist.angular.z = 0.0
        else:
            # Convert steering angle to angular velocity using bicycle model
            # omega = v * tan(delta) / L
            # where: omega = angular velocity (rad/s)
            #        v = linear velocity (m/s)
            #        delta = steering angle (rad)
            #        L = wheelbase (m)
            twist.angular.z = (ackermann_msg.speed *
                              math.tan(ackermann_msg.steering_angle) /
                              self.wheelbase)

        twist.angular.x = 0.0
        twist.angular.y = 0.0

        # Publish converted message
        self.publisher.publish(twist)

        # Debug logging (ENABLED for troubleshooting)
        self.get_logger().info(
            f'[ACKERMANN->TWIST] IN: speed={ackermann_msg.speed:.2f} m/s, '
            f'steer={ackermann_msg.steering_angle:.3f} rad | '
            f'OUT: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = AckermannToTwist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AckermannToTwist node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
