#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import math


class DriveToJointController(Node):
    """
    Converts /drive (AckermannDriveStamped) to individual joint commands
    for Gazebo simulation with proper Ackermann steering geometry
    """

    def __init__(self):
        super().__init__('drive_to_joint_controller')

        # F1TENTH vehicle parameters (from URDF)
        self.wheelbase = 0.3302  # meters (distance between front and rear axles)
        self.track_width = 0.2413  # meters (distance between left and right wheels)
        self.wheel_radius = 0.0508  # meters

        # Subscribe to /drive topic
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        # Publishers for each joint velocity
        self.left_rear_pub = self.create_publisher(
            Float64, '/left_rear_wheel_velocity_controller/command', 10)
        self.right_rear_pub = self.create_publisher(
            Float64, '/right_rear_wheel_velocity_controller/command', 10)
        self.left_front_pub = self.create_publisher(
            Float64, '/left_front_wheel_velocity_controller/command', 10)
        self.right_front_pub = self.create_publisher(
            Float64, '/right_front_wheel_velocity_controller/command', 10)

        # Publishers for steering joints
        self.left_steer_pub = self.create_publisher(
            Float64, '/left_steering_controller/command', 10)
        self.right_steer_pub = self.create_publisher(
            Float64, '/right_steering_controller/command', 10)

        self.get_logger().info('Drive to Joint Controller initialized')
        self.get_logger().info(f'Vehicle params: wheelbase={self.wheelbase}m, '
                               f'track_width={self.track_width}m, '
                               f'wheel_radius={self.wheel_radius}m')

    def drive_callback(self, msg):
        """
        Convert AckermannDriveStamped to individual joint commands

        Args:
            msg: AckermannDriveStamped with speed (m/s) and steering_angle (rad)
        """
        speed = msg.drive.speed  # m/s
        steering_angle = msg.drive.steering_angle  # radians

        # Calculate wheel velocities using Ackermann geometry
        if abs(steering_angle) < 0.001:
            # Straight line motion
            left_steer = 0.0
            right_steer = 0.0
            left_wheel_vel = speed / self.wheel_radius
            right_wheel_vel = speed / self.wheel_radius
        else:
            # Calculate turning radius at center of rear axle
            turning_radius = self.wheelbase / math.tan(abs(steering_angle))

            # Calculate inner and outer steering angles (Ackermann geometry)
            if steering_angle > 0:  # Left turn
                left_steer = math.atan(self.wheelbase / (turning_radius - self.track_width / 2))
                right_steer = math.atan(self.wheelbase / (turning_radius + self.track_width / 2))
            else:  # Right turn
                steering_angle = abs(steering_angle)
                left_steer = -math.atan(self.wheelbase / (turning_radius + self.track_width / 2))
                right_steer = -math.atan(self.wheelbase / (turning_radius - self.track_width / 2))

            # Calculate angular velocity
            angular_vel = speed / turning_radius if turning_radius > 0 else 0.0

            # Calculate wheel velocities considering differential drive
            # v_left = v - (track_width / 2) * omega
            # v_right = v + (track_width / 2) * omega
            left_linear_vel = speed - (self.track_width / 2) * angular_vel
            right_linear_vel = speed + (self.track_width / 2) * angular_vel

            # Convert linear velocity to wheel angular velocity
            left_wheel_vel = left_linear_vel / self.wheel_radius
            right_wheel_vel = right_linear_vel / self.wheel_radius

        # Publish steering angles
        left_steer_msg = Float64()
        left_steer_msg.data = float(left_steer)
        self.left_steer_pub.publish(left_steer_msg)

        right_steer_msg = Float64()
        right_steer_msg.data = float(right_steer)
        self.right_steer_pub.publish(right_steer_msg)

        # Publish wheel velocities
        # Front wheels
        left_front_msg = Float64()
        left_front_msg.data = float(left_wheel_vel)
        self.left_front_pub.publish(left_front_msg)

        right_front_msg = Float64()
        right_front_msg.data = float(right_wheel_vel)
        self.right_front_pub.publish(right_front_msg)

        # Rear wheels
        left_rear_msg = Float64()
        left_rear_msg.data = float(left_wheel_vel)
        self.left_rear_pub.publish(left_rear_msg)

        right_rear_msg = Float64()
        right_rear_msg.data = float(right_wheel_vel)
        self.right_rear_pub.publish(right_rear_msg)

        # Log for debugging
        self.get_logger().debug(
            f'[DRIVE->JOINT] IN: speed={speed:.2f} m/s, steer={steering_angle:.3f} rad | '
            f'OUT: wheel_vel={left_wheel_vel:.2f} rad/s, steer_L={left_steer:.3f}, steer_R={right_steer:.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveToJointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
