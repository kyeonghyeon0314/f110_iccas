#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
from math import sin, cos, pi

class WheelOdometryPublisher(Node):
    def __init__(self):
        super().__init__('wheel_odometry_publisher')

        # Vehicle parameters (f1tenth.urdf.xacro와 일치)
        self.WHEEL_RADIUS = 0.0508  # meters
        self.WHEELBASE = 0.3302     # meters
        self.TRACK_WIDTH = 0.2413   # meters

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = None
        self.last_left_pos = None
        self.last_right_pos = None

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            '/wheel/odom',
            10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Wheel Odometry Publisher started')

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert euler angles to quaternion
        """
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def joint_state_callback(self, msg):
        """
        Calculate odometry from wheel encoders (joint positions)
        """
        try:
            # Find wheel indices
            left_rear_idx = msg.name.index('left_rear_wheel_joint')
            right_rear_idx = msg.name.index('right_rear_wheel_joint')
            left_steer_idx = msg.name.index('left_steering_hinge_joint')
            right_steer_idx = msg.name.index('right_steering_hinge_joint')

            # Get current positions
            left_pos = msg.position[left_rear_idx]
            right_pos = msg.position[right_rear_idx]

            # Get steering angles
            left_steer = msg.position[left_steer_idx]
            right_steer = msg.position[right_steer_idx]
            steer_angle = (left_steer + right_steer) / 2.0

            # Get current time
            current_time = self.get_clock().now()

            # Initialize on first callback
            if self.last_time is None:
                self.last_time = current_time
                self.last_left_pos = left_pos
                self.last_right_pos = right_pos
                return

            # Calculate time delta
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt <= 0.0:
                return

            # Calculate wheel displacement
            delta_left = left_pos - self.last_left_pos
            delta_right = right_pos - self.last_right_pos

            # Average wheel displacement
            delta_wheels = (delta_left + delta_right) / 2.0

            # Linear displacement of the vehicle
            delta_s = delta_wheels * self.WHEEL_RADIUS

            # Calculate angular velocity using steering angle (Ackermann)
            # v = delta_s / dt (linear velocity)
            # omega = (v / L) * tan(steering_angle)
            if abs(steer_angle) > 0.001:
                # Using Ackermann geometry
                delta_theta = (delta_s / self.WHEELBASE) * math.tan(steer_angle)
            else:
                delta_theta = 0.0

            # Update pose
            if abs(delta_theta) > 0.0001:
                # Arc movement
                radius = delta_s / delta_theta
                self.x += radius * (sin(self.theta + delta_theta) - sin(self.theta))
                self.y += radius * (-cos(self.theta + delta_theta) + cos(self.theta))
                self.theta += delta_theta
            else:
                # Straight line movement
                self.x += delta_s * cos(self.theta)
                self.y += delta_s * sin(self.theta)

            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(sin(self.theta), cos(self.theta))

            # Calculate velocities
            v_x = delta_s / dt
            v_theta = delta_theta / dt

            # Publish odometry
            self.publish_odometry(current_time, v_x, v_theta)

            # Update last values
            self.last_time = current_time
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Joint state error: {e}', throttle_duration_sec=5.0)

    def publish_odometry(self, current_time, v_x, v_theta):
        """
        Publish odometry message and TF
        """
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)

        # Set velocity
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = v_theta

        # Covariance (tune these values based on your wheel encoder accuracy)
        # Position covariance
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[14] = 1e6    # z (not used)
        odom.pose.covariance[21] = 1e6    # roll (not used)
        odom.pose.covariance[28] = 1e6    # pitch (not used)
        odom.pose.covariance[35] = 0.01   # yaw

        # Velocity covariance
        odom.twist.covariance[0] = 0.001  # vx
        odom.twist.covariance[7] = 1e6    # vy (not used)
        odom.twist.covariance[14] = 1e6   # vz (not used)
        odom.twist.covariance[21] = 1e6   # vroll (not used)
        odom.twist.covariance[28] = 1e6   # vpitch (not used)
        odom.twist.covariance[35] = 0.01  # vyaw

        # Publish odometry
        self.odom_pub.publish(odom)

        # Publish TF (odom -> base_link)
        # Note: EKF will publish this TF, so we might want to disable this later
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.quaternion_from_euler(0, 0, self.theta)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
