#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray
import math

class SimpleDriveController(Node):
    def __init__(self):
        super().__init__('simple_drive_controller')
        
        # Vehicle parameters (URDF 값과 일치)
        self.WHEEL_RADIUS = 0.0508  # meters
        self.WHEELBASE = 0.3302     # meters
        self.TRACK_WIDTH = 0.2413   # meters
        
        self.drive_subscriber = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10)
        
        # Rear wheel velocity publishers
        self.right_rear_wheel_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            '/right_rear_wheel_velocity_controller/commands',
            10)

        self.left_rear_wheel_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            '/left_rear_wheel_velocity_controller/commands',
            10)

        # Steering publishers
        self.right_steering_publisher = self.create_publisher(
            Float64MultiArray,
            '/right_steering_controller/commands',
            10)

        self.left_steering_publisher = self.create_publisher(
            Float64MultiArray,
            '/left_steering_controller/commands',
            10)
    
    def calculate_ackermann_angles(self, steering_angle):
        """
        Ackermann 조향 기하학을 사용하여 좌우 바퀴의 조향각 계산
        
        공식:
        tan(δ_inner) = L / (R - W/2)
        tan(δ_outer) = L / (R + W/2)
        
        여기서:
        L = wheelbase (축간거리)
        W = track_width (윤거)
        R = turning radius (회전 반경)
        """
        if abs(steering_angle) < 0.001:  # 직진
            return 0.0, 0.0
        
        # 중심 조향각으로부터 회전 반경 계산
        try:
            turning_radius = self.WHEELBASE / math.tan(steering_angle)
        except ZeroDivisionError:
            return 0.0, 0.0

        # 내측 및 외측 바퀴의 조향각 계산
        # 분모가 0이 되는 경우 방지
        if abs(turning_radius) < self.TRACK_WIDTH / 2.0:
            # 물리적으로 불가능한 조향각 (회전 반경이 트랙 폭보다 작음)
            # 최대 조향각으로 설정하거나, 오류 처리
            # 여기서는 간단히 0으로 처리
            return 0.0, 0.0

        inner_angle = math.atan(self.WHEELBASE / (turning_radius - self.TRACK_WIDTH / 2.0))
        outer_angle = math.atan(self.WHEELBASE / (turning_radius + self.TRACK_WIDTH / 2.0))
        
        # 좌회전/우회전에 따라 좌우 바퀴 각도 할당
        if steering_angle > 0:  # 좌회전
            left_angle = inner_angle
            right_angle = outer_angle
        else:  # 우회전
            left_angle = outer_angle
            right_angle = inner_angle
        
        return left_angle, right_angle
    
    def drive_callback(self, msg):
        # 후륜 속도 제어 (각속도로 변환)
        
        angular_velocity = msg.drive.speed / self.WHEEL_RADIUS

        velocity_msg = Float64MultiArray()
        velocity_msg.data = [angular_velocity]

        self.right_rear_wheel_velocity_publisher.publish(velocity_msg)
        self.left_rear_wheel_velocity_publisher.publish(velocity_msg)

        # Ackermann 조향각 계산
        left_angle, right_angle = self.calculate_ackermann_angles(msg.drive.steering_angle)

        # 좌측 조향 명령
        left_steering_msg = Float64MultiArray()
        left_steering_msg.data = [left_angle]
        self.left_steering_publisher.publish(left_steering_msg)

        # 우측 조향 명령
        right_steering_msg = Float64MultiArray()
        right_steering_msg.data = [right_angle]
        self.right_steering_publisher.publish(right_steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
