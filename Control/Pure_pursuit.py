#!/usr/bin/env python3
"""
토픽 기반 Pure Pursuit 경로 추종 노드
/target_point 토픽에서 목표 포인트를 받아 경로를 추종합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
import math


class PurePursuitTracker(Node):
    """토픽 기반 Pure Pursuit 경로 추종 노드"""

    def __init__(self):
        super().__init__('pure_pursuit_tracker_topic_based')

        # Parameters
        self.declare_parameter('look_ahead_distance', 0.3)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('wheelbase', 0.33)  # F1TENTH 휠베이스
        self.declare_parameter('control_rate', 10.0)  # Hz

        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value
        control_rate = self.get_parameter('control_rate').value

        # 현재 상태
        self.current_pose = None
        self.target_point = None

        # 구독 및 발행
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # /target_point 구독 (trajectory_lookahead_node에서 발행)
        self.target_point_sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.target_point_callback,
            10
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # 제어 루프
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info('Pure Pursuit 노드 시작 (토픽 기반)')
        self.get_logger().info(f' - 구독 토픽: /odom, /target_point')
        self.get_logger().info(f' - 발행 토픽: /drive (AckermannDriveStamped)')  # 변경
        self.get_logger().info(f' - Look-ahead 거리: {self.look_ahead_distance} m')
        self.get_logger().info(f' - 최대 속도: {self.max_linear_velocity} m/s')


    def odom_callback(self, msg: Odometry):
        """현재 로봇 위치 업데이트"""
        self.current_pose = msg.pose.pose

    def target_point_callback(self, msg: PointStamped):
        """목표 포인트 업데이트 (trajectory_lookahead_node에서 수신)"""
        self.target_point = (msg.point.x, msg.point.y)

    def get_robot_yaw(self):
        """로봇의 yaw 각도 계산"""
        if self.current_pose is None:
            return 0.0

        q = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def get_robot_position(self):
        """로봇의 현재 위치"""
        if self.current_pose is None:
            return 0.0, 0.0

        return self.current_pose.position.x, self.current_pose.position.y

    def calculate_steering_angle(self, target_x, target_y):
        """Pure Pursuit 알고리즘으로 steering angle 계산"""
        robot_x, robot_y = self.get_robot_position()
        yaw = self.get_robot_yaw()

        # 로봇 좌표계로 변환
        dx = target_x - robot_x
        dy = target_y - robot_y

        # 로봇 좌표계에서의 상대 위치
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Pure Pursuit 공식
        if local_x > 0:
            # 곡률 계산: curvature = 2 * y / L²
            L_squared = local_x**2 + local_y**2
            curvature = 2.0 * local_y / (L_squared + 1e-6)

            # Ackermann steering: δ = arctan(wheelbase * curvature)
            steering_angle = math.atan(self.wheelbase * curvature)
        else:
            steering_angle = 0.0

        # 범위 제한
        max_steering = math.radians(25)  # F1TENTH 최대 조향각
        steering_angle = max(-max_steering, min(max_steering, steering_angle))

        return steering_angle

    def control_loop(self):
        """메인 제어 루프"""
        
        if self.current_pose is None:
            return
        
        if self.target_point is None:
            self.get_logger().warn_once('아직 /target_point 토픽을 받지 못했습니다')
            return
        
        # Steering angle 계산
        steering_angle = self.calculate_steering_angle(
            self.target_point[0],
            self.target_point[1]
        )
        
        # AckermannDriveStamped 메시지 생성
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = self.max_linear_velocity
        drive_msg.drive.steering_angle = steering_angle
        
        self.drive_pub.publish(drive_msg)
        
        # 디버그 정보
        self.get_logger().debug(
            f'로봇 위치: ({self.get_robot_position()[0]:.2f}, '
            f'{self.get_robot_position()[1]:.2f}) | '
            f'목표 포인트: ({self.target_point[0]:.2f}, '
            f'{self.target_point[1]:.2f}) | '
            f'조향각: {math.degrees(steering_angle):.2f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
