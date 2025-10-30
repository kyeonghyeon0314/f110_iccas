#!/usr/bin/env python3
"""
CSV 경로에서 현재 위치를 기준으로 3초 앞의 포인트를 발행하는 노드
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import csv
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np


class TrajectoryLookaheadNode(Node):
    """CSV 경로를 읽고 현재 위치에서 3초 앞의 포인트를 발행하는 노드"""

    def __init__(self):
        super().__init__('trajectory_lookahead_node')

        # 파라미터 선언
        self.declare_parameter('csv_file_path', 'trajectory.csv')
        self.declare_parameter('lookahead_time', 3.0)  # 3초
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # 파라미터 가져오기
        csv_path = self.get_parameter('csv_file_path').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        publish_rate = self.get_parameter('publish_rate').value

        # CSV 파일 읽기
        self.trajectory = self._load_trajectory(csv_path)

        if not self.trajectory:
            self.get_logger().error(f'CSV 파일을 읽을 수 없습니다: {csv_path}')
            return

        self.get_logger().info(f'경로 로드됨: {len(self.trajectory)} 포인트')

        # 구독자 생성
        qos_profile = QoSProfile(depth=10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile
        )

        # 발행자 생성
        self.target_point_publisher = self.create_publisher(
            PointStamped,
            '/target_point',
            qos_profile
        )

        # 타이머 생성
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_target_point)

        # 현재 위치 저장
        self.current_pose = None
        self.current_velocity = None

    def _load_trajectory(self, csv_path):
        """CSV 파일에서 경로 로드"""
        trajectory = []
        try:
            with open(csv_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                # 헤더 스킵 (있는 경우)
                next(reader, None)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            trajectory.append((x, y))
                        except ValueError:
                            continue
        except FileNotFoundError:
            self.get_logger().error(f'CSV 파일을 찾을 수 없습니다: {csv_path}')
            return []

        return trajectory

    def _odom_callback(self, msg: Odometry):
        """Odometry 메시지 콜백"""
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.current_velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

    def _find_closest_trajectory_point(self, current_pos):
        """현재 위치에서 가장 가까운 경로 포인트 찾기"""
        if not self.trajectory or not current_pos:
            return None, None

        min_distance = float('inf')
        closest_idx = 0

        for i, (x, y) in enumerate(self.trajectory):
            distance = math.sqrt(
                (x - current_pos[0])**2 + 
                (y - current_pos[1])**2
            )
            if distance < min_distance:
                min_distance = distance
                closest_idx = i

        return closest_idx, min_distance

    def _estimate_velocity_magnitude(self):
        """속도의 크기 계산"""
        if self.current_velocity is None:
            return 0.0

        vx, vy = self.current_velocity
        return math.sqrt(vx**2 + vy**2)

    def _calculate_arc_length(self, start_idx, end_idx):
        """경로의 호 길이 계산 (선형 거리 합)"""
        if start_idx > end_idx:
            return 0.0

        total_length = 0.0
        for i in range(start_idx, end_idx):
            x1, y1 = self.trajectory[i]
            x2, y2 = self.trajectory[i + 1]
            total_length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        return total_length

    def _find_lookahead_point(self):
        """3초 앞의 포인트 찾기"""
        if not self.current_pose:
            return None

        # 현재 위치에서 가장 가까운 경로 포인트 찾기
        closest_idx, distance = self._find_closest_trajectory_point(self.current_pose)

        if closest_idx is None:
            return None

        # 속도 크기 계산
        velocity_magnitude = self._estimate_velocity_magnitude()

        # 3초 후 예상 이동 거리
        lookahead_distance = velocity_magnitude * self.lookahead_time

        if lookahead_distance < 0.01:  # 정지 상태
            # 가장 가까운 포인트로부터 작은 거리 설정
            lookahead_distance = 0.5

        self.get_logger().debug(
            f'속도: {velocity_magnitude:.2f} m/s, '
            f'선택정 거리: {lookahead_distance:.2f} m'
        )

        # 경로 포인트를 따라가며 lookahead_distance만큼 떨어진 포인트 찾기
        cumulative_distance = 0.0

        for i in range(closest_idx, len(self.trajectory) - 1):
            x1, y1 = self.trajectory[i]
            x2, y2 = self.trajectory[i + 1]
            segment_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            if cumulative_distance + segment_length >= lookahead_distance:
                # 세그먼트 내에서 정확한 포인트 계산
                remaining_distance = lookahead_distance - cumulative_distance
                if segment_length > 0:
                    t = remaining_distance / segment_length
                    target_x = x1 + t * (x2 - x1)
                    target_y = y1 + t * (y2 - y1)
                    return (target_x, target_y)
                else:
                    return (x1, y1)

            cumulative_distance += segment_length

        # 경로의 끝에 도달한 경우
        return self.trajectory[-1]

    def _publish_target_point(self):
        """목표 포인트 발행"""
        target_point = self._find_lookahead_point()

        if target_point is not None:
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.point.x = target_point[0]
            msg.point.y = target_point[1]
            msg.point.z = 0.0

            self.target_point_publisher.publish(msg)

            self.get_logger().debug(
                f'목표 포인트 발행: ({target_point[0]:.2f}, {target_point[1]:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLookaheadNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
