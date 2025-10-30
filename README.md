# 실행 순서
```
chmod +x ~/f110_ws/src/docker.sh
./docker.sh
colcon build --symlink-install



ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 2.0, steering_angle: 0.4}}" --once

```