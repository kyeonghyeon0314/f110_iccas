import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_f110_description = get_package_share_directory('f110_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated clock for all F110 nodes'
    )

    # Set Gazebo model path
    models_path = os.path.join(pkg_f110_description, 'models')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_f110_description, 'iccas_f110.world'),
            'gui': 'true'
        }.items()
    )

    # URDF file
    urdf_file_name = 'urdf/f1tenth.urdf.xacro'
    urdf_file = os.path.join(pkg_f110_description, urdf_file_name)

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'f1tenth',
            '-x', '0.0',
            '-y', '-1.5',
            '-z', '0.18',  # 바퀴 반지름(0.0508m)보다 충분히 높게
            '-Y', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    simple_drive_controller = Node(
        package='f110_description',
        executable='simple_drive_controller.py',
        name='simple_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    wheel_odometry_publisher = Node(
        package='f110_description',
        executable='wheel_odometry_publisher.py',
        name='wheel_odometry_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'publish_tf': False}]
    )

    imu_republisher = Node(
        package='f110_description',
        executable='imu_republisher.py',
        name='imu_republisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    left_rear_wheel_velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_rear_wheel_velocity_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    right_rear_wheel_velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_rear_wheel_velocity_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    left_steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_steering_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    right_steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_steering_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    controller_spawner_sequence = TimerAction(
        period=2.0,
        actions=[
            joint_state_broadcaster_spawner,
            left_rear_wheel_velocity_spawner,
            right_rear_wheel_velocity_spawner,
            left_steering_spawner,
            right_steering_spawner,
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        simple_drive_controller,
        wheel_odometry_publisher,
        imu_republisher,
        controller_spawner_sequence,
    ])
