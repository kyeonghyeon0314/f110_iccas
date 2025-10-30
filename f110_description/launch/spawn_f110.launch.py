import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_f110_description = get_package_share_directory('f110_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Controller configuration file
    controller_config_file = os.path.join(
        pkg_f110_description, 'config', 'f110_controllers.yaml'
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
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Broadcaster (spawned after robot spawns)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60',
                   '--param-file', controller_config_file],
        output='screen'
    )

    # Ackermann Steering Controller (spawned after joint_state_broadcaster)
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60',
                   '--param-file', controller_config_file],
        output='screen'
    )

    # Delay controller spawning until after robot is spawned
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_ackermann_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_ackermann_controller,
    ])
