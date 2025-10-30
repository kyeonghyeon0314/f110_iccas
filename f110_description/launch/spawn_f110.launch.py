import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_f110_description = get_package_share_directory('f110_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_f110_description, 'iccas_f110.world')}.items()
    )

    # URDF file
    urdf_file_name = 'f1tenth.urdf'
    urdf_file = os.path.join(pkg_f110_description, urdf_file_name)
    
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'f1tenth'],
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

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
