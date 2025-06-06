from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro

def generate_launch_description():
    use_xacro = LaunchConfiguration('use_xacro')

    # Paths
    pkg_path = FindPackageShare('pickeycar_description').find('pickeycar_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'pickeycar.urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'pickeycar.urdf.xacro')
    print(f"URDF file: {urdf_file}")
    print(f"XACRO file: {xacro_file}")
    # Choose source based on argument
    robot_description_content = None
    if os.environ.get('USE_XACRO', 'false').lower() == 'true':
        robot_description_content = xacro.process_file(xacro_file).toxml()
    else:
        with open(urdf_file, 'r') as f:
            robot_description_content = f.read()

    # Add a file to be loaded by RViz2
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'pickeycar.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_xacro',
            default_value='false',
            description='Use xacro instead of URDF'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
