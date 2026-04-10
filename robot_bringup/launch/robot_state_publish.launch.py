import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )


    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'four_wheel_steering_robot.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()


    robot_state_publisher_node = Node (
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        robot_state_publisher_node
    ])
