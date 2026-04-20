import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_cartographer')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true')

    configuration_basename = LaunchConfiguration('configuration_basename')
    configuration_basename_cmd = DeclareLaunchArgument('configuration_basename', default_value='localization.lua')

    load_state_filename = LaunchConfiguration('load_state_filename')
    load_state_filename_cmd = DeclareLaunchArgument('load_state_filename', default_value='3d_map.pbstream')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'),
            '-configuration_basename', configuration_basename,
            '-load_state_filename', PathJoinSubstitution([pkg_share, 'map', load_state_filename]),
        ],
        remappings=[
            ('points2', '/lidar/point_cloud'),
            ('imu', '/imu'),
        ],
    )
    
    return LaunchDescription([
        use_sim_time_cmd,
        configuration_basename_cmd,
        load_state_filename_cmd,
        cartographer_node
    ])