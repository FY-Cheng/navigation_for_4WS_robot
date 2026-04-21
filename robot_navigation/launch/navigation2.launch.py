
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    nav_package = "robot_navigation"

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Webots) clock if true"
    )

    map_file = LaunchConfiguration("map_file")
    map_arg = DeclareLaunchArgument(
        "map_file",
        default_value="webots_map.yaml",
        description="Full path to map file to load"
    )

    params_file = LaunchConfiguration("params_file")
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value="webots_params.yaml",
        description="Full path to nav2 parameters file"
    )


    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/bringup_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': PathJoinSubstitution([get_package_share_directory(nav_package), "map", map_file]),
            'params_file': PathJoinSubstitution([get_package_share_directory(nav_package), "param", params_file]),
            'use_amcl': 'false',
        }.items()
    )


    rviz_config_dir = os.path.join(get_package_share_directory(nav_package), 'rviz', 'nav2.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # 组装启动描述
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(map_arg)
    ld.add_action(params_arg)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_node)

    return ld