
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


    # nav2_bringup 禁用了 'use_localization' 后，map_server也会被禁用，所以在此单独启动
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': PathJoinSubstitution([get_package_share_directory(nav_package), "map", map_file])
        }]
    )

    # 这是核心！负责激活 map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'autostart': True,  # 自动激活节点
             'node_names': ['map_server']}  # 管理的节点
        ]
    )


    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/bringup_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'map': PathJoinSubstitution([get_package_share_directory(nav_package), "map", map_file]),
            'params_file': PathJoinSubstitution([get_package_share_directory(nav_package), "param", params_file]),
            'use_localization': 'False' # 禁用 localization_launch.py，从而禁用 AMCL，通过外部定位节点提供位姿信息。但需要单独启动地图服务器
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
    # ld.add_action(map_server_node)
    # ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)

    return ld