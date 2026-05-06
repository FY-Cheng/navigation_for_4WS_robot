import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    slam_mode = LaunchConfiguration("slam_mode")
    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="localization",
        choices=["mapping", "localization"],
        description="SLAM mode: mapping / localization"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="webots_2D_map",
        description="Map name (without .yaml extension)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true"
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": slam_mode,
                "map_file_name": PathJoinSubstitution([os.path.join(get_package_share_directory("robot_navigation"), "map"), map_file]),
                "resolution": 0.05,
                "min_laser_range": 0.1,
                "max_laser_range": 15.0,
                "scan_buffer_size": 10,
                "transform_publish_period": 0.05,
            }
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"node_names": ["slam_toolbox"]},
            {"autostart": True}
        ],
    )


    rviz_config_path = os.path.join(get_package_share_directory("robot_slam_toolbox"), "rviz", "webots_2D_mapping.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        slam_mode_arg,
        map_file_arg,
        use_sim_time_arg,
        use_rviz_arg,
        slam_node,
        lifecycle_manager,
        rviz_node
    ])