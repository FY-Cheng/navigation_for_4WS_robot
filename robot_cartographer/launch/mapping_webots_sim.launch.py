import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('robot_cartographer')
    
    slam_config_dir = os.path.join(package_dir, 'config')
    slam_config_file = "webots_sim.lua"

    # 1. SLAM 核心节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        # 仿真必须开启：使用仿真时间
        parameters=[{'use_sim_time': True}],
        # 配置文件（命令行传参，正确用法）
        arguments=[
            '-configuration_directory', slam_config_dir,
            '-configuration_basename', slam_config_file
        ],
        # 话题绑定你的仿真
        remappings=[
            ('points2', '/lidar/point_cloud'),
            ('imu', '/imu'),
            ('odom', '/odom')
        ]
    )

    # 2. 地图发布节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        # 地图节点也必须用仿真时间
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05}
        ]
    )

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Use rviz if true'
    )
    rviz_config_dir = os.path.join(package_dir, 'rviz', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        use_rviz_cmd,
        rviz_node
    ])