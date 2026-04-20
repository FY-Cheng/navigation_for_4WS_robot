import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_cartographer')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # cartographer_node：核心SLAM节点[reference:7]
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'),
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('points2', '/lidar/point_cloud'),
            ('imu', '/imu'),
        ],
    )
    
    # occupancy_grid_node：生成占据栅格地图（可选，用于2D导航）
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {"resolution": 0.05},
            {"min_z": 0.03},
            {"max_z": 2.3},
        ],
        arguments=['-resolution', '0.05'],
    )
    
    # rviz_node：启动RViz（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'webots_mapping.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('configuration_basename', default_value='robot_lds_3D.lua', description='Lua config file'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz'),
        cartographer_node,
        occupancy_grid_node,  # 如果只需要3D点云地图，可注释或删除此行
        rviz_node
    ])