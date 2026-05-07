import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'robot_webots_sim'
    
    
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'webots_rviz.rviz'
    )


    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'four_wheel_steering_robot.wbt')
    webots = ExecuteProcess(
        cmd=['webots', world_file],
        output='screen'
    )

    robot_driver_node = Node(
        package=package_name,
        executable='robot_webots_sim_node',
        name='webots_driver_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}  # 重要：仿真时间
        ],
        remappings=[]
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/lidar/point_cloud"),
            ("scan", "/scan")
        ],
        parameters=[{
            # 'qos_overrides./scan.publisher.reliability': 'best_effort',
            # 'qos_overrides./scan.publisher.durability': 'volatile',
            "min_height": 0.0,   # 低于这个高度的点去掉（地面）
            "max_height": 2.0,    # 高于这个高度的点去掉（天花板）
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            "range_min": 0.3,
            "range_max": 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d', rviz_config
        ]
    )

    # 等 Webots 启动后再开驱动（工程化顺序）
    delay_driver = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=webots,
            on_start=[robot_driver_node]
        )
    )

    # 返回启动描述
    return LaunchDescription([
        webots,
        delay_driver, 
        declare_use_rviz,
        pointcloud_to_laserscan,
        rviz_node
    ])