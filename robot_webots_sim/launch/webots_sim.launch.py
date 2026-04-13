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

    webots = ExecuteProcess(
        cmd=['webots'],
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d', rviz_config
        ]
    )

    # 等 Webots 启动后再开驱动（工程化顺序）
    delay_driver = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=webots,
            on_start=[robot_driver_node, rviz_node]
        )
    )

    # 返回启动描述
    return LaunchDescription([
        declare_use_rviz,
        webots,
        delay_driver
    ])