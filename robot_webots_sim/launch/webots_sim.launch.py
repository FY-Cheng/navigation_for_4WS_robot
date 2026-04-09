import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'robot_webots_sim'

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
        delay_driver
    ])