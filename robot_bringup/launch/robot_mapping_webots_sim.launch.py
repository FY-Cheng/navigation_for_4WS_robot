import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_webots_sim'),
                'launch',
                'webots_sim.launch.py'
            )
        ),
        launch_arguments={
            'use_rviz': 'false'
        }.items()
    )

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'launch',
                'robot_state_publish.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    webots_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_cartographer'),
                'launch',
                'mapping_webots_sim.launch.py'
            )
        ),
        launch_arguments={
            'use_rviz': 'false'
        }.items()
    )


    return LaunchDescription([
        webots_launch,
        state_publisher_launch,
        webots_mapping_launch
    ])