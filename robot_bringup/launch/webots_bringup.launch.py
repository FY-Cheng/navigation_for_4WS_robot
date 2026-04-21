import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition



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

    use_mapping = LaunchConfiguration('use_mapping')
    use_mapping_cmd = DeclareLaunchArgument(
        'use_mapping',
        default_value='false',
        description='Use mapping or localization'
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_cartographer'),
                'launch',
                'robot_cartographer_mapping.launch.py'
            )
        ),
        condition=IfCondition(use_mapping),
        launch_arguments={
            'use_rviz': 'true',
            'use_sim_time': 'true'
        }.items()
    )


    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_cartographer'),
                'launch',
                'robot_cartographer_localization.launch.py'
            )
        ),
        condition=UnlessCondition(use_mapping),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )


    use_navigation = LaunchConfiguration('use_navigation')
    use_navigation_cmd = DeclareLaunchArgument(
        'use_navigation',
        default_value='false',
        description='Use navigation'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_navigation'),
                'launch',
                'navigation2.launch.py'
            )
        ),
        condition = IfCondition(use_navigation),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': 'webots_params.yaml'
        }.items()
    )


    return LaunchDescription([
        webots_launch,
        state_publisher_launch,

        use_mapping_cmd,
        mapping_launch,
        localization_launch,
        
        use_navigation_cmd,
        navigation_launch

    ])