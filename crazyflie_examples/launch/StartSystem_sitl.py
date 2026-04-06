import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    home_dir = os.path.expanduser('~')
    
    default_crazyflies_yaml = os.path.join(
        home_dir, 
        'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies_sitl.yaml'
    )
    
    rviz_config_path = os.path.join(
        home_dir, 
        'ros2_ws/src/crazyswarm2/crazyflie_examples/config/rviz2_custom_cfs.rviz'
    )
    
    viz_data_path = os.path.join(
        home_dir, 
        'ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data'
    )

    yaml_arg = DeclareLaunchArgument(
        'crazyflies_yaml_file',
        default_value=default_crazyflies_yaml,
        description='Percorso completo al file di configurazione crazyflies'
    )

    chosen_yaml = LaunchConfiguration('crazyflies_yaml_file')

    cf_server_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('crazyflie'), '/launch/launch_sitl.py'
                ]),
                launch_arguments={
                    'backend': 'cflib',
                    'crazyflies_yaml_file': chosen_yaml 
                }.items()
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_path],
        output='screen'
    )

    dynamic_obs_pub = Node(
        package='crazyflie_examples',
        executable='cfs_obs_pub',
        # arguments=[viz_data_path],
        output='screen'
    )

    return LaunchDescription([
        yaml_arg, 
        cf_server_launch,
        # dynamic_obs_pub,
        rviz_node,
    ])

