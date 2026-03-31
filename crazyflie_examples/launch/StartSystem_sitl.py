import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # --- 1. CONFIGURAZIONE PERCORSI DI DEFAULT ---
    home_dir = os.path.expanduser('~')
    
    # Questo è il file di default. Se non specifichi nulla da terminale, aprirà questo.
    default_crazyflies_yaml = os.path.join(
        home_dir, 
        'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies_sitl.yaml'
    )
    
    # Percorso configurazione RViz
    rviz_config_path = os.path.join(
        home_dir, 
        'ros2_ws/src/crazyswarm2/crazyflie_examples/config/rviz2_custom_cfs.rviz'
    )
    
    # Percorso dati per visualize_trajectory
    viz_data_path = os.path.join(
        home_dir, 
        'ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data'
    )

    # --- 2. DICHIARAZIONE DEGLI ARGOMENTI NEL PADRE ---
    # Fondamentale per ricevere 'crazyflies_yaml_file' dal comando tmux
    yaml_arg = DeclareLaunchArgument(
        'crazyflies_yaml_file',
        default_value=default_crazyflies_yaml,
        description='Percorso completo al file di configurazione crazyflies'
    )

    # Legge l'argomento (che sia quello di default o quello passato da terminale)
    chosen_yaml = LaunchConfiguration('crazyflies_yaml_file')

    # --- 3. CRAZYFLIE SERVER (Backend cflib) ---
    cf_server_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('crazyflie'), '/launch/launch_sitl.py'
                ]),
                launch_arguments={
                    'backend': 'cflib',
                    'crazyflies_yaml_file': chosen_yaml # <--- PASSA IL VALORE SCELTO AL FIGLIO
                }.items()
            )
        ]
    )

    # --- 4. FLIGHT SERVER (Il tuo Nodo Esecutore) ---
    flight_server_node = TimerAction(
        period=8.0, 
        actions=[
            Node(
                package='crazyflie_examples',
                executable='cfs_server_system',
                name='mission_server',
                output='screen',
                emulate_tty=False
            )
        ]
    )

    # --- 6. VISUALIZE TRAJECTORY ---
    viz_traj_node = Node(
        package='crazyflie_examples',
        executable='visualize_trajectory',
        arguments=[viz_data_path],
        output='screen'
    )

    # --- 7. RVIZ2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_path],
        output='screen'
    )

        # --- 6. VISUALIZE TRAJECTORY ---
    dynamic_obs_pub = Node(
        package='crazyflie_examples',
        executable='cfs_obs_pub',
        # arguments=[viz_data_path],
        output='screen'
    )

    # --- RITORNO DESCRIZIONE FINALE ---
    return LaunchDescription([
        yaml_arg, # DEVI aggiungere la dichiarazione qui
        cf_server_launch,
        # dynamic_obs_pub,
        # flight_server_node,
        # viz_traj_node,
        rviz_node,
    ])

