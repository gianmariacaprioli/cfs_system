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
        'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml'
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
                    FindPackageShare('crazyflie'), '/launch/launch.py'
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
        dynamic_obs_pub,
        # flight_server_node,
        # viz_traj_node,
        rviz_node,
    ])

# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
    
#     # --- CONFIGURAZIONE PERCORSI ---
#     home_dir = os.path.expanduser('~')
#     crazyflies_yaml_path = os.path.join(
#         home_dir, 
#         'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml' # Metti il path corretto del TUO file 1
#     )
    
#     # Percorso configurazione RViz
#     rviz_config_path = os.path.join(
#         home_dir, 
#         'ros2_ws/src/crazyswarm2/crazyflie_examples/config/rviz2_custom_cfs.rviz'
#     )
    
#     # Percorso dati per visualize_trajectory
#     viz_data_path = os.path.join(
#         home_dir, 
#         'ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data'
#     )


#     # --- 3. CRAZYFLIE SERVER (Backend cflib) ---
#     # Gestisce la comunicazione radio. Parte dopo 5s per assicurarsi che il mocap sia attivo.
#     cf_server_launch = TimerAction(
#         period=5.0,
#         actions=[
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([
#                     FindPackageShare('crazyflie'), '/launch/launch.py'
#                 ]),
#                 launch_arguments={
#                     'backend': 'cflib',
#                     'crazyflies_yaml_file': crazyflies_yaml_path # <--- QUESTA È LA CHIAVE
#                 }.items()
#             )
#         ]
#     )

#     # --- 4. FLIGHT SERVER (Il tuo Nodo Esecutore) ---
#     # Questo nodo ("cfs_server_system") inizializza Crazyswarm e aspetta comandi.
#     # Ritardo di 8s per dare tempo alla radio di connettersi.
#     flight_server_node = TimerAction(
#         period=8.0, 
#         actions=[
#             Node(
#                 package='crazyflie_examples',
#                 executable='cfs_server_system', # Assicurati che setup.py abbia questo entry point
#                 name='mission_server',
#                 output='screen',
#                 emulate_tty=False
#             )
#         ]
#     )

#     # --- 5. FLIGHT CLIENT (Il tuo Nodo Comando) ---
#     # Questo nodo ("cfs_client_system") invia la richiesta di missione.
#     # Parte dopo 12s per essere sicuri che il server sia pronto a ricevere.
#     # Usa 'xterm -e' per aprirsi in una finestra dedicata (così puoi vedere l'output pulito).
#     # flight_client_node = TimerAction(
#     #     period=12.0,
#     #     actions=[
#     #         Node(
#     #             package='crazyflie_examples',
#     #             executable='cfs_client_system', # Assicurati che setup.py abbia questo entry point
#     #             name='mission_client',
#     #             output='screen',
#     #             # Se non hai xterm, puoi rimuovere questa riga, ma l'output si mischierà agli altri
#     #             # prefix='gnome-terminal --' 
#     #         )
#     #     ]
#     # )

#     # --- 6. VISUALIZE TRAJECTORY ---
#     # Visualizza le curve CSV in RViz
#     viz_traj_node = Node(
#         package='crazyflie_examples',
#         executable='visualize_trajectory',
#         arguments=[viz_data_path],
#         output='screen'
#     )

#     # --- 7. RVIZ2 ---
#     # Interfaccia grafica
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['--display-config', rviz_config_path],
#         output='screen'
#     )

#     return LaunchDescription([
#         cf_server_launch,
#         # flight_server_node,
#         # viz_traj_node,
#         rviz_node,
#         # flight_client_node # Nota: Il client partirà per ultimo
#     ])


# # import os
# # from launch import LaunchDescription
# # from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
# # from launch.launch_description_sources import PythonLaunchDescriptionSource
# # from launch_ros.actions import Node
# # from launch_ros.substitutions import FindPackageShare
# # from ament_index_python.packages import get_package_share_directory

# # def generate_launch_description():
    
# #     # --- CONFIGURAZIONE PERCORSI ---
# #     # Recuperiamo la home directory in modo dinamico (o usiamo '/home/jimlab' fisso se preferisci)
# #     home_dir = os.path.expanduser('~')
    
# #     # Percorso configurazione RViz (preso dalla tua richiesta)
# #     rviz_config_path = os.path.join(
# #         home_dir, 
# #         'ros2_ws/src/crazyswarm2/crazyflie_examples/config/rviz2_custom_cfs.rviz'
# #     )
    
# #     # Percorso dati per visualize_trajectory
# #     viz_data_path = os.path.join(
# #         home_dir, 
# #         'ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data'
# #     )

# #     # --- 1. MOCAP DRIVER (Optitrack) ---
# #     # Equivalente a: ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# #     # NOTA: Assumo che il file optitrack2.launch.py sia nella cartella 'launch' del pacchetto
# #     mocap_launch = IncludeLaunchDescription(
# #         PythonLaunchDescriptionSource([
# #             FindPackageShare('mocap4r2_optitrack_driver'), '/launch/optitrack2.launch.py'
# #         ])
# #     )

# #     # --- 2. LIFECYCLE ACTIVATION ---
# #     # Equivalente a: ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
# #     # Usiamo un TimerAction per aspettare 3 secondi che il nodo sia su prima di attivarlo
# #     activate_mocap_cmd = TimerAction(
# #         period=3.0,
# #         actions=[
# #             ExecuteProcess(
# #                 cmd=['ros2', 'lifecycle', 'set', '/mocap4r2_optitrack_driver_node', 'activate'],
# #                 output='screen'
# #             )
# #         ]
# #     )

# #     # --- 3. CRAZYFLIE SERVER ---
# #     # Equivalente a: ros2 launch crazyflie launch.py backend:=cflib
# #     # Lo avviamo dopo 5 secondi (per essere sicuri che il mocap sia attivo)
# #     cf_server_launch = TimerAction(
# #         period=5.0,
# #         actions=[
# #             IncludeLaunchDescription(
# #                 PythonLaunchDescriptionSource([
# #                     FindPackageShare('crazyflie'), '/launch/launch.py'
# #                 ]),
# #                 launch_arguments={'backend': 'cflib'}.items()
# #             )
# #         ]
# #     )

# #     # # --- 4. POSE LISTENER ---
# #     # # Equivalente a: ros2 run crazyflie_examples cfs_pose_listener
# #     # # Avviato insieme al server crazyflie (dopo 6 secondi totali)
# #     # pose_listener_node = TimerAction(
# #     #     period=6.0,
# #     #     actions=[
# #     #         Node(
# #     #             package='crazyflie_examples',
# #     #             executable='cfs_pose_listener',
# #     #             name='cfs_pose_listener',
# #     #             output='screen'
# #     #         )
# #     #     ]
# #     # )

# #     # --- 5. VISUALIZE TRAJECTORY ---
# #     # Equivalente a: ros2 run crazyflie_examples visualize_trajectory [path]
# #     viz_traj_node = Node(
# #         package='crazyflie_examples',
# #         executable='visualize_trajectory',
# #         arguments=[viz_data_path],
# #         output='screen'
# #     )

# #     # --- 6. RVIZ2 ---
# #     # Equivalente a: rviz2 --display-config ...
# #     rviz_node = Node(
# #         package='rviz2',
# #         executable='rviz2',
# #         name='rviz2',
# #         arguments=['--display-config', rviz_config_path],
# #         output='screen'
# #     )

# #     # --- RITORNO DESCRIZIONE FINALE ---
# #     return LaunchDescription([
# #         mocap_launch,
# #         activate_mocap_cmd,
# #         cf_server_launch,
# #         # pose_listener_node,
# #         viz_traj_node,
# #         rviz_node
# #     ])