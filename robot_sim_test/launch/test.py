from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    # Noeud principal de communication entre la simulation et ROS
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )

    # noeud du lancement de Gazebo
    gazebo_node = Node(
        package='ros_gz_sim',  # Package pour le lien ROS-Gazebo
        executable='gz_sim',   # Exécutable pour lancer la simulation
        name='gz_sim',
        output='screen',
        arguments=['-r', '/home/merlin/ros2/f1tenth/src/robot_sim_test/world/simplified_circuit_world.world']
    )

    # Lancement de la carte 2D via le map_server pour la transformation
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )

    # Gestionnaire de cycle de vie pour la carte
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # Publisher pour l'état du robot
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )

    # Ajout du script de transformation de carte 2D en 3D
    # Note: Remplacez 'path/to/your/script.py' par le chemin complet ou relatif vers votre script Python
    # transformation_node = Node(
    #     package='your_package_name',
    #     executable='python3',
    #     name='map_2d_to_3d_transform',
    #     output='screen',
    #     arguments=['/path/to/your/script.py']
    # )

    # Finalisation des actions de lancement
    ld.add_action(gazebo_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)
    
    # Décommenter pour lancer le script de transformation
    # ld.add_action(transformation_node)

    return ld
