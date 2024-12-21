import os
import sys
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node

def generate_launch_description():
    # Charger la configuration du fichier map.yaml
    config_path = os.path.join(
        get_package_share_directory('robot_sim_test'),
        'maps',
        'map3.yaml'  # Le fichier de la carte YAML
    )

    # Charger le fichier YAML
    with open(config_path, 'r') as file:
        config_dict = yaml.safe_load(file)

    # Extraction de l'origine et de la résolution de la carte
    origin = config_dict.get('origin', [0, 0, 0])  # L'origine de la carte [x, y, theta]
    resolution = config_dict.get('resolution', 0.05)  # Résolution de la carte (en mètres par pixel)

    # Déclaration de l'argument pour la version du monde
    world_version = DeclareLaunchArgument(
        'world_version', default_value=TextSubstitution(text='simplified_circuit_world.world')
    )

    contexte_world = LaunchContext()
    # Gestion de la version du monde
    if len(sys.argv) > 4:
        if len(sys.argv) == 5:
            if ((sys.argv[4]).split('='))[0] == "world_version:":
                contexte_world.launch_configurations['world_version'] = sys.argv[4]
        else:
            contexte_world.launch_configurations['world_version'] = sys.argv[5]
    else:
        contexte_world.launch_configurations['world_version'] = "world_version:=simplified_circuit_world.world"

    type_world = ((LaunchConfiguration('world_version')).perform(contexte_world))
    world = type_world.split('=')

    # Arguments de lancement
    script_path = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.dirname(script_path)
    script_path = os.path.join(script_path, "gazebo")

    try:
        prev_env = os.environ["IGN_GAZEBO_RESOURCE_PATH"] + ":"
    except KeyError:
        prev_env = ""
    str_env = prev_env + f'{os.path.join(script_path,"worlds")}:{os.path.join(script_path,"models")}'
    env = launch.actions.SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", str_env)

    # Paramètres de simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    simulation_path = os.path.join(get_package_share_directory('robot_sim_test'))
    xacro_file = os.path.join(simulation_path, 'xacro', 'ego_racecar.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    # Fichier du monde Gazebo
    world_file = os.path.join(simulation_path, 'world', world[1])

    config_file = os.path.join(
        get_package_share_directory('robot_sim_test'),
        'config',
        'diff_drive_controller_velocity_robot.yaml'
    )

    # Nœud robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    
    # Nœud controller_manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='screen',
        parameters=[config_file],  # Passage du fichier de config
        remappings=[('/joint_states', '/robot/joint_states')],  # Si nécessaire, ajustez le remapping
    )

    # Commandes pour charger les contrôleurs
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    # Création de l'entité Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'ego_racecar',
                   '-allow_renaming', 'true',
                   '-x', '-1.5',
                   '-y', '-6.5',
                   '-z', '1'],
    )

    # Bridge pour la communication avec Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/model/ego_racecar/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                   '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                   #'/model/ego_racecar/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        output='screen'
    )

    # Retourner la description du lancement
    return LaunchDescription([
        world_version,
        env,
        bridge,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ' + world_file])],
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        controller_manager_node,
        node_robot_state_publisher,
        ignition_spawn_entity,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
    ])
