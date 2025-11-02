from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch.logging
import os

def generate_launch_description():

    logger = launch.logging.get_logger('my_nav_launch')
    # === Locate packages ===
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_bringup_pkg = get_package_share_directory('turtlebot3_bringup')
    pkg_my_nav = get_package_share_directory('my_nav_pkg')

    # === Map and RViz config ===
    map_file = os.path.join(pkg_my_nav, 'maps', 'map.yaml')
    rviz_config = os.path.join(pkg_my_nav, 'launch', 'rviz.rviz')
    logger.info(f"Using map file: {map_file}")
    logger.info(f"Using RViz config: {rviz_config}")

    # === Launch arguments ===
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_model = DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model type')

    # === Set environment variables ===
    set_tb3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=LaunchConfiguration('model'))
    set_lds_model = SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01')

    # === Gazebo world ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # === Bringup (robot_state_publisher, sensors, etc.) ===
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # === Map Server ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # === Static TF (map → odom) ===
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # === Custom Nodes ===
    astar_node = Node(
        package='my_nav_pkg',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[{
            'map_yaml': map_file,
            'robot_radius': 0.20,
            'start': [0.0, 0.0],
            'goal': [2.0, 2.0],
        }]
    )

    local_nav_node = Node(
        package='my_nav_pkg',
        executable='local_nav',
        name='local_nav',
        output='screen',
        parameters=[{
            'look_ahead_dist': 0.6,
            'linear_speed': 0.15,
            'kp_angular': 2.0,
            'obstacle_stop_dist': 0.35,
        }]
    )

    debug_node = Node(
        package='my_nav_pkg',
        executable='debug_viz',
        name='debug_viz',
        output='screen'
    )

    # === RViz ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # === Combine all ===
    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        set_tb3_model,
        set_lds_model,
        gazebo_launch,
        robot_state_pub,
        map_server_node,
        static_tf,
        astar_node,
        local_nav_node,
        debug_node,
        rviz_node,
    ])
