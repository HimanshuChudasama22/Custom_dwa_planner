from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_bringup_pkg = get_package_share_directory('turtlebot3_bringup')
    tb3_description_pkg = get_package_share_directory('turtlebot3_description')

    # change this to your real map
    map_file = '/home/himanshu/ros2_ws/src/my_nav_pkg/maps/map.yaml'

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_model = DeclareLaunchArgument('model', default_value='waffle')  # or burger

    # 1) set env so TB3 launch doesn’t crash
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('model'))
    set_lds_model = SetEnvironmentVariable('LDS_MODEL', 'LDS-01')

    # 2) gazebo with turtlebot3 world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3) robot state publisher / bringup
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'model': LaunchConfiguration('model'),
        }.items()
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    # 4) rviz
    rviz_config = '/home/himanshu/ros2_ws/src/my_nav_pkg/launch/rviz.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # 5) our nodes
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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        set_tb3_model,
        set_lds_model,
        map_server_node,
        gazebo_launch,
        robot_state_pub,
        static_tf,
        astar_node,
        local_nav_node,
        debug_node,
        rviz_node,
    ])
