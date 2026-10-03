"""One-command bringup: Gazebo + robot + localization + Nav2 + GPS mission + RViz.

    ros2 launch gps_nav_robot sim.launch.py [gui:=true] [rviz:=true] [mission:=true] [record:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BAG_TOPICS = [
    '/clock', '/tf', '/tf_static', '/robot_description', '/joint_states',
    '/scan', '/gps/fix', '/gps/filtered', '/imu/data', '/camera/depth/image_raw',
    '/wheel/odom', '/odometry/local', '/odometry/global', '/odometry/gps',
    '/cmd_vel', '/plan', '/local_plan', '/diagnostics', '/min_obstacle_distance',
    '/gps_waypoints/markers',
]


def generate_launch_description():
    pkg = get_package_share_directory('gps_nav_robot')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    localization_yaml = os.path.join(pkg, 'config', 'localization.yaml')
    nav2_yaml = os.path.join(pkg, 'config', 'nav2_params.yaml')
    waypoints_yaml = os.path.join(pkg, 'config', 'waypoints.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'nav.rviz')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    mission = LaunchConfiguration('mission')
    record = LaunchConfiguration('record')
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'gps_obstacles.world')),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mission', default_value='true', description='Start GPS mission A->B'),
        DeclareLaunchArgument('record', default_value='false', description='Record rosbag'),
    ]

    # ---------- 1. Simulation ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui, 'verbose': 'false'}.items())

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}, sim_time])

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'gps_bot', '-topic', 'robot_description',
                   '-x', '-9.0', '-y', '0.0', '-z', '0.02', '-Y', '0.0'])

    # ---------- 2. Localization ----------
    ekf_odom = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
        output='screen', parameters=[localization_yaml, sim_time],
        remappings=[('odometry/filtered', 'odometry/local')])

    ekf_map = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_map',
        output='screen', parameters=[localization_yaml, sim_time],
        remappings=[('odometry/filtered', 'odometry/global')])

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
        output='screen', parameters=[localization_yaml, sim_time],
        # 'imu' covers releases where the input is named imu instead of imu/data
        remappings=[('imu', 'imu/data'), ('gps/fix', 'gps/fix'),
                    ('odometry/filtered', 'odometry/global')])

    # ---------- 3. Navigation (Nav2 without AMCL/map_server) ----------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': nav2_yaml,
                          'autostart': 'True', 'use_composition': 'False'}.items())

    # ---------- 4. Application nodes ----------
    sensor_monitor = Node(
        package='gps_nav_robot', executable='sensor_monitor', output='screen',
        parameters=[sim_time])

    waypoint_follower = Node(
        package='gps_nav_robot', executable='gps_waypoint_follower', output='screen',
        parameters=[waypoints_yaml, sim_time], condition=IfCondition(mission))

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
        parameters=[sim_time], condition=IfCondition(rviz), output='log')

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time'] + BAG_TOPICS,
        output='screen', condition=IfCondition(record))

    return LaunchDescription(args + [
        gazebo, robot_state_publisher, spawn, rviz_node, bag,
        TimerAction(period=3.0, actions=[ekf_odom, ekf_map, navsat, sensor_monitor]),
        TimerAction(period=6.0, actions=[navigation]),
        TimerAction(period=8.0, actions=[waypoint_follower]),
    ])
