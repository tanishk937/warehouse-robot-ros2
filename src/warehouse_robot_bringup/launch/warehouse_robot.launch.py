#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

"""
Launch the complete warehouse robot demonstration.

Single entry point for the whole system (Section 24 of the upgrade
brief: "one command should launch the complete demonstration").

Starts, in order:
  1. Gazebo (gzserver + gzclient) loaded with the dressed warehouse
     world (walls, shelf racks, pallets, a charging-zone marker).
  2. robot_state_publisher, publishing the URDF as /robot_description.
  3. Spawns the robot model into the running world.
  4. sensor_fault_injector -- gates the raw Gazebo sensor topics onto
     their public names, enabling fault-injection demos with no
     process killing required.
  5. Every autonomy / safety / diagnostics node, parameters loaded from
     warehouse_robot_control's config YAML files.
  6. dashboard_bridge (unless use_dashboard:=false) -- the real-time
     web operations console.

Usage:
    ros2 launch warehouse_robot_bringup warehouse_robot.launch.py
    ros2 launch warehouse_robot_bringup warehouse_robot.launch.py gui:=false
    ros2 launch warehouse_robot_bringup warehouse_robot.launch.py use_dashboard:=false
    ros2 launch warehouse_robot_bringup warehouse_robot.launch.py world:=/path/to/other.world
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('warehouse_robot_description')
    control_pkg = get_package_share_directory('warehouse_robot_control')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(description_pkg, 'urdf', 'warehouse_robot.urdf')
    default_world_path = os.path.join(description_pkg, 'worlds', 'warehouse.world')
    sensors_yaml = os.path.join(control_pkg, 'config', 'sensors.yaml')
    robot_params_yaml = os.path.join(control_pkg, 'config', 'robot_params.yaml')
    fault_injector_yaml = os.path.join(control_pkg, 'config', 'fault_injector.yaml')

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_dashboard = LaunchConfiguration('use_dashboard')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    robot_name = LaunchConfiguration('robot_name')
    dashboard_port = LaunchConfiguration('dashboard_port')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Gazebo /clock topic as the ROS 2 time source.'
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch the Gazebo client (gzclient) GUI in addition to the server.'
    )
    declare_use_dashboard = DeclareLaunchArgument(
        'use_dashboard', default_value='true',
        description='Start the real-time web operations dashboard (warehouse_robot_dashboard).'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description=(
            'Path to the Gazebo .world file to load '
            '(default: the dressed warehouse world).'
        )
    )
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x', default_value='0.0', description='Initial X position for the spawned robot.'
    )
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y', default_value='0.0', description='Initial Y position for the spawned robot.'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='warehouse_robot',
        description=(
            'Entity name in Gazebo. Topics themselves are NOT currently '
            'namespaced by this value (see docs/architecture.md, '
            '"Multi-robot readiness" -- this is documented future work, '
            'not an implemented fleet capability).'
        )
    )
    declare_dashboard_port = DeclareLaunchArgument(
        'dashboard_port', default_value='8080',
        description='HTTP port for the dashboard web console.'
    )

    # gazebo_ros's own gazebo.launch.py already exposes 'gui' and 'world'
    # arguments, so we simply forward ours straight through instead of
    # duplicating that logic here.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'false', 'gui': gui, 'world': world}.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_warehouse_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.15',
        ],
    )

    sensor_fault_injector_node = Node(
        package='warehouse_robot_control',
        executable='sensor_fault_injector',
        name='sensor_fault_injector',
        output='screen',
        parameters=[fault_injector_yaml, {'use_sim_time': use_sim_time}],
    )

    e_stop_manager_node = Node(
        package='warehouse_robot_control',
        executable='e_stop_manager',
        name='e_stop_manager',
        output='screen',
        parameters=[robot_params_yaml, {'use_sim_time': use_sim_time}],
    )

    battery_manager_node = Node(
        package='warehouse_robot_control',
        executable='battery_manager',
        name='battery_manager',
        output='screen',
        parameters=[robot_params_yaml, {'use_sim_time': use_sim_time}],
    )

    localization_monitor_node = Node(
        package='warehouse_robot_control',
        executable='localization_monitor',
        name='localization_monitor',
        output='screen',
        parameters=[robot_params_yaml, {'use_sim_time': use_sim_time}],
    )

    sensor_heartbeat_monitor_node = Node(
        package='warehouse_robot_control',
        executable='sensor_heartbeat_monitor',
        name='sensor_heartbeat_monitor',
        output='screen',
        parameters=[sensors_yaml, {'use_sim_time': use_sim_time}],
    )

    safety_controller_node = Node(
        package='warehouse_robot_control',
        executable='safety_controller',
        name='safety_controller',
        output='screen',
        parameters=[robot_params_yaml, {'use_sim_time': use_sim_time}],
    )

    health_aggregator_node = Node(
        package='warehouse_robot_control',
        executable='health_aggregator',
        name='health_aggregator',
        output='screen',
        parameters=[robot_params_yaml, {'use_sim_time': use_sim_time}],
    )

    dashboard_bridge_node = Node(
        package='warehouse_robot_dashboard',
        executable='dashboard_bridge',
        name='dashboard_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'http_port': dashboard_port,
            'robot_id': robot_name,
        }],
        condition=IfCondition(use_dashboard),
    )

    # sensor_fault_injector must be up before anything else expects /scan,
    # /imu or /odom to exist under their public names, and everything else
    # needs Gazebo + the spawned robot to be present first.
    delayed_fault_injector = TimerAction(period=3.0, actions=[sensor_fault_injector_node])

    delayed_control_stack = TimerAction(
        period=5.0,
        actions=[
            e_stop_manager_node,
            battery_manager_node,
            localization_monitor_node,
            sensor_heartbeat_monitor_node,
            safety_controller_node,
            health_aggregator_node,
        ],
    )

    delayed_dashboard = TimerAction(period=6.0, actions=[dashboard_bridge_node])

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_use_dashboard,
        declare_world,
        declare_spawn_x,
        declare_spawn_y,
        declare_robot_name,
        declare_dashboard_port,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        delayed_fault_injector,
        delayed_control_stack,
        delayed_dashboard,
    ])
