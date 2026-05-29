"""bringup.launch.py — orchestrates the DEFAULT turtlebot3 stack + our algae_dt DT layer.

Modes (RULES/SETUP): sim_only (default, home dev), real_only, both.
This is the combined-launch convenience path; the TA-familiar multi-terminal fallback is documented
in docs/BEST_APPROACHES.md. Integration points still being wired are marked TODO with PLAN task IDs.

    ros2 launch algae_dt bringup.launch.py mode:=sim_only
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('algae_dt')
    params = os.path.join(pkg, 'config', 'twin.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'map.yaml')

    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('use_rviz')

    is_sim = PythonExpression(["'", mode, "' == 'sim_only'"])
    use_sim_time = is_sim  # true ONLY in sim_only (RULES.md §B-4)

    # --- our DT layer (all modes); params bind via the /** wildcard in twin.yaml ---
    common = [{'use_sim_time': use_sim_time}, params, {'mode': mode}]
    dt_nodes = GroupAction([
        Node(package='algae_dt', executable='twin_mediator', output='screen', parameters=common),
        Node(package='algae_dt', executable='sync_supervisor', output='screen', parameters=common),
        # NOTE: never set name= on mission_runner (process-wide remap trap, BEST_APPROACHES).
        Node(package='algae_dt', executable='mission_runner', output='screen', parameters=common,
             remappings=[('/cmd_vel', '/dt/cmd_vel_raw')]),
        Node(package='algae_dt', executable='operator_gui', output='screen', parameters=common),
    ])

    # --- stock Nav2 (sim_only uses sim time + our map) — PLAN T1.1 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map_yaml}.items(),
    )

    # --- stock RViz (optional) ---
    rviz = Node(package='rviz2', executable='rviz2', output='screen', condition=IfCondition(use_rviz))

    # TODO PLAN T1.1: include turtlebot3_gazebo with our worlds/algae_arena.world (sim_only/both).
    # TODO PLAN T5.1: in `both`, push the sim into the /sim/* namespace + route sim TF to /sim/tf,
    #                 and bring up turtlebot3_bringup on the robot Pi (done over SSH, not here).

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim_only',
                              description='sim_only | real_only | both'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        nav2,
        dt_nodes,
        rviz,
    ])
