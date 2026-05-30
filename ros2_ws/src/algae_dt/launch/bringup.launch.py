"""bringup.launch.py — orchestrates the DEFAULT turtlebot3 stack + our algae_dt DT layer.

    ros2 launch algae_dt bringup.launch.py mode:=sim_only            # home dev (default)
    ros2 launch algae_dt bringup.launch.py mode:=sim_only headless:=true   # CI / no display
    ros2 launch algae_dt bringup.launch.py mode:=real_only          # robot bringup runs on the Pi
    ros2 launch algae_dt bringup.launch.py mode:=both               # real leads, sim mirrors (/sim/*)

Modes (RULES/SETUP/DECISIONS): sim_only develops at home; real_only/both are tested in the lab.
Built with an OpaqueFunction so mode/headless are plain Python booleans (clearer than nested
substitutions).

cmd_vel TYPE CONTRACT (RULES §B-1/§B-2, verified in the container): the stock turtlebot3
`burger.yaml` already sets `enable_stamped_cmd_vel: true` on controller/behavior/smoother/
collision_monitor, so Nav2 speaks TwistStamped — same as the real turtlebot3_node, teleop and the
gz bridge. We route Nav2's FINAL velocity into our safety chokepoint by rewriting the
collision_monitor `cmd_vel_out_topic` to /dt/cmd_vel_raw (a blanket /cmd_vel remap would double the
bus because controller + collision_monitor both use `cmd_vel` internally). The mediator gates
/dt/cmd_vel_raw and republishes /cmd_vel to the robot (+ /sim/cmd_vel in `both`).

sim_only AMCL auto-seed: the params_file flips `set_initial_pose:true` (initial_pose 0,0,0 = the
Gazebo spawn) so headless sim_only localises with no human 2D-Pose-Estimate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _src(pkg: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(pkg), *parts)


def _gz_sim(gz_args: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('ros_gz_sim', 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('algae_dt')
    params = os.path.join(pkg, 'config', 'twin.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'map.yaml')
    world = os.path.join(pkg, 'worlds', 'algae_arena.world')

    mode = LaunchConfiguration('mode').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'

    sim = mode in ('sim_only', 'both')
    nav = mode in ('sim_only', 'real_only')        # autonomy stack (Nav2/AMCL); `both` mirrors only
    use_sim_time = (mode == 'sim_only')
    use_sim_time_str = 'true' if use_sim_time else 'false'

    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    actions = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', os.environ.get('TURTLEBOT3_MODEL', 'burger')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               _src('turtlebot3_gazebo', 'models') + os.pathsep + os.path.join(pkg, 'worlds')),
    ]
    if headless:
        # turtlebot3_navigation2 launches its own RViz unconditionally; offscreen keeps it from
        # crashing on a display-less CI host (it renders to nothing instead of aborting).
        actions.append(SetEnvironmentVariable('QT_QPA_PLATFORM', 'offscreen'))

    # ---- stock Gazebo sim into OUR arena world (sim_only/both) — PLAN T1.1 ----
    if sim:
        server = '-r -s -v2 ' + ('--headless-rendering ' if headless else '') + world
        actions.append(_gz_sim(server))
        if not headless:
            actions.append(_gz_sim('-g -v2 '))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_src('turtlebot3_gazebo', 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time_str}.items()))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_src('turtlebot3_gazebo', 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()))

    # ---- stock Nav2 (+AMCL+map) with our rewrites (sim_only/real_only) — PLAN T1.1/T2.2 ----
    if nav:
        nav2_params = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                                     'param', 'burger.yaml'),
            param_rewrites={
                'set_initial_pose': 'True',           # auto-localise headless sim_only at the spawn
                'cmd_vel_out_topic': '/dt/cmd_vel_raw',  # Nav2's FINAL velocity -> our safety bus
                'use_sim_time': use_sim_time_str,
            },
            convert_types=True)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_src('turtlebot3_navigation2', 'launch', 'navigation2.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time_str,
                              'map': map_yaml,
                              'params_file': nav2_params}.items()))

    # ---- our DT layer (all modes); params bind via the /** wildcard in twin.yaml ----
    common = [params, {'use_sim_time': use_sim_time, 'mode': mode}]
    actions += [
        Node(package='algae_dt', executable='twin_mediator', output='screen', parameters=common),
        Node(package='algae_dt', executable='sync_supervisor', output='screen', parameters=common),
        # NEVER set name= on mission_runner (process-wide remap trap, BEST_APPROACHES); it carries no
        # cmd_vel remap (Nav2's final cmd_vel is rewritten to the bus above).
        Node(package='algae_dt', executable='mission_runner', output='screen', parameters=common),
    ]
    if not headless:
        actions.append(Node(package='algae_dt', executable='operator_gui', output='screen', parameters=common))
    if use_rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='screen'))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim_only',
                              description='sim_only | real_only | both'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true => gz server only (--headless-rendering), no GUI/RViz'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
