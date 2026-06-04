"""bringup.launch.py — orchestrates the DEFAULT turtlebot3 stack + our algae_dt DT layer.

    ros2 launch algae_dt bringup.launch.py mode:=sim_only                  # home dev (default)
    ros2 launch algae_dt bringup.launch.py mode:=sim_only headless:=true   # CI / no display
    ros2 launch algae_dt bringup.launch.py mode:=real_only                 # robot bringup on the Pi
    ros2 launch algae_dt bringup.launch.py mode:=both                      # real leads, sim mirrors
    ros2 launch algae_dt bringup.launch.py mode:=both use_fake_robot:=true # both, hardware-free @home

Built with an OpaqueFunction so mode/headless are plain Python booleans.

Topic model (RULES §B-3): the ACTIVE robot is on BARE topics (/scan,/odom,/cmd_vel,/battery_state)
— in sim_only that bare robot IS the Gazebo robot; in real_only/both it is the real Burger (or
fake_robot at home). In `both` the mirroring sim lives entirely on /sim/* (config/sim_bridge.yaml +
namespaced RSP, sim TF off the global /tf), so it never collides with the real robot's bare topics.

cmd_vel chokepoint (RULES §B-1/§B-2, verified in-container): stock burger.yaml already sets
enable_stamped_cmd_vel:true everywhere (TwistStamped end-to-end). We rewrite collision_monitor's
cmd_vel_out_topic -> /dt/cmd_vel_raw so Nav2's FINAL velocity passes through the mediator's safety
gate before reaching /cmd_vel (+ /sim/cmd_vel mirror in `both`). sim_only flips set_initial_pose:true
for headless AMCL auto-seed (D8: use_sim_time true ONLY in sim_only).
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


def _sim_bringup(pkg, world, headless, gz_gui):
    """sim_only: stock bare-topic Gazebo robot (model + RSP + bare ros_gz bridge).
    gz_gui=False runs the server only (it still GPU-renders the sensors via the display context) and
    skips the heavy gz 3D client — useful where that client can't init (e.g. OGRE2 on the WSL d3d12 GL,
    which refuses its large VBO); drive/visualise via operator_gui + RViz instead."""
    actions = [_gz_sim('-r -s -v2 ' + ('--headless-rendering ' if headless else '') + world)]
    if not headless and gz_gui:
        actions.append(_gz_sim('-g -v2 '))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('turtlebot3_gazebo', 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('turtlebot3_gazebo', 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()))
    return actions


def _sim_mirror(pkg, world, headless):
    """both: the Gazebo mirror pushed entirely onto /sim/* (custom bridge + namespaced RSP)."""
    model_sdf = _src('turtlebot3_gazebo', 'models', 'turtlebot3_burger', 'model.sdf')
    urdf = _src('turtlebot3_gazebo', 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    actions = [_gz_sim('-r -s -v2 ' + ('--headless-rendering ' if headless else '') + world)]
    if not headless:
        actions.append(_gz_sim('-g -v2 '))
    actions += [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': False, 'robot_description': robot_desc, 'frame_prefix': 'sim/'}],
             remappings=[('/tf', '/sim/tf'), ('/tf_static', '/sim/tf_static')]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'burger_sim', '-file', model_sdf, '-x', '0.0', '-y', '0.0', '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p', f"config_file:={os.path.join(pkg, 'config', 'sim_bridge.yaml')}"]),
    ]
    return actions


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('algae_dt')
    params = os.path.join(pkg, 'config', 'twin.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'map.yaml')
    world = os.path.join(pkg, 'worlds', 'algae_arena.world')

    mode = LaunchConfiguration('mode').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    use_fake_robot = LaunchConfiguration('use_fake_robot').perform(context).lower() == 'true'
    gz_gui = LaunchConfiguration('gz_gui').perform(context).lower() == 'true'

    use_sim_time = (mode == 'sim_only')
    use_sim_time_str = 'true' if use_sim_time else 'false'

    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    actions = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', os.environ.get('TURTLEBOT3_MODEL', 'burger')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               _src('turtlebot3_gazebo', 'models') + os.pathsep + os.path.join(pkg, 'worlds')),
    ]

    if mode == 'sim_only':
        actions += _sim_bringup(pkg, world, headless, gz_gui)
    elif mode == 'both':
        actions += _sim_mirror(pkg, world, headless)

    # The real side at home (no hardware): fake_robot publishes the bare /scan /odom /cmd_vel(sub)
    # /battery_state so real_only/both can be exercised + the topic-collision rule validated (T5.1b).
    if use_fake_robot and mode in ('real_only', 'both'):
        actions.append(Node(package='algae_dt', executable='fake_robot', output='screen', parameters=[params]))

    # Stock Nav2 (+AMCL+map) with our rewrites — runs in every mode (on the active/bare robot).
    rewrites = {
        'cmd_vel_out_topic': '/dt/cmd_vel_raw',     # Nav2's FINAL velocity -> our safety bus
        'use_sim_time': use_sim_time_str,
    }
    if mode == 'sim_only':
        rewrites['set_initial_pose'] = 'True'        # headless auto-seed at the spawn (no human 2D pose)
        # The GPU-less / WSL software render gives ~2.5-3.5 Hz LiDAR; the stock collision_monitor
        # scan source_timeout (0.2 s) then rejects every scan ("invalid source") and halts autonomy.
        # Loosen it to 2.0 s for SIM ONLY so the demo navigates; real_only/both keep the stock 0.2 s.
        rewrites['source_timeout'] = '2.0'
        # Same throttle makes the controller sluggish, so the default progress checker (move 0.5 m
        # within 10 s) aborts ("Failed to make progress") before the robot settles into the goal ->
        # blooms skipped, never sprayed. Loosen it for SIM ONLY (real_only/both keep stock); our own
        # nav_goal_timeout_s still bounds a truly stuck goal.
        rewrites['movement_time_allowance'] = '30.0'
        rewrites['required_movement_radius'] = '0.1'
    nav2_params = RewrittenYaml(
        source_file=_src('turtlebot3_navigation2', 'param', 'burger.yaml'),
        param_rewrites=rewrites, convert_types=True)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('nav2_bringup', 'launch', 'bringup_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time_str, 'map': map_yaml,
                          'params_file': nav2_params, 'autostart': 'true'}.items()))

    # ---- our DT layer (all modes); params bind via the /** wildcard in twin.yaml ----
    common = [params, {'use_sim_time': use_sim_time, 'mode': mode}]
    actions += [
        Node(package='algae_dt', executable='twin_mediator', output='screen', parameters=common),
        Node(package='algae_dt', executable='sync_supervisor', output='screen', parameters=common),
        # NEVER set name= on mission_runner (process-wide remap trap, BEST_APPROACHES).
        Node(package='algae_dt', executable='mission_runner', output='screen', parameters=common),
    ]
    if not headless:
        actions.append(Node(package='algae_dt', executable='operator_gui', output='screen', parameters=common))
    if use_rviz:
        rviz_cfg = _src('turtlebot3_navigation2', 'rviz', 'tb3_navigation2.rviz')
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
                            parameters=[{'use_sim_time': use_sim_time}], output='screen'))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim_only', description='sim_only | real_only | both'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true => gz server only (--headless-rendering), no GUI/RViz'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_fake_robot', default_value='false',
                              description='real_only/both: spawn the kinematic fake_robot (no hardware)'),
        DeclareLaunchArgument('gz_gui', default_value='true',
                              description='sim_only: run the gz 3D client (false = server-only GPU sensors, no 3D window)'),
        OpaqueFunction(function=launch_setup),
    ])
