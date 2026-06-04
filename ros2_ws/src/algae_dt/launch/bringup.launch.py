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


def _gz_sim(gz_args: str, critical: bool = True):
    """gz sim include. critical=False (the 3D GUI client) must NOT take the launch down with it:
    the client's large OGRE2 VBO is documented to crash on weak GL stacks (WSL d3d12, some lab
    GPUs) — with on_exit_shutdown the whole graded demo (Nav2, mediator, GUI, sim server) died
    with it. The SERVER stays critical: without it there is no sim at all."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('ros_gz_sim', 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args,
                          'on_exit_shutdown': 'true' if critical else 'false'}.items())


# Robot spawn pose — single source of truth. AMCL's default initial_pose is (0,0,0); sim_only's
# set_initial_pose:'True' rewrite seeds it THERE, so the spawn must stay at the origin (or AMCL
# seeding must be updated in lockstep). This constant is the explicit contract for that coupling.
_SPAWN_X, _SPAWN_Y = '0.0', '0.0'


def _sim_bringup(pkg, world, headless, gz_gui):
    """sim_only: stock bare-topic Gazebo robot (model + RSP + bare ros_gz bridge).
    gz_gui=False runs the server only (it still GPU-renders the sensors via the display context) and
    skips the heavy gz 3D client — useful where that client can't init (e.g. OGRE2 on the WSL d3d12 GL,
    which refuses its large VBO); drive/visualise via operator_gui + RViz instead."""
    actions = [_gz_sim('-r -s -v2 ' + ('--headless-rendering ' if headless else '') + world)]
    if not headless and gz_gui:
        actions.append(_gz_sim('-g -v2 ', critical=False))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('turtlebot3_gazebo', 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_src('turtlebot3_gazebo', 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': _SPAWN_X, 'y_pose': _SPAWN_Y}.items()))
    return actions


def _sim_mirror(pkg, world, headless, gz_gui):
    """both: the Gazebo mirror pushed entirely onto /sim/* (custom bridge + namespaced RSP).
    gz_gui works exactly as in sim_only — `both` is the PRIMARY graded lab demo and needs the same
    escape from the crash-prone gz 3D client (it previously had none short of headless, which also
    killed the operator GUI)."""
    model_sdf = _src('turtlebot3_gazebo', 'models', 'turtlebot3_burger', 'model.sdf')
    urdf = _src('turtlebot3_gazebo', 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    actions = [_gz_sim('-r -s -v2 ' + ('--headless-rendering ' if headless else '') + world)]
    if not headless and gz_gui:
        actions.append(_gz_sim('-g -v2 ', critical=False))
    actions += [
        # The sim RSP must be FULLY isolated from the real robot (topic-collision rule, RULES §B-3):
        #  * /tf,/tf_static -> /sim/* (sim TF off the global tree),
        #  * /robot_description -> /sim/robot_description (the real RSP latches the bare one),
        #  * /joint_states -> /sim/joint_states (else it consumed the REAL robot's wheel angles).
        # No frame_prefix: the gz bridge's /sim/tf carries BARE frame names (odom->base_footprint),
        # so a 'sim/'-prefixed RSP tree never connected to it — bare names on namespaced TOPICS are
        # both internally consistent and collision-free.
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
             remappings=[('/tf', '/sim/tf'), ('/tf_static', '/sim/tf_static'),
                         ('/robot_description', '/sim/robot_description'),
                         ('/joint_states', '/sim/joint_states')]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'burger_sim', '-file', model_sdf,
                        '-x', _SPAWN_X, '-y', _SPAWN_Y, '-z', '0.01']),
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
    use_fake_robot = LaunchConfiguration('use_fake_robot').perform(context).lower() == 'true'
    gz_gui = LaunchConfiguration('gz_gui').perform(context).lower() == 'true'
    use_dynamic_obstacle = LaunchConfiguration('use_dynamic_obstacle').perform(context).lower() == 'true'

    # RViz default is MODE-AWARE ('auto'): real_only/both REQUIRE the operator to seed AMCL with
    # RViz's "2D Pose Estimate" (there is no other /initialpose source in the stack), so RViz must
    # actually open there — every documented lab flow assumed it did while the old default ('false')
    # never opened it, stranding AMCL at the map origin. sim_only auto-seeds (set_initial_pose) and
    # keeps RViz off unless asked.
    use_rviz_cfg = LaunchConfiguration('use_rviz').perform(context).lower()
    use_rviz = (use_rviz_cfg == 'true'
                or (use_rviz_cfg == 'auto' and mode in ('real_only', 'both') and not headless))
    if mode in ('real_only', 'both') and not use_rviz:
        # headless:=true (or use_rviz:=false) in a REAL mode removes the stack's ONLY /initialpose
        # source: AMCL stays seeded at the map origin and every Nav2 goal plans from a wrong pose.
        # Allowed (an operator may publish /initialpose by hand) but it must never be silent.
        print(f"[bringup.launch] WARNING: mode:={mode} without RViz — AMCL has NO initial-pose "
              "source (sim_only auto-seeds; real modes are seeded by RViz '2D Pose Estimate'). "
              "Publish /initialpose manually or relaunch with use_rviz:=true, else the robot is "
              "PERMANENTLY MISLOCALIZED at the map origin.", flush=True)

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
        actions += _sim_mirror(pkg, world, headless, gz_gui)

    # Pillar-③ "live environment change" tool, now launchable (it previously existed only as a
    # bare `ros2 run` buried in the scenario docs — the demo script had no wired way to trigger
    # it). Needs a Gazebo world, so sim_only/both only. use_sim_time matches the launch-wide rule
    # so the sweep period counts SIM seconds in sim_only (wall-clock sweeps ran visibly fast under
    # a throttled real-time factor).
    if use_dynamic_obstacle and mode in ('sim_only', 'both'):
        actions.append(Node(package='algae_dt', executable='dynamic_obstacle', output='screen',
                            parameters=[params, {'use_sim_time': use_sim_time}]))

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
        # Headless auto-seed at the spawn (no human 2D pose). EXPLICIT CONTRACT: AMCL's default
        # initial_pose is (0,0,0) and the robot spawns at (_SPAWN_X,_SPAWN_Y,yaw 0) — these
        # coincide BY DESIGN. Changing the spawn pose requires also rewriting amcl's
        # initial_pose.{x,y,yaw} here, or sim_only mis-localizes from t=0.
        rewrites['set_initial_pose'] = 'True'
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
        DeclareLaunchArgument('use_rviz', default_value='auto',
                              description="auto => ON in real_only/both (AMCL needs the RViz 2D Pose "
                                          "Estimate there), OFF in sim_only; or force true/false"),
        DeclareLaunchArgument('use_fake_robot', default_value='false',
                              description='real_only/both: spawn the kinematic fake_robot (no hardware)'),
        DeclareLaunchArgument('gz_gui', default_value='true',
                              description='sim_only/both: run the gz 3D client (false = server-only '
                                          'GPU sensors, no 3D window; a client crash never kills the launch)'),
        DeclareLaunchArgument('use_dynamic_obstacle', default_value='false',
                              description='sim_only/both: sweep the moving obstacle box (pillar-III live change)'),
        OpaqueFunction(function=launch_setup),
    ])
