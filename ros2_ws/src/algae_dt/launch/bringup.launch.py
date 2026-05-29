"""bringup.launch.py — orchestrates the DEFAULT turtlebot3 stack + our algae_dt DT layer.

Modes (RULES/SETUP): sim_only (default, home dev), real_only, both.
This is the combined-launch convenience path; the TA-familiar multi-terminal fallback is documented
in docs/BEST_APPROACHES.md. Items still being wired are marked TODO with PLAN task IDs.

    ros2 launch algae_dt bringup.launch.py mode:=sim_only

cmd_vel TYPE CONTRACT (see RULES §B, Jazzy doc-confirmed): /dt/cmd_vel_raw is geometry_msgs/
TwistStamped (real turtlebot3_node + turtlebot3_teleop use TwistStamped on Jazzy). Nav2 on Jazzy
defaults to plain Twist, so we (a) set enable_stamped_cmd_vel:true on Nav2 via a params_file and
(b) remap Nav2's controller cmd_vel onto /dt/cmd_vel_raw (below). The mediator forwards TwistStamped
to the real /cmd_vel and the runtime-verified type to /sim/cmd_vel.
"""
import os

from ament_index_python.packages import (PackageNotFoundError,
                                          get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def _share(pkg: str) -> str:
    """Resolve a stock-package share dir with a clear 'source the turtlebot3 stack' error."""
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError as exc:  # pragma: no cover - launch-time guard
        raise RuntimeError(
            f"Required runtime package '{pkg}' not found. Source the turtlebot3 stack / run inside "
            f"the turtlebot3_ws container (see docs/SETUP.md §0/§2)."
        ) from exc


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('algae_dt')
    params = os.path.join(pkg, 'config', 'twin.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'map.yaml')

    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('use_rviz')

    # Node parameters need a real BOOL (a string "true" won't engage rclpy's use_sim_time).
    use_sim_time = PythonExpression(["'sim_only' == '", mode, "'"])            # -> Python True/False
    # Launch-include args expect a lowercase string, so build that separately.
    use_sim_time_str = PythonExpression(["'true' if '", mode, "' == 'sim_only' else 'false'"])

    # --- our DT layer (all modes); params bind via the /** wildcard in twin.yaml ---
    common = [{'use_sim_time': use_sim_time}, params, {'mode': mode}]
    dt_nodes = GroupAction([
        Node(package='algae_dt', executable='twin_mediator', output='screen', parameters=common),
        Node(package='algae_dt', executable='sync_supervisor', output='screen', parameters=common),
        # NEVER set name= on mission_runner (process-wide remap trap, BEST_APPROACHES). It does NOT
        # publish cmd_vel itself (it's a BasicNavigator action client), so it carries no cmd_vel remap.
        Node(package='algae_dt', executable='mission_runner', output='screen', parameters=common),
        Node(package='algae_dt', executable='operator_gui', output='screen', parameters=common),
    ])

    # --- stock Gazebo sim (sim_only/both) with our arena world — PLAN T1.1 ---
    # TODO PLAN T1.1: replace with the verified turtlebot3_gazebo gz bring-up + spawn into
    # worlds/algae_arena.world (model + robot_state_publisher + ros_gz bridge). Arg names confirmed
    # empirically in the container before this is enabled; until then use the multi-terminal fallback.
    # ALSO T1.1: pass Nav2 a `params_file` with `set_initial_pose: true` + the spawn pose so AMCL
    # auto-localizes in headless sim_only (no human 2D Pose Estimate) — else goals plan unlocalized.
    # In `both` (PLAN T5.1) push the sim to /sim/* and keep sim TF off the global /tf.

    # --- stock Nav2 (+AMCL+map). Remap the controller's cmd_vel onto our pre-safety bus so EVERY
    #     autonomous command passes through the mediator's safety gate (single-chokepoint).
    #     SetRemap inside the group rewrites /cmd_vel for the included Nav2 nodes.
    # TODO PLAN T2.2: pass params_file=<turtlebot3 Nav2 params + enable_stamped_cmd_vel:true on
    #     controller_server/behavior_server/velocity_smoother + (sim_only) set_initial_pose to the
    #     spawn pose>. Build it from turtlebot3_navigation2's bundled params via nav2_common
    #     RewrittenYaml (param_rewrites) so the tuned params are kept — see docs/RULES §B-2. Until
    #     then Nav2 publishes plain Twist (Jazzy default) and autonomous motion won't reach the
    #     TwistStamped bus; teleop/GUI (TwistStamped) still work. ---
    nav2 = GroupAction([
        SetRemap('/cmd_vel', '/dt/cmd_vel_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                _share('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time_str, 'map': map_yaml}.items(),
        ),
    ])

    rviz = Node(package='rviz2', executable='rviz2', output='screen', condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim_only',
                              description='sim_only | real_only | both'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # NOTE: sim bring-up (sim_needed) is gated above and pending T1.1; nav2 + dt_nodes run now.
        nav2,
        dt_nodes,
        rviz,
    ])
