"""Fail-loud verification of the stock Nav2 params file the launch rewrites (no ROS).

WHY THIS EXISTS (safety-critical lab failure class): bringup.launch.py routes Nav2's FINAL
velocity through the mediator's 25 cm gate by rewriting `collision_monitor.cmd_vel_out_topic ->
/dt/cmd_vel_raw` via nav2_common.RewrittenYaml — and RewrittenYaml `param_rewrites` only replace
keys that ALREADY EXIST in the source file; a missing key is a SILENT no-op. The home image's
burger.yaml (apt ros-jazzy-turtlebot3 2.3.6) carries every key we rewrite, but the lab PC builds
turtlebot3_navigation2 FROM SOURCE at an unpinned version: if its burger.yaml lacked the
collision_monitor section, Nav2 would publish /cmd_vel DIRECTLY and the real robot would drive
UNGATED — with no error anywhere. The launch calls check_nav2_params() and REFUSES to start real
modes on problems; scripts/lab_run.sh runs the same check early (python3 -m algae_dt.lib.nav2check).

Pure dict-in, problems-out (tests: test/test_nav2check.py); only __main__ touches files.
"""
from __future__ import annotations

import sys

# Every node in Nav2's Jazzy cmd_vel chain must run TwistStamped (the bus type contract, RULES
# §B-1): controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor
# -> (rewritten) /dt/cmd_vel_raw. Jazzy defaults enable_stamped_cmd_vel to FALSE, so the params
# file must say true on each of these or commands are silently dropped by type-mismatched peers.
STAMPED_CHAIN_NODES = ('controller_server', 'velocity_smoother', 'behavior_server',
                       'collision_monitor')


def _node_params(doc: dict, node: str):
    sec = doc.get(node)
    if not isinstance(sec, dict):
        return None
    rp = sec.get('ros__parameters')
    return rp if isinstance(rp, dict) else None


def _key_exists(tree, key: str) -> bool:
    """True if `key` appears as a mapping key anywhere in the (nested) yaml tree."""
    if isinstance(tree, dict):
        return key in tree or any(_key_exists(v, key) for v in tree.values())
    if isinstance(tree, list):
        return any(_key_exists(v, key) for v in tree)
    return False


def check_nav2_params(doc, rewrite_keys=()) -> list[str]:
    """Return human-readable problems (empty list = safe), each stating WHY it matters."""
    if not isinstance(doc, dict) or not doc:
        return ["params file is empty/unparseable — Nav2 would run on pure defaults (plain-Twist "
                "cmd_vel straight to /cmd_vel, BYPASSING the 25 cm safety gate)"]
    problems: list[str] = []
    cm = _node_params(doc, 'collision_monitor')
    if cm is None:
        problems.append(
            "no `collision_monitor:` section — the cmd_vel_out_topic rewrite silently no-ops and "
            "Nav2 publishes /cmd_vel DIRECTLY, BYPASSING the 25 cm safety gate (ungated robot)")
    elif 'cmd_vel_out_topic' not in cm:
        problems.append(
            "collision_monitor has no `cmd_vel_out_topic` key — RewrittenYaml cannot reroute "
            "Nav2's final velocity onto /dt/cmd_vel_raw; the safety gate would be BYPASSED")
    for node in STAMPED_CHAIN_NODES:
        rp = _node_params(doc, node)
        if rp is None:
            problems.append(
                f"no `{node}:` section — cannot confirm enable_stamped_cmd_vel; the TwistStamped "
                "bus contract (RULES §B-1) may be broken (commands silently dropped)")
        elif rp.get('enable_stamped_cmd_vel') is not True:
            problems.append(
                f"{node}: enable_stamped_cmd_vel is not true — on Jazzy it then talks plain Twist "
                "and the TwistStamped bus/robot silently ignores it (robot activates, never moves)")
    if _node_params(doc, 'amcl') is None:
        problems.append("no `amcl:` section — localization params missing (wrong/foreign file?)")
    for key in rewrite_keys:
        if not _key_exists(doc, key):
            problems.append(f"rewrite key `{key}` not present anywhere — the RewrittenYaml "
                            f"rewrite of `{key}` will SILENTLY no-op")
    return problems


def main(argv=None) -> int:
    """CLI for the lab preflight: python3 -m algae_dt.lib.nav2check <params.yaml> [--mode MODE].

    Exit 0 = OK (or warn-only in sim_only); exit 6 = FATAL problems in a real mode (both/
    real_only) — scripts/lab_run.sh aborts on it rather than launch an ungated robot.
    """
    import yaml
    argv = list(sys.argv[1:] if argv is None else argv)
    mode = 'both'
    if '--mode' in argv:
        i = argv.index('--mode')
        mode = argv[i + 1]
        del argv[i:i + 2]
    if len(argv) != 1:
        print("usage: python3 -m algae_dt.lib.nav2check <nav2-params.yaml> [--mode MODE]",
              file=sys.stderr)
        return 2
    try:
        with open(argv[0], 'r', encoding='utf-8') as f:
            problems = check_nav2_params(yaml.safe_load(f))
    except OSError as exc:
        problems = [f"cannot read {argv[0]}: {exc}"]
    if not problems:
        print(f"nav2check OK: {argv[0]} carries the safety-gate chokepoint keys")
        return 0
    for p in problems:
        print(f"nav2check PROBLEM: {p}", file=sys.stderr)
    if mode in ('both', 'real_only'):
        print("nav2check FATAL: refusing a REAL mode with a compromised Nav2 params file (the "
              "robot would drive ungated). Update/rebuild turtlebot3_navigation2 (jazzy branch) "
              "or repair the file, then re-run.", file=sys.stderr)
        return 6
    print("nav2check WARNING: sim_only continues (no real robot at risk).", file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
