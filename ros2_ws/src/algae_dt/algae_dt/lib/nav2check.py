"""Check AND REPAIR the stock Nav2 params file the launch feeds to nav2_bringup (no ROS).

WHY (safety-critical lab failure class): bringup.launch.py used nav2_common.RewrittenYaml to
reroute Nav2's final velocity onto the safety bus — but RewrittenYaml only replaces keys that
ALREADY EXIST; a missing key is a SILENT no-op, and the lab PC builds turtlebot3_navigation2 from
source at an unpinned version. Observed live (lab, 2026-06-11, robot #36): the lab's burger.yaml
has NO `use_sim_time` key anywhere — harmless in itself, but a one-tier "missing key = refuse"
gate blocked a perfectly safe demo at the door, while the worst case (a missing
collision_monitor.cmd_vel_out_topic leaving the robot UNGATED) is the one that must never pass.

So this module now FIXES instead of just flagging: repair_nav2_params() replaces every occurrence
of each rewrite key and ADDS the ones that are missing, with path-aware placement (use_sim_time on
every node, speed caps under the controller's FollowPath, `topic` on every laser observation
source, the chokepoint reroute on collision_monitor, the TwistStamped chain enforced). The launch
dumps the patched dict to a temp file and hands THAT to nav2_bringup — RewrittenYaml's
rewrite-only semantics are out of the safety path entirely. check_nav2_params() keeps only the
truly FATAL verdicts (no collision_monitor section / unparseable file / no amcl): a real mode
still refuses to start on those, because there is nowhere safe to put the chokepoint.

rewrites_for_mode() is the SINGLE source of truth for the per-mode rewrite set — the launch and
the lab_run.sh CLI gate (python3 -m algae_dt.lib.nav2check) both call it, so the two gates can
never disagree again (they did, observed the same lab day: CLI said OK, the launch refused).

Pure dict-in/dict-out (deep-copied, the input is never mutated); only __main__ touches files.
Tests: test/test_nav2check.py.
"""
from __future__ import annotations

import copy
import sys

# Every node in Nav2's Jazzy cmd_vel chain must run TwistStamped (the bus type contract, RULES
# §B-1): controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor
# -> (rerouted) /dt/cmd_vel_raw. Jazzy defaults enable_stamped_cmd_vel to FALSE, so the params
# must say true on each of these or commands are silently dropped by type-mismatched peers.
STAMPED_CHAIN_NODES = ('controller_server', 'velocity_smoother', 'behavior_server',
                       'collision_monitor')

BUS_TOPIC = '/dt/cmd_vel_raw'
NAV_SCAN_TOPIC = '/dt/scan_nav'


def rewrites_for_mode(mode: str, fake_robot: bool = False) -> dict:
    """The per-mode Nav2 overrides — the ONE list the launch applies and the CLI verifies.

    Rationale per key (full forensics: docs/BEST_APPROACHES.md §Lessons, lab 2026-06):
    - cmd_vel_out_topic -> the bus: Nav2's FINAL velocity must pass the 25 cm gate (chokepoint).
    - topic -> /dt/scan_nav: every obstacle source (4 costmap layers + collision_monitor source)
      reads the mediator's merged scan so virtual obstacles shape REAL planning; AMCL's
      scan_topic is a different key and stays on the bare /scan.
    - max_vel_x / max_speed_xy -> 0.22: stock plans 0.3 m/s, above the Burger wheel ceiling —
      saturation bent every fast arc tighter than the RViz plan on the real robot.
    - use_sim_time: true only in sim_only (D8).
    - source_timeout: sim rigs render the LiDAR at ~3 Hz (stock 0.2 s rejected every scan); the
      REAL robot gets the documented 1.0 s real-scan staleness budget (stock 0.2 s == the LDS-02
      period — Wi-Fi jitter kept expiring the scan and stuttering autonomy).
    - sim-only extras: AMCL auto-seed + a slow-render-proof progress checker.
    - both+fake (home rig): auto-seed + the throttled-host TF budget.
    """
    r = {
        'cmd_vel_out_topic': BUS_TOPIC,
        'topic': NAV_SCAN_TOPIC,
        'max_vel_x': 0.22,
        'max_speed_xy': 0.22,
        'use_sim_time': mode == 'sim_only',
        'enable_stamped_cmd_vel': True,
    }
    if mode == 'sim_only':
        r.update(set_initial_pose=True, source_timeout=2.0,
                 movement_time_allowance=30.0, required_movement_radius=0.1)
    elif mode == 'both' and fake_robot:
        r.update(set_initial_pose=True, source_timeout=2.0, transform_tolerance=2.0)
    else:                                    # the REAL robot: real_only / lab both
        r['source_timeout'] = 1.0
    return r


def _node_params(doc, node: str):
    sec = doc.get(node) if isinstance(doc, dict) else None
    if not isinstance(sec, dict):
        return None
    rp = sec.get('ros__parameters')
    return rp if isinstance(rp, dict) else None


def _key_exists(tree, key: str) -> bool:
    if isinstance(tree, dict):
        return key in tree or any(_key_exists(v, key) for v in tree.values())
    if isinstance(tree, list):
        return any(_key_exists(v, key) for v in tree)
    return False


def _replace_existing(tree, key: str, value) -> int:
    """Replace every existing occurrence of `key` anywhere in the tree; returns the count."""
    n = 0
    if isinstance(tree, dict):
        if key in tree:
            tree[key] = value
            n += 1
        for v in tree.values():
            n += _replace_existing(v, key, value)
    elif isinstance(tree, list):
        for v in tree:
            n += _replace_existing(v, key, value)
    return n


def _observation_source_dicts(tree):
    """Yield every named observation-source dict (costmap layers + collision_monitor declare
    their laser inputs as `observation_sources: scan` / `["scan"]` next to a `scan: {...}`)."""
    if isinstance(tree, dict):
        names = tree.get('observation_sources')
        if isinstance(names, str):
            names = names.split()
        if isinstance(names, list):
            for name in names:
                src = tree.get(name)
                if isinstance(src, dict):
                    yield src
        for v in tree.values():
            yield from _observation_source_dicts(v)
    elif isinstance(tree, list):
        for v in tree:
            yield from _observation_source_dicts(v)


def _add_missing(doc, key: str, value) -> int:
    """Path-aware ADD for a rewrite key that exists NOWHERE in the file; returns how many spots
    received it (0 = no safe place to put it -> the caller reports it as unfixable)."""
    if key == 'cmd_vel_out_topic':
        cm = _node_params(doc, 'collision_monitor')
        if cm is not None:
            cm[key] = value
            return 1
        return 0
    if key in ('topic', 'source_timeout'):
        n = 0
        for src in _observation_source_dicts(doc):
            src[key] = value
            n += 1
        if key == 'source_timeout':
            cm = _node_params(doc, 'collision_monitor')
            if cm is not None:
                cm[key] = value
                n += 1
        return n
    if key in ('max_vel_x', 'max_speed_xy'):
        rp = _node_params(doc, 'controller_server')
        fp = rp.get('FollowPath') if rp else None
        if isinstance(fp, dict):
            fp[key] = value
            return 1
        return 0
    if key == 'set_initial_pose':
        rp = _node_params(doc, 'amcl')
        if rp is not None:
            rp[key] = value
            return 1
        return 0
    if key in ('movement_time_allowance', 'required_movement_radius'):
        rp = _node_params(doc, 'controller_server')
        pc = rp.get('progress_checker') if rp else None
        if isinstance(pc, dict):
            pc[key] = value
            return 1
        return 0
    return 0


def repair_nav2_params(doc, rewrites: dict) -> tuple[dict, list[str], list[str]]:
    """Apply every rewrite as replace-or-ADD on a DEEP COPY; never mutates the input.

    Returns (patched_doc, applied, unfixable): `applied` is a human-readable action log,
    `unfixable` lists rewrite keys with no safe placement (caller warns with consequence()).
    The TwistStamped chain is additionally ENFORCED on every chain node that exists.
    """
    if not isinstance(doc, dict) or not doc:
        return {}, [], sorted(rewrites)
    new = copy.deepcopy(doc)
    applied: list[str] = []
    unfixable: list[str] = []
    for key, value in rewrites.items():
        replaced = _replace_existing(new, key, value)
        added = 0
        if key == 'use_sim_time':
            # Deterministic clock policy: every node section gets it, present or not.
            for name in list(new):
                rp = _node_params(new, name)
                if rp is not None and 'use_sim_time' not in rp:
                    rp['use_sim_time'] = value
                    added += 1
        elif key == 'enable_stamped_cmd_vel':
            for node in STAMPED_CHAIN_NODES:
                rp = _node_params(new, node)
                if rp is not None and 'enable_stamped_cmd_vel' not in rp:
                    rp['enable_stamped_cmd_vel'] = value
                    added += 1
        elif replaced == 0:
            added = _add_missing(new, key, value)
            if added == 0:
                unfixable.append(key)
                continue
        if replaced or added:
            applied.append(f"{key}={value} ({replaced} replaced, {added} added)")
    return new, applied, unfixable


def check_nav2_params(doc) -> list[str]:
    """FATAL problems only (empty = safe to drive) — run this on the REPAIRED doc.

    Fatal means "the robot would drive UNGATED or the command bus is broken, and repair had no
    safe place to fix it": a foreign/empty file, no collision_monitor section to host the
    chokepoint, an unstamped chain node (post-repair this only fires if the section is so odd
    the repair could not enforce it), or no amcl section at all.
    """
    if not isinstance(doc, dict) or not doc:
        return ["params file is empty/unparseable — Nav2 would run on pure defaults (plain-Twist "
                "cmd_vel straight to /cmd_vel, BYPASSING the 25 cm safety gate)"]
    problems: list[str] = []
    cm = _node_params(doc, 'collision_monitor')
    if cm is None:
        problems.append(
            "no `collision_monitor:` section — there is nowhere to host the safety chokepoint; "
            "Nav2 would publish /cmd_vel DIRECTLY, BYPASSING the 25 cm gate (ungated robot)")
    elif 'cmd_vel_out_topic' not in cm:
        problems.append(
            "collision_monitor has no `cmd_vel_out_topic` key — Nav2's final velocity cannot be "
            "rerouted onto /dt/cmd_vel_raw; the safety gate would be BYPASSED")
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
    return problems


# What is actually LOST when a rewrite key could not be placed (advisory; keeps the operator
# from either panicking or ignoring it):
_KEY_CONSEQUENCE = {
    'use_sim_time': "harmless in real modes (every node defaults to wall time) and sim time is "
                    "set by nav2_bringup's own use_sim_time argument anyway",
    'topic': "the virtual-obstacle feed is LOST — Nav2's obstacle sources stay on the raw /scan, "
             "so things placed in the twin will not shape real planning (gate still covers them)",
    'max_vel_x': "Nav2 may plan above the 0.22 m/s Burger ceiling; the mediator's curvature-"
                 "preserving clamp still caps the command (slower but on-arc)",
    'max_speed_xy': "Nav2 may plan above the 0.22 m/s Burger ceiling; the mediator's curvature-"
                    "preserving clamp still caps the command (slower but on-arc)",
    'source_timeout': "the stock collision_monitor scan timeout stays (0.2 s == the LDS-02 "
                      "period; Wi-Fi jitter may stutter autonomy)",
    'set_initial_pose': "sim_only loses the AMCL auto-seed (seed by hand / RViz)",
    'movement_time_allowance': "the stock progress checker stays (slow-render aborts possible)",
    'required_movement_radius': "the stock progress checker stays (slow-render aborts possible)",
    'transform_tolerance': "the stock TF freshness budget stays (throttled-host aborts possible)",
}


def consequence(key: str) -> str:
    return _KEY_CONSEQUENCE.get(key, "that launch override is skipped (stock value stays)")


def main(argv=None) -> int:
    """CLI for the lab preflight: python3 -m algae_dt.lib.nav2check <params.yaml> [--mode MODE].

    Runs the SAME repair the launch will run and reports what it would fix. Exit 0 = launchable
    (possibly with named degradations); exit 6 = FATAL in a real mode (both/real_only) — there is
    no safe place for the chokepoint, scripts/lab_run.sh aborts rather than drive ungated.
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
            doc = yaml.safe_load(f)
    except OSError as exc:
        print(f"nav2check PROBLEM: cannot read {argv[0]}: {exc}", file=sys.stderr)
        print("nav2check FATAL: cannot verify the safety chokepoint at all.", file=sys.stderr)
        return 6 if mode in ('both', 'real_only') else 0
    patched, applied, unfixable = repair_nav2_params(doc, rewrites_for_mode(mode))
    for line in applied:
        print(f"nav2check repair: {line}")
    for k in unfixable:
        print(f"nav2check WARNING: no place for `{k}` — {consequence(k)}; continuing.",
              file=sys.stderr)
    problems = check_nav2_params(patched)
    if not problems:
        print(f"nav2check OK: {argv[0]} hosts the safety chokepoint "
              f"(launch will apply the same {len(applied)} repair(s))")
        return 0
    for p in problems:
        print(f"nav2check PROBLEM: {p}", file=sys.stderr)
    if mode in ('both', 'real_only'):
        print("nav2check FATAL: refusing a REAL mode — even after repair this params file "
              "cannot host the safety chokepoint (the robot would drive ungated). Update/"
              "rebuild turtlebot3_navigation2 (jazzy branch), then re-run.", file=sys.stderr)
        return 6
    print("nav2check WARNING: sim_only continues (no real robot at risk).", file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
