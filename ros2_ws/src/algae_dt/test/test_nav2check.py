"""TDD for lib/nav2check.py — CHECK AND REPAIR the stock Nav2 params (the safety chokepoint).

REGRESSION CLASS (lab): the launch used to reroute Nav2's final velocity through the safety gate
with nav2_common.RewrittenYaml — which only replaces keys that ALREADY EXIST; a missing key is a
SILENT no-op. The lab PC builds turtlebot3_navigation2 from source at an unpinned version, and
its burger.yaml turned out to have NO `use_sim_time` key anywhere (observed live, 2026-06-11,
robot #36): a one-tier "missing key = refuse" gate blocked a perfectly safe demo at the door,
while the CLI gate and the launch disagreed about it. The module now REPAIRS: every override is
replaced-or-ADDED with path-aware placement, both gates run the identical per-mode set
(rewrites_for_mode), and a real mode refuses only when there is genuinely no safe place for the
chokepoint (no collision_monitor / no amcl / unparseable file).
"""
import copy

from algae_dt.lib import nav2check

# The shape that matters from turtlebot3_navigation2/param/burger.yaml. Like the LAB's file (and
# unlike the apt one), it has NO use_sim_time key anywhere — the live 2026-06-11 trip-up.
LABLIKE = {
    'amcl': {'ros__parameters': {'set_initial_pose': False, 'scan_topic': 'scan'}},
    'controller_server': {'ros__parameters': {
        'enable_stamped_cmd_vel': True,
        'FollowPath': {'max_vel_x': 0.3, 'max_speed_xy': 0.3, 'max_vel_theta': 1.0},
        'progress_checker': {'required_movement_radius': 0.5, 'movement_time_allowance': 10.0},
    }},
    'velocity_smoother': {'ros__parameters': {'enable_stamped_cmd_vel': True}},
    'behavior_server': {'ros__parameters': {'enable_stamped_cmd_vel': True}},
    'collision_monitor': {'ros__parameters': {
        'enable_stamped_cmd_vel': True,
        'cmd_vel_in_topic': 'cmd_vel_smoothed',
        'cmd_vel_out_topic': 'cmd_vel',
        'source_timeout': 5.0,
        'observation_sources': ['scan'],
        'scan': {'type': 'scan', 'topic': '/scan', 'source_timeout': 0.2},
    }},
    'local_costmap': {'local_costmap': {'ros__parameters': {'obstacle_layer': {
        'observation_sources': 'scan',                  # the STRING form some layers use
        'scan': {'topic': '/scan', 'clearing': True},
    }}}},
}


# ------------------------------ rewrites_for_mode ------------------------------

def test_rewrites_for_mode_carries_the_chokepoint_everywhere():
    for mode, fake in (('sim_only', False), ('real_only', False), ('both', False), ('both', True)):
        r = nav2check.rewrites_for_mode(mode, fake_robot=fake)
        assert r['cmd_vel_out_topic'] == '/dt/cmd_vel_raw'
        assert r['topic'] == '/dt/scan_nav'
        assert r['max_vel_x'] == 0.22 and r['max_speed_xy'] == 0.22
        assert r['enable_stamped_cmd_vel'] is True


def test_rewrites_for_mode_clock_and_timeouts():
    assert nav2check.rewrites_for_mode('sim_only')['use_sim_time'] is True
    assert nav2check.rewrites_for_mode('both')['use_sim_time'] is False
    assert nav2check.rewrites_for_mode('both')['source_timeout'] == 1.0       # real-scan budget
    assert nav2check.rewrites_for_mode('real_only')['source_timeout'] == 1.0
    assert nav2check.rewrites_for_mode('sim_only')['source_timeout'] == 2.0   # slow-render budget
    assert nav2check.rewrites_for_mode('both', fake_robot=True)['transform_tolerance'] == 2.0
    assert 'set_initial_pose' not in nav2check.rewrites_for_mode('both')      # operator seeds AMCL


# ----------------------------------- repair -----------------------------------

def _repair(doc, mode='both', fake=False):
    return nav2check.repair_nav2_params(doc, nav2check.rewrites_for_mode(mode, fake_robot=fake))


def test_repair_lab_file_2026_06_11_no_use_sim_time_anywhere():
    """THE live lab regression: a file with no use_sim_time key must be fully repairable —
    the key is ADDED to every node section and nothing is unfixable."""
    patched, applied, unfixable = _repair(LABLIKE, 'both')
    assert unfixable == []
    assert any('use_sim_time' in a for a in applied)
    for node in ('amcl', 'controller_server', 'velocity_smoother', 'behavior_server',
                 'collision_monitor'):
        assert patched[node]['ros__parameters']['use_sim_time'] is False
    assert nav2check.check_nav2_params(patched) == []


def test_repair_never_mutates_the_input():
    before = copy.deepcopy(LABLIKE)
    _repair(LABLIKE, 'both')
    assert LABLIKE == before


def test_repair_reroutes_the_chokepoint_and_caps_the_plan():
    patched, _, _ = _repair(LABLIKE, 'both')
    cm = patched['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_out_topic'] == '/dt/cmd_vel_raw'
    fp = patched['controller_server']['ros__parameters']['FollowPath']
    assert fp['max_vel_x'] == 0.22 and fp['max_speed_xy'] == 0.22
    assert fp['max_vel_theta'] == 1.0                      # untouched keys stay untouched


def test_repair_points_every_observation_source_at_scan_nav():
    patched, _, _ = _repair(LABLIKE, 'both')
    assert patched['collision_monitor']['ros__parameters']['scan']['topic'] == '/dt/scan_nav'
    layer = patched['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
    assert layer['scan']['topic'] == '/dt/scan_nav'        # the STRING observation_sources form
    assert patched['amcl']['ros__parameters']['scan_topic'] == 'scan', \
        "AMCL's scan_topic is a different key and must NEVER be redirected"


def test_repair_adds_topic_when_no_source_declares_one():
    doc = copy.deepcopy(LABLIKE)
    del doc['collision_monitor']['ros__parameters']['scan']['topic']
    del doc['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic']
    patched, applied, unfixable = _repair(doc, 'both')
    assert 'topic' not in unfixable
    assert patched['collision_monitor']['ros__parameters']['scan']['topic'] == '/dt/scan_nav'
    assert (patched['local_costmap']['local_costmap']['ros__parameters']
            ['obstacle_layer']['scan']['topic'] == '/dt/scan_nav')


def test_repair_enforces_the_stamped_chain():
    doc = copy.deepcopy(LABLIKE)
    doc['velocity_smoother']['ros__parameters']['enable_stamped_cmd_vel'] = False
    del doc['behavior_server']['ros__parameters']['enable_stamped_cmd_vel']
    patched, _, _ = _repair(doc, 'both')
    assert patched['velocity_smoother']['ros__parameters']['enable_stamped_cmd_vel'] is True
    assert patched['behavior_server']['ros__parameters']['enable_stamped_cmd_vel'] is True
    assert nav2check.check_nav2_params(patched) == []


def test_repair_adds_missing_speed_caps_under_followpath():
    doc = copy.deepcopy(LABLIKE)
    del doc['controller_server']['ros__parameters']['FollowPath']['max_vel_x']
    patched, _, unfixable = _repair(doc, 'both')
    assert 'max_vel_x' not in unfixable
    assert patched['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.22


def test_repair_sim_only_seeds_amcl_and_loosens_the_progress_checker():
    patched, _, unfixable = _repair(LABLIKE, 'sim_only')
    assert unfixable == []
    assert patched['amcl']['ros__parameters']['set_initial_pose'] is True
    pc = patched['controller_server']['ros__parameters']['progress_checker']
    assert pc['movement_time_allowance'] == 30.0 and pc['required_movement_radius'] == 0.1


def test_repair_without_collision_monitor_is_unfixable_and_fatal():
    doc = copy.deepcopy(LABLIKE)
    del doc['collision_monitor']
    patched, _, unfixable = _repair(doc, 'both')
    assert 'cmd_vel_out_topic' in unfixable
    problems = nav2check.check_nav2_params(patched)
    joined = ' '.join(problems).lower()
    assert 'collision_monitor' in joined and ('bypass' in joined or 'gate' in joined)


def test_repair_empty_doc_reports_everything_unfixable():
    patched, applied, unfixable = _repair({}, 'both')
    assert patched == {} and applied == [] and len(unfixable) > 0
    assert nav2check.check_nav2_params(patched) != []
    assert nav2check.check_nav2_params(None) != []   # yaml.safe_load of an empty file is None


# ------------------------------- fatal-only check -------------------------------

def test_check_flags_an_unstamped_chain_on_an_unrepaired_doc():
    doc = copy.deepcopy(LABLIKE)
    doc['controller_server']['ros__parameters']['enable_stamped_cmd_vel'] = False
    problems = nav2check.check_nav2_params(doc)
    assert any('controller_server' in p and 'enable_stamped_cmd_vel' in p for p in problems)


def test_check_flags_missing_amcl():
    doc = copy.deepcopy(LABLIKE)
    del doc['amcl']
    assert any('amcl' in p for p in nav2check.check_nav2_params(doc))


def test_consequence_has_specific_text_and_a_default():
    assert 'costmap' not in nav2check.consequence('use_sim_time')   # the harmless one says so
    assert 'harmless' in nav2check.consequence('use_sim_time')
    assert '/scan' in nav2check.consequence('topic')
    assert nav2check.consequence('totally_unknown_key')             # generic fallback exists


# ------------------------------- CLI (the lab_run.sh gate) -------------------------------

def _write_yaml(tmp_path, doc):
    import yaml
    p = tmp_path / 'params.yaml'
    p.write_text(yaml.safe_dump(doc), encoding='utf-8')
    return str(p)


def test_cli_lab_file_2026_06_11_launches_with_repairs(tmp_path, capsys):
    """The exact lab blocker must now exit 0, reporting the use_sim_time repair."""
    assert nav2check.main([_write_yaml(tmp_path, LABLIKE), '--mode', 'both']) == 0
    out = capsys.readouterr()
    assert 'use_sim_time' in out.out and 'repair' in out.out
    assert 'OK' in out.out


def test_cli_real_mode_fails_6_only_when_the_chokepoint_has_no_home(tmp_path, capsys):
    doc = copy.deepcopy(LABLIKE)
    del doc['collision_monitor']
    assert nav2check.main([_write_yaml(tmp_path, doc), '--mode', 'both']) == 6
    assert 'collision_monitor' in capsys.readouterr().err


def test_cli_sim_only_warns_but_exits_zero(tmp_path, capsys):
    doc = copy.deepcopy(LABLIKE)
    del doc['collision_monitor']
    assert nav2check.main([_write_yaml(tmp_path, doc), '--mode', 'sim_only']) == 0
    assert 'collision_monitor' in capsys.readouterr().err


def test_cli_unreadable_file_is_fatal_in_real_modes(tmp_path):
    assert nav2check.main([str(tmp_path / 'nope.yaml'), '--mode', 'both']) == 6
    assert nav2check.main([str(tmp_path / 'nope.yaml'), '--mode', 'sim_only']) == 0
