"""TDD for lib/gzcli.py — the shared `gz service` CLI helpers (no ROS, no running Gazebo).

dynamic_obstacle and twin_resync both drive Gazebo through the `gz service` CLI; the request
composition + reply parsing live in ONE tested place. The runner is injectable so these tests
never need a sim.
"""
import math
import subprocess

import pytest

from algae_dt.lib import gzcli


class _Result:
    def __init__(self, stdout='', stderr=''):
        self.stdout = stdout
        self.stderr = stderr


def test_call_boolean_composes_the_gz_cli_invocation():
    seen = {}

    def runner(argv, **kw):
        seen['argv'] = argv
        seen['kw'] = kw
        return _Result(stdout='data: true')

    ok, diag = gzcli.call_boolean('/world/default/set_pose', 'gz.msgs.Pose',
                                  'name: "x"', timeout_ms=500, runner=runner)
    assert ok and diag == ''
    assert seen['argv'] == ['gz', 'service', '-s', '/world/default/set_pose',
                            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '500', '--req', 'name: "x"']
    assert seen['kw'].get('capture_output') is True and seen['kw'].get('text') is True
    # the subprocess timeout must outlast the gz --timeout (else we kill a still-working call)
    assert seen['kw'].get('timeout') >= 0.5


def test_call_boolean_false_reply_returns_diagnostic():
    ok, diag = gzcli.call_boolean('/s', 't', 'r',
                                  runner=lambda *a, **k: _Result(stdout='data: false'))
    assert not ok and 'data: false' in diag


def test_call_boolean_empty_reply_is_a_failure_with_diagnostic():
    ok, diag = gzcli.call_boolean('/s', 't', 'r',
                                  runner=lambda *a, **k: _Result(stdout='', stderr=''))
    assert not ok and diag != '', "a silent failure must still carry a diagnostic (RULES §C)"


def test_call_boolean_exception_never_raises():
    def runner(*a, **k):
        raise subprocess.TimeoutExpired(cmd='gz', timeout=1.0)
    ok, diag = gzcli.call_boolean('/s', 't', 'r', runner=runner)
    assert not ok and 'TimeoutExpired' in diag


def test_pose_req_formats_position_only():
    req = gzcli.pose_req('burger_sim', 1.25, -0.5, 0.01)
    assert req == 'name: "burger_sim", position {x: 1.25 y: -0.5 z: 0.01}'


def test_pose_req_with_yaw_appends_planar_orientation():
    req = gzcli.pose_req('burger_sim', 0.0, 0.0, 0.01, yaw=math.pi)
    assert req.startswith('name: "burger_sim", position {x: 0.0 y: 0.0 z: 0.01}, orientation {z: ')
    # yaw pi -> quaternion (z=1, w=0)
    assert 'z: 1.0' in req.split('orientation')[1]


def test_pose_req_rejects_non_finite_coordinates():
    for bad in (float('nan'), float('inf')):
        with pytest.raises(ValueError):
            gzcli.pose_req('burger_sim', bad, 0.0, 0.0)
        with pytest.raises(ValueError):
            gzcli.pose_req('burger_sim', 0.0, 0.0, 0.0, yaw=bad)


def test_pose_req_rejects_unsafe_entity_names():
    # quotes/braces/spaces would break the protobuf --req text: validate at the boundary
    for bad in ('a b', 'x"y', 'x{y}', ''):
        with pytest.raises(ValueError):
            gzcli.pose_req(bad, 0.0, 0.0, 0.0)


def test_safe_token_accepts_the_known_good_names():
    for good in ('default', 'burger_sim', 'algae_obstacle', 'a.b-c_1'):
        assert gzcli.safe_token(good, 'name') == good
