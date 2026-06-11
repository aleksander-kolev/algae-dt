"""Shared `gz service` CLI helpers (subprocess only — no ROS, no gz python deps).

The single place the gz request composition + Boolean-reply parsing live: dynamic_obstacle
(spawn + sweep teleport) and twin_resync (snap the sim onto the real robot) both call gz through
this. The runner is injectable so the logic is unit-tested without a running Gazebo
(test/test_gzcli.py); callers decide how loud a failure is (retry loop vs /dt/alerts).
"""
from __future__ import annotations

import math
import re
import subprocess

from algae_dt.lib.geometry import quaternion_from_yaw


def safe_token(value: str, label: str) -> str:
    """Reject gz entity/world names that would break the protobuf --req text (quotes/braces/
    whitespace): validate at the boundary, fail fast (RULES §C) instead of emitting a bad request."""
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', value or ''):
        raise ValueError(f"{label} must match [A-Za-z0-9_.-]+ (got {value!r})")
    return value


def pose_req(name: str, x: float, y: float, z: float, yaw: float | None = None) -> str:
    """gz.msgs.Pose proto-text for /world/*/set_pose: validated name + finite coordinates (a NaN
    would silently produce a garbage request), optional planar orientation from yaw."""
    safe_token(name, 'entity name')
    vals = (x, y, z) + ((yaw,) if yaw is not None else ())
    if not all(math.isfinite(v) for v in vals):
        raise ValueError(f"non-finite pose for {name!r}: x={x} y={y} z={z} yaw={yaw}")
    req = f'name: "{name}", position {{x: {x} y: {y} z: {z}}}'
    if yaw is not None:
        qz, qw = quaternion_from_yaw(yaw)
        req += f', orientation {{z: {qz} w: {qw}}}'
    return req


def call_boolean(service: str, reqtype: str, req: str, timeout_ms: int = 300,
                 runner=subprocess.run) -> tuple[bool, str]:
    """Call a gz Boolean-reply service; returns (ok, diagnostic). ok iff it replied `data: true`.
    Never raises: a missing sim / timeout / false reply comes back as (False, why)."""
    try:
        r = runner(['gz', 'service', '-s', service, '--reqtype', reqtype,
                    '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req],
                   check=False, capture_output=True, text=True,
                   timeout=max(1.0, timeout_ms / 1000.0 + 1.0))
        if 'data: true' in (r.stdout or ''):
            return True, ''
        return False, ((r.stderr or '').strip() or (r.stdout or '').strip()
                       or 'no/false reply (sim running?)')
    except Exception as exc:
        return False, repr(exc)
