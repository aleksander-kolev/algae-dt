"""Pure real<->sim synchronisation math (no ROS). Implemented TDD per PLAN T3.1 (Rubric ②).

Computes the *measured* discrepancy the sync_supervisor publishes/logs and compares it against the
documented thresholds in config/twin.yaml. Also provides the unicycle integrator for the sim_only
"commanded shadow pose" (so pillar ② is demonstrable without a real robot). Tests: test/test_sync.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from algae_dt.lib.geometry import angle_diff, euclidean

INF = float('inf')


@dataclass(frozen=True)
class SyncError:
    dxy: float       # XY pose discrepancy (m)
    dyaw: float      # heading discrepancy (rad, absolute, shortest-arc)
    sensor: float    # front-range discrepancy (m); +inf if only one world sees an obstacle


def pose_error(real_xy_yaw, sim_xy_yaw) -> tuple[float, float]:
    """(dxy, dyaw) between two (x, y, yaw) poses; dyaw uses the shortest signed arc."""
    dxy = euclidean(real_xy_yaw[0], real_xy_yaw[1], sim_xy_yaw[0], sim_xy_yaw[1])
    dyaw = abs(angle_diff(real_xy_yaw[2], sim_xy_yaw[2]))
    return dxy, dyaw


def sensor_error(real_front: float, sim_front: float) -> float:
    """Front-range discrepancy. Both worlds seeing nothing (+inf) agree (0.0); exactly one seeing
    an obstacle is maximal disagreement (+inf) so it trips the tolerance check."""
    if real_front == INF and sim_front == INF:
        return 0.0
    if real_front == INF or sim_front == INF:
        return INF
    return abs(real_front - sim_front)


def compute(real_pose, sim_pose, real_front: float, sim_front: float) -> SyncError:
    dxy, dyaw = pose_error(real_pose, sim_pose)
    return SyncError(dxy, dyaw, sensor_error(real_front, sim_front))


def within_tolerance(err: SyncError, tol_pose_xy: float,
                     tol_pose_yaw: float, tol_sensor_range: float) -> bool:
    """True iff every component is at-or-within its documented threshold (twin.yaml)."""
    return (err.dxy <= tol_pose_xy
            and err.dyaw <= tol_pose_yaw
            and err.sensor <= tol_sensor_range)


def classify(err: SyncError, tol_pose_xy: float,
             tol_pose_yaw: float, tol_sensor_range: float) -> str:
    """'ok' when in tolerance, else 'warn' (drives /dt/sync_ok and /dt/alerts)."""
    return 'ok' if within_tolerance(err, tol_pose_xy, tol_pose_yaw, tol_sensor_range) else 'warn'


def integrate_unicycle(x: float, y: float, yaw: float,
                       v: float, omega: float, dt: float) -> tuple[float, float, float]:
    """Exact constant-(v, omega) unicycle step over dt; yaw wrapped to (-pi, pi].

    Used for the sim_only commanded shadow pose (integrate /dt/cmd_vel_raw): the "real" pose
    against which the achieved sim pose is compared when no physical robot is present.
    """
    if abs(omega) < 1e-9:
        x2 = x + v * math.cos(yaw) * dt
        y2 = y + v * math.sin(yaw) * dt
        yaw2 = yaw
    else:
        yaw2 = yaw + omega * dt
        x2 = x + (v / omega) * (math.sin(yaw2) - math.sin(yaw))
        y2 = y - (v / omega) * (math.cos(yaw2) - math.cos(yaw))
    yaw2 = (yaw2 + math.pi) % (2.0 * math.pi) - math.pi
    return x2, y2, yaw2
