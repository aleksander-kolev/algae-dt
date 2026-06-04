"""Tiny ROS-side helpers shared by every algae_dt node (rclpy/ament only, no business logic).

The pure libs stay ROS-free; this module is the one place node boilerplate is allowed to live so
it is not copy-pasted six times (the declare+get parameter idiom and the package-map loader were
duplicated verbatim across the nodes before).
"""
from __future__ import annotations

import os


def declare_get(node, name: str, default):
    """declare_parameter + get_parameter in one call — the standard twin.yaml-backed read.
    (Was an identical private `_declare` method in all 6 nodes.)"""
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def package_map_path(package: str = 'algae_dt', name: str = 'map.pgm') -> str:
    """Absolute path of an installed map asset (share/<pkg>/maps/<name>)."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), 'maps', name)


def load_package_map(package: str = 'algae_dt'):
    """Parse the installed maps/map.pgm with the in-tree parser -> lib.pgm.Pgm.
    (The path-build + open + parse step was duplicated in operator_gui and mission_runner.)"""
    from algae_dt.lib import pgm
    with open(package_map_path(package), 'rb') as f:
        return pgm.parse(f.read())


def load_map_yaml(package: str = 'algae_dt') -> dict:
    """Parse the installed maps/map.yaml (map_server metadata: resolution/origin/negate/
    free_thresh/...). Returns {} when absent so callers can fall back to defaults LOUDLY."""
    import yaml
    path = package_map_path(package, 'map.yaml')
    if not os.path.isfile(path):
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}
