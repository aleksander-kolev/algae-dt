#!/usr/bin/env bash
# Run the algae-dt dev container the course-faithful way (WSL + WSLg display).
# Mirrors docs/SETUP.md §1: --net=host for ROS 2 DDS, X11 mounts for Gazebo/RViz/PyQt5 via WSLg,
# and the repo's ros2_ws bind-mounted at /ws. Attach more shells with:
#   docker exec -it algae_dt_container bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/.." && pwd)/ros2_ws"
# LIBGL_ALWAYS_SOFTWARE=1: under WSLg the gz 3D client + RViz use OGRE2/OpenGL, which hits the
# documented d3d12 VBO crash without a software-GL fallback (the README clone-and-run instructs
# `ros2 launch ... mode:=sim_only`, which opens the gz client by default). llvmpipe renders them.
# Mount docker/ at /ci too so the working GUI launchers (open_sim.sh / demo_run.sh) are reachable.
docker run --rm -it --name algae_dt_container \
  --net=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WS:/ws" \
  -v "$HERE:/ci" \
  algae-dt:dev bash
