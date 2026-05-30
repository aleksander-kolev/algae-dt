# Run the algae-dt dev container with the workspace bind-mounted (Windows / PowerShell).
# Drops you into /ws (= this repo's ros2_ws). Headless by default — for a rendered Gazebo/RViz/GUI
# demo, run from WSL via docker/run.sh (WSLg display), as the course recommends (docs/SETUP.md §1).
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$ws   = Join-Path (Split-Path -Parent $here) 'ros2_ws'
docker run --rm -it --name algae_dt_container -v "${ws}:/ws" algae-dt:dev bash
