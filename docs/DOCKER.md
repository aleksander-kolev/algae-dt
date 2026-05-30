# DOCKER.md — the reproducible dev image (`algae-dt:dev`)

This repo ships a **course-faithful Docker image** that is our reproducible equivalent of the
course's provided `turtlebot3_ws` image (docs/SETUP.md §0/§1). It exists so the **definition of
done** — a clean `colcon build` + `pytest` + a `sim_only` run — is reproducible on any machine
(home WSL/Docker, the lab laptop, CI) without hand-installing ROS.

It is built **on top of the stock turtlebot3 packages** exactly as the course mandates
(DECISIONS.md D2). Nothing in `docker/` re-implements them.

## What's inside
ROS 2 **Jazzy** + the full stock turtlebot3 stack (`turtlebot3_gazebo`, `_navigation2`, `_bringup`,
`_teleop`, `_cartographer`, `turtlebot3_msgs`, `turtlebot3_simulations`) + **Nav2** +
`nav2_simple_commander` + **Gazebo Harmonic** (via `ros-gz`) + `rviz2` + **PyQt5** + `colcon` +
`pytest`/`pytest-cov`. `TURTLEBOT3_MODEL=burger`, `LDS_MODEL=LDS-02`, `ROS_DOMAIN_ID=36` preset.

The workspace is **bind-mounted at `/ws`** at run time (not copied in), matching the course
`-v ~/turtlebot3_ws:/ws`. The repo's `ros2_ws/` IS that workspace root (`ros2_ws/src/algae_dt`).

## Build
```powershell
docker/build.ps1          # Windows
```
```bash
bash docker/build.sh      # WSL / lab laptop / Linux
# or, raw:  docker build -t algae-dt:dev docker
```

## Verify (clean build + full test suite, headless)
From the repo root:
```bash
docker run --rm -v "$PWD/ros2_ws:/ws" -v "$PWD/docker:/ci" algae-dt:dev bash /ci/ci.sh
```
```powershell
docker run --rm -v "${PWD}\ros2_ws:/ws" -v "${PWD}\docker:/ci" algae-dt:dev bash /ci/ci.sh
```
`ci.sh` sources ROS, `colcon build --packages-select algae_dt`, sources the overlay, and runs
`pytest src/algae_dt/test`. Pass pytest args through: `... bash /ci/ci.sh -k safety`.

## Interactive run (course-faithful, with display)
Best from **WSL** so WSLg renders Gazebo/RViz/the PyQt5 GUI:
```bash
bash docker/run.sh                       # --net=host + X11 mounts + ros2_ws→/ws
# inside the container:
cd /ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select algae_dt && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```
Attach more shells: `docker exec -it algae_dt_container bash`.
Windows/PowerShell headless shell: `docker/run.ps1` (no display — use WSL for the rendered demo).

## Headless simulation (verification without a display)
Gazebo runs server-only when no GUI is needed (topics/Nav2/mission still run):
```bash
export GZ_HEADLESS=1            # honored by bringup.launch.py (gz sim -s, no gui)
ros2 launch algae_dt bringup.launch.py mode:=sim_only
```

## Relationship to the course image
On the **lab laptop** the native `~/turtlebot3_ws` already has this stack — you don't need Docker
there (docs/SETUP.md §2). At **home**, either the course's `turtlebot3_ws` image or this
`algae-dt:dev` image works; this one is pinned and reproducible from `docker/Dockerfile`.
