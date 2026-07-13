# GCS Trajectory Planner (ROS2 Humble)

A **GCS (Graph of Convex Sets)** trajectory planner for 3D navigation. Given a point
cloud map and a nominal waypoint path, the planner builds a corridor of overlapping
convex free-space regions and optimizes a smooth Bezier trajectory that respects corridor containment. 
It supports **2.5D traversability-aware ground corridors**, **aerial corridors**, and **takeoff/landing hops** 
between them.

## Packages

| Package | Description |
|---|---|
| `planner_msgs` | Custom messages: `Trajectory` / `TrajectoryPoint`. |
| `gcs_core` | ROS-agnostic C++17 library: sphere-flip convex free-space regions, corridor builder, and Bezier + composite shape-timing GCS optimization (uses OSQP + Qhull). |
| `traversability_core` | ROS-agnostic C++17 library: turns a point cloud into a 2D occupancy grid for the 2.5D ground corridor generation. |
| `gcs_planner` | ROS2 `planner_node` that (integrates and visualizes the above in RViz), `trajectory_executor_node` (samples trajectories and publishes moving velocity-vector visualization), and launch/RViz config. |

Dependency chain: `gcs_core`, `traversability_core`, and `planner_msgs` → `gcs_planner`.
`gcs_core` and `traversability_core` are independent siblings.

## Prerequisites

- **ROS2 Humble**
- **Qhull** — system dependency not covered by rosdep:
  ```bash
  sudo apt install libqhull-dev
  ```
- **PCL** + `pcl_conversions`, **Eigen3**, and **OSQP** (pulled in via `osqp_vendor`).

Everything else is resolved by `rosdep` (see below).

## Setup & Build

Clone this repository into the `src/` directory of a colcon workspace:
```bash
mkdir -p ~/gcs_planning_ws/src
cd ~/gcs_planning_ws/src
git clone <repo-url> .        
cd ~/gcs_planning_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

After cloning and initial setup, if modifications are made to source files, do a standard 
`source` and `build` commands before running:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Using the launch file (starts the planner, executer, and RViz):

```bash
ros2 launch gcs_planner planner.launch.py params_file:=/path/to/planner_params.yaml
```

Or run the node directly:

```bash
ros2 run gcs_planner planner_node --ros-args --params-file src/gcs_planner/config/planner_params.yaml
```

Parameters live in `gcs_planner/config/planner_params.yaml`. Key ones include
`pcd_file`, `nominal_path_csv_files`, `traj_output_directory`, `region_radius`, 
`vel_limit`, `frame_id`, `vehicles`, and `pcd_yaw_deg` (rotates the loaded cloud about Z to match the path's frame). See the
comments in that file for the full list.

## Inputs & Outputs

**Inputs**
- A point cloud map (`.pcd`). Set via the `pcd_file` parameter.
- Nominal waypoint paths as CSV with a header row `seg_idx,tag,x,y,z`. Set via
  `nominal_path_csv_files` (one file per vehicle). The `tag` column (e.g.
  `GROUND` / `TRANS` / `AERIAL`) selects per-segment behavior.

Map and path file locations are configured by absolute paths in
`planner_params.yaml`; point them at your own `.pcd` and `.csv` files.

**Published Topics**
- `map_cloud` — downsampled input cloud
- `occupancy_grid` — `nav_msgs/OccupancyGrid` traversability occupancy
- `traversability` — traversability score markers
- `corridor` — convex free-space regions
- `trajectory_path`, `trajectory_segments` — planned trajectory visualization
- `trajectory` — `planner_msgs/Trajectory` (full trajectory)

Set `traj_output_dir` to also export per-vehicle `traj_<vehicle>.csv` files.
