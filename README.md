#  <div align="center">**OmniPlanner: Universal Exploration and Inspection Path Planning Across Robot Morphologies (aka GBPlanner 3.0)**</div>

<div align="center"> <a href="https://ntnu-arl.github.io/omniplanner/"><img src="https://img.shields.io/badge/Homepage-1E88E5?style=flat-square" alt="Webpage"></a>  <a href="https://arxiv.org/abs/2603.04284"><img src="https://img.shields.io/badge/arXiv-78909C?style=flat-square" alt="arXiv"></a> <a href="https://www.youtube.com/watch?v=kT4hGuejhuQ"><img src="https://img.shields.io/badge/YouTube-E57373?style=flat-square" alt="YouTube"></a> </div>

<br>

> **OmniPlanner builds upon GBPlanner 2.0**, the exploration planning method that guided all robots of **Team CERBERUS** during the winning run at the **DARPA Subterranean Challenge**. An updated version of **GBPlanner 2.0** is available in the [`gbplanner2`](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2).

<br>

**OmniPlanner** is a unified graph-based planning framework that enables autonomous robots to explore unknown environments, inspect structures and regions of interest, and navigate to specified targets. Its modular formulation adapts the planning process to the motion and sensing characteristics of aerial, ground, and underwater platforms, allowing the same framework to generate feasible and informative paths across diverse robot morphologies and operating environments.

![swag](img/omniplanner_intro.png)
<sub><em>**OmniPlanner:** A unified framework for autonomous exploration, inspection, and target-reach planning with aerial, ground, and underwater robots.</em></sub>


For an extensive documentation, installation instructions, and demos please visit the documentation page of the repository here: [**Documetation**](https://github.com/ntnu-arl/gbplanner3_wiki/wiki).


## Setup


### Create workspace for OmniPlanner
```bash
mkdir ~/omniplanner_dev_env
```

### GazeboSim: Garden
If you intend to use the [Gazebo](https://gazebosim.org/home) simulator, you will need to install the Gazebo Garden from source on Ubuntu 20.04 using the following instructions. The instructions have been taken from the original documentation [here](https://gazebosim.org/docs/garden/install_ubuntu_src).

#### Install tools:
```bash
sudo apt install python3-pip lsb-release gnupg curl git
pip3 install vcstool
pip3 install -U colcon-common-extensions
```

#### Create a workspace for gazebo:
```bash
cd ~/omniplanner_dev_env
mkdir -p gazebo_garden_ws/src
cd gazebo_garden_ws/src
```

#### Get source files:
```bash
curl -O https://raw.githubusercontent.com/ntnu-arl/gz-sim/refs/heads/fix/position_control/collection-garden.yaml
vcs import < collection-garden.yaml
```

#### Install dependancies:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

cd ~/omniplanner_dev_env/gazebo_garden_ws/src
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
```
> **_NOTE:_** Replace the files of the `gz-sim` folder with the files from [this](https://github.com/ntnu-arl/gz-sim/tree/dev/multicopter_control) and switch `gz-common` to 82a649e1 commit.

#### Build:

```bash
cd ~/omniplanner_dev_env/gazebo_garden_ws
colcon graph
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

#### Source the workspace:
```bash
source ~/omniplanner_dev_env/gazebo_garden_ws/install/setup.bash
```

### ROS-GZ Bridge
#### Create a workspace for gazebo:
```bash
cd ~/omniplanner_dev_env
mkdir -p ros_gz_bridge_ws/src
cd ros_gz_bridge_ws/src
```
#### Clone the bridge:
```bash
git clone git@github.com:ntnu-arl/ros_gz.git -b garden_noetic
cd ~/ros_gz_bridge_ws
catkin config --install
catkin build
```
> **_NOTE:_** Make sure `ros_gz_bridge_ws` extends `~/omniplanner_dev_env/gazebo_garden_ws/install` and `/opt/ros/noetic`.

#### Source the workspace:
```bash
source ~/omniplanner_dev_env/ros_gz_bridge_ws/install/setup.bash
```

## OmniPlanner Installation

#### Install dependancies:
```bash
sudo apt install python3-catkin-tools \
libgoogle-glog-dev \
ros-noetic-joy \
ros-noetic-twist-mux \
ros-noetic-interactive-marker-twist-server \
ros-noetic-octomap-msgs \
ros-noetic-octomap-ros \
git-lfs
```

#### Create the workspace:
```bash
mkdir -p ~/omniplanner_dev_env/omniplanner_ws/src/exploration
cd ~/omniplanner_dev_env/omniplanner_ws/src/exploration
```
#### Clone the planner
```bash
git clone git@github.com:ntnu-arl/gbplanner_ros.git
cd gbplanner_ros
git submodule update --init --recursive
```

Run the submodule update command after pulling changes that update submodule revisions. It checks out `autoassess_bridge` at the commit required by this repository.

#### Clone and update the required packages
```bash
cd ~/omniplanner_dev_env/omniplanner_ws/
vcs import < ./src/exploration/gbplanner_ros/vcstool/packages.repos
cd src/sim/subt_cave_sim
git lfs pull
```

#### Build
```bash
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```
> **_NOTE:_** Make sure `omniplanner_ws` extends `~/omniplanner_dev_env/gazebo_garden_ws/install`, `~/omniplanner_dev_env/ros_gz_bridge_ws/install` and `/opt/ros/noetic`.

#### Source
```bash
source ~/omniplanner_dev_env/omniplanner_ws/devel/setup.sh
```

### Run the AutoAssess CGN inspection setup

On the `gbplanner_ros-autoassess` branch, after building and sourcing the workspace, put the CDF credentials in `sdk/.env` at the workspace root and install the bridge's Python dependencies as described in the [bridge README](autoassess_bridge/README.md#typical-setup). Then run:

```bash
roslaunch gbplanner gbplanner_cgn.launch
```

This launch file starts the planner, manhole detector, simulation, and AutoAssess bridge together.
It loads `waypoint_reach_bwt.xml`. Press **NDT waypoint navigation** in the CGN RViz
panel. The UI sends the existing position controller a world-frame setpoint at z = 1.0 m
while preserving x, y, and yaw. After odometry shows a stable hover for three seconds,
it starts automatic planning. In each compartment, the planner builds its inspection
graph but discards the inspection path, then selects unfinished target poses published by
the bridge one at a time. Once those targets are reached, it traverses the manhole and repeats.
RViz displays the bridge's `/autoassess/inspection_targets` poses as cyan arrows.
**Stop Planner** also cancels the altitude and hover stage. If the vehicle is already
positioned, automatic planning can be started directly with:

```bash
rosservice call /planner_control_interface/std_srvs/automatic_planning
```

### Generate compartment configuration from a downloaded map

Run `gbplanner/scripts/configure_compartments_from_map.py` after the bridge has
downloaded a new PCD. The script estimates transverse wall planes, orders a row
of compartments, and calculates:

- `PlanningParams.compartment_centers` (XYZ in the map frame).
- `PlanningParams.compartment_dimensions` (one shared, axis-aligned sampling box).
- `BoundedSpaceParams.Global` (enclosing all generated compartment boxes).

From this repository directory, preview the current reference map:

```bash
python3 gbplanner/scripts/configure_compartments_from_map.py \
  --map ~/.ros/autoassess_maps/7810494868369385/mesh_984.pcd \
  --config gbplanner/config/uav/gzc/cgn_elios3/gbplanner_config_elios3.yaml \
  --roi -2 -2.8 -0.5 12.2 3 3.3 \
  --expected-compartments 5 \
  --report-prefix /tmp/compartment_map
```

Add **`--apply`** to that command to automatically update the config after
inference succeeds. It first saves an exact timestamped `.bak.*` copy and then
atomically replaces the YAML. Other parameter values are preserved, but YAML
comments and formatting are rewritten. Alternatively use
`--output /tmp/generated_planner.yaml` to create a separate complete config.
Neither option changes the running planner. Restart it to load the file:

```bash
roslaunch gbplanner gbplanner_cgn.launch autoassess_set_global_bound:=false
# For a separate generated config, also pass:
# gbplanner_config_file:=/tmp/generated_planner.yaml
```

The bridge bound override must be disabled when using the generated bounds.
The script writes a JSON report and a PNG top-down preview (the rectangles show
estimated room extents; the shared sampling box also includes `--margin`).
No C++ rebuild is needed to run the script directly. Its dependencies are
`python3-numpy`, `python3-scipy`, `python3-yaml`, and `python3-matplotlib`.
After rebuilding the package it is also available through
`rosrun gbplanner configure_compartments_from_map.py`.

For a **new map**, replace `--map` and select the tank region with
`--roi XMIN YMIN ZMIN XMAX YMAX ZMAX`. Use `--whole-map` only if it contains just
the tank of interest. This script assumes a straight row of compartments with
roughly parallel walls, including both end walls. It supports ASCII and
uncompressed binary PCD; if normals are absent, they are estimated locally.
The PCD must already use the planner's world coordinates; no TF conversion or
map registration is performed.

`--axis-yaw-deg` gives the approximate forward direction (default +X; use 180
to reverse the ordering). `--min-width` and `--max-width` constrain room spacing
(defaults 1.5–3.5 m); change these for different tanks. Expected room count is
optional but useful as a validation constraint. The first center is estimated
from the first room, rather than forced to the drone's starting position.

Wall selection is a spacing heuristic: baffles, missing walls, branched rooms,
and incomplete scans can produce incorrect estimates. The report includes all
wall candidates and the selected sequence; inspect the preview on a new map.
For a reviewed layout, `--wall-offsets` can supply the boundary-plane offsets
along the fitted axis, including both end walls. These offsets are not generally
world X coordinates. Geometry bounds do not change collision voxels or robot
clearance settings, and wall inference does not prove that manholes are traversable.

The planner classifies each target between the detected entry and exit wall planes.
CGN uses map-aligned compartment centers at x = 0, 3.04, 5.42, 7.83, 10.44 m,
y = 0 m and z = 1.5 m. Relative compartment bounds are `[-2, -1, -1.5]`
to `[2, 1, 1]` m; configured global bounds are `[-2, -2, 0.5]` to `[10, 2, 2]` m.
Only X centers have been adjusted in this comparison: the old third center at
x = 4 m caused the detected exit at x = 6.63 m to exceed the opening-selection
distance limit. Global bounds still truncate the final compartment and will need
review before full coverage there can be expected.
`autoassess_set_global_bound` defaults to true, allowing the bridge to request
structural-element bounds as before. Set it to false to keep configured bounds.
The entry wall is the manhole used for the previous traversal; the exit wall is selected
with the same opening-selection routine used for the next traversal. Missing exit
walls make the tree wait. Wall normals are oriented along mission progress, so tilted
walls and reversed detector normals are handled. Within 5 cm of a wall, target viewing
direction resolves which side should inspect it. Targets in other compartments are
deferred and completed targets are remembered across traversals.
Inspection arrival requires remaining within `inspection_waypoint_reach_radius_m`
(default 0.15 m) for `inspection_waypoint_hold_s` (default 3.0 s of ROS time in
`gbplanner_cgn.launch`). After the hold, the planner selects the next pending target
in the compartment. Once none remain, it traverses the next manhole and repeats
the target check in the next compartment.
With `waypoint_reach_bwt.xml`, PCI's path completion radius is half the inspection
arrival radius (0.075 m by default). This lets the controller finish short trajectories
before PCI requests another path; the previous 0.7 m tolerance could cause repeated
commands to restart the trajectory before its final waypoint was applied.
Leaving that radius resets the dwell. The planner first tries the requested position,
then searches nearest-first on a 3D lattice for a reachable free pose in the same
compartment. `inspection_waypoint_search_radius_m` defaults to 1.0 m and
`inspection_waypoint_search_resolution_m` to 0.1 m. Up to
`inspection_waypoint_search_attempts` (default 8) free candidates are checked for a
route per planning attempt. This is a bounded sampled search, not an exact continuous
nearest-point solution. Unknown and occupied robot boxes and blocked path segments
are rejected. Inspection target endpoints must additionally be free using the
extended robot box (`size + size_extension`), even on relaxed planning retries,
so they have the clearance required for the next departure. This checks endpoint
clearance at planning time; it does not guarantee a subsequent manhole approach
route or account for later map changes or tracking error.
The adjusted goal retains the requested viewing orientation; the
flight path uses level poses with yaw, as expected by the position controller.
For the `InspectionWaypoint` action, yaw follows the shortest turn from the
starting heading to the target heading, interpolated by distance along the route.
Intermediate graph viewing angles are ignored. This is confined to target travel
in the waypoint tree; ordinary inspection and manhole traversal retain their yaw behavior.
The selected goal appears in orange as **Reachable Inspection Waypoint** on
`/gbplanner/inspection_waypoint`; the bridge's original targets remain cyan. Arrival
and the dwell use that selected goal, while completion refers to the original target
ID. Failed searches keep the target pending. These checks cover robot clearance and
path validity in the current map, but do not verify inspection visibility from the adjusted pose.
The first compartment has no entry plane; the last has no exit plane. Configured
compartment centers still specify mission order and direction, and configured planning
bounds still constrain graph generation. This assumes a sequence of compartments
separated by manhole walls. The bridge supplies poses only.
The old standalone waypoint sequencer remains available with
`inspection_target_waypoints_en:=true`, but it must not run alongside this tree.

## Citation

```bibtex
@article{zacharia2026omniplanner,
  title   = {OmniPlanner: Universal Exploration and Inspection Path Planning across Robot Morphologies},
  author  = {Zacharia, Angelos and Dharmadhikari, Mihir and Singh, Mohit and Alexis, Kostas},
  journal = {arXiv preprint arXiv:2603.04284},
  year    = {2026},
  url     = {https://arxiv.org/abs/2603.04284}
}
```

## GBPlanner Legacy

![swag](img/cerberus_subt_winners.png)

Earlier versions of GBPlanner have been deployed on multiple aerial and ground robot platforms:

![robots](img/gbplanner3_robots.png)

For background on the methods and deployments that preceded OmniPlanner, please refer to the following publications:

**Graph-based subterranean exploration path planning using aerial and legged robots**
```bibtex
@article{dang2020graph,
  title={Graph-based subterranean exploration path planning using aerial and legged robots},
  author={Dang, Tung and Tranzatto, Marco and Khattak, Shehryar and Mascarich, Frank and Alexis, Kostas and Hutter, Marco},
  journal={Journal of Field Robotics},
  volume = {37},
  number = {8},
  pages = {1363-1388},  
  year={2020},
  note={Wiley Online Library}
}
```
**Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots**
```bibtex
@INPROCEEDINGS{9812401,
  author={Kulkarni, Mihir and Dharmadhikari, Mihir and Tranzatto, Marco and Zimmermann, Samuel and Reijgwart, Victor and De Petris, Paolo and Nguyen, Huan and Khedekar, Nikhil and Papachristos, Christos and Ott, Lionel and Siegwart, Roland and Hutter, Marco and Alexis, Kostas},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)}, 
  title={Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots}, 
  year={2022},
  volume={},
  number={},
  pages={3306-3313},
  doi={10.1109/ICRA46639.2022.9812401}}
```

## Acknowledgements

This work was supported in part by the Research Council of Norway through the NCEI project (Grant No. 357451), and by the European Commission under the Horizon Europe Programme through the SYNERGISE (Grant No. 101121321), AUTOASSESS (Grant No. 101120732), SPEAR (Grant No. 101119774), and DIGIFOREST (Grant No. 101070405) projects. The authors are solely responsible for the content and ideas presented here.

OmniPlanner is intended for civilian use only and is provided under the terms of the repository's [LICENSE](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner3/LICENSE).

## Contact

For questions, please contact:

- [Angelos Zacharia](mailto:angelos.zacharia@ntnu.no)
- [Mihir Dharmadhikari](mailto:mihir.dharmadhikari@ntnu.no)
- [Mohit Singh](mailto:mohit.singh@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
