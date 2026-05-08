# Autonomous Exploring Rover

A ROS1-based autonomous navigation project for the Waveshare JetRacer ROS AI Kit.

This repository contains the software developed for an autonomous mobile rover that can map an unknown environment, detect ArUco visual landmarks, save landmark positions, localize itself on a saved map, plan paths, and navigate toward marker-based goals.

The project was developed for **AIS4104 - Robotics and Intelligent Systems with Project** at NTNU.

---

## Project Overview

The goal of this project was to build an integrated autonomous navigation pipeline for the JetRacer platform. The system combines several robotics modules:

- SLAM-based 2D mapping using LiDAR and odometry
- Autonomous exploration using a custom frontier-based controller
- Automated map finalization and saving
- ArUco marker detection using OpenCV
- Marker waypoint registration in a YAML file
- AMCL-based localization on a saved map
- Custom A* global planning integrated into `move_base`
- TEB local planning for trajectory execution
- Marker-ID-based goal navigation

The final system allows the robot to receive a semantic command such as:

```bash
rostopic pub -1 /target_marker_id std_msgs/Int32 "data: 2"
```

and convert that marker ID into a navigation goal on the saved map.

---

## System Architecture

The project is divided into two main phases.

---

### Phase 1: Mapping and Landmark Generation

During the first phase, the robot explores the environment, builds a map, detects ArUco markers, and saves marker positions.

Main components:

- `slam_gmapping` generates the occupancy grid map.
- `auto_explorer` drives the robot autonomously during mapping.
- `youssef_slam` supervises the mapping process and saves the map when it becomes stable.
- `aruco_processor.py` detects ArUco markers and estimates their pose.
- `waypoint_saver.py` stores detected marker poses in a YAML file.

Output files from this phase include:

```text
mymap.pgm
mymap.yaml
aruco_waypoints.yaml
```

These files form the reusable geometric and semantic navigation dataset.

---

### Phase 2: Localization and Navigation

During the second phase, the saved map and marker database are reused for autonomous navigation.

Main components:

- `map_server` loads the saved occupancy grid map.
- `AMCL` estimates the robot pose in the map frame.
- `aruco_processor.py` can bootstrap localization using a known marker.
- `move_base` coordinates global and local planning.
- A custom A* global planner generates paths on the costmap.
- `teb_local_planner` executes local motion.
- `marker_goal_navigator.py` converts marker IDs into `MoveBaseGoal` commands.

The navigation pipeline is:

```text
/target_marker_id
        |
        v
marker_goal_navigator.py
        |
        v
move_base
        |
        v
Custom A* Global Planner
        |
        v
TEB Local Planner
        |
        v
/cmd_vel
        |
        v
JetRacer driver
        |
        v
Physical robot motion
```

---

## Main Features

---

### Autonomous SLAM Supervision

The `youssef_slam` node monitors the occupancy grid during SLAM and checks how much the map changes between consecutive updates.

The map-change heuristic is conceptually:

```text
C = number of changed cells between two consecutive occupancy grid maps
```

When the map remains stable for a configured period, the node automatically:

1. Publishes `/exploration_done`
2. Stops the robot
3. Saves the map using `map_server`
4. Copies the map files to the target directory
5. Stops the SLAM node

This removes the need to manually decide when the map should be saved.

---

### Frontier-Based Autonomous Exploration

The `auto_explorer` node performs autonomous exploration using:

- LiDAR sector analysis
- frontier detection from the occupancy grid
- BFS-based frontier clustering
- heuristic goal scoring
- short-horizon velocity sampling
- obstacle clearance checks
- recovery behaviours for blocked or stuck situations

The node subscribes to:

```text
/scan
/odom
/map
/exploration_done
```

and publishes velocity commands to:

```text
/cmd_vel
```

---

### ArUco Marker Detection and Mapping

The vision system uses OpenCV ArUco detection to identify markers and estimate their pose relative to the camera.

The system supports:

- multiple ArUco dictionaries
- camera calibration
- coordinate conversion between OpenCV and ROS frames
- temporal filtering to reduce jitter
- marker pose publishing
- waypoint saving to YAML
- localization bootstrapping using known markers

The final marker database is stored in:

```text
aruco_waypoints.yaml
```

---

### Marker-Based Navigation

Instead of manually sending map coordinates, the robot can be sent to a marker ID.

Example:

```bash
rostopic pub -1 /target_marker_id std_msgs/Int32 "data: 2"
```

The `marker_goal_navigator.py` node:

1. Loads marker positions from `aruco_waypoints.yaml`
2. Finds the requested marker ID
3. Builds a `MoveBaseGoal`
4. Sends the goal to `move_base`
5. Optionally calculates goal yaw from the robot's current AMCL pose

The goal yaw can be calculated as:

```text
yaw_goal = atan2(y_goal - y_robot, x_goal - x_robot)
```

---

### Custom A* Global Planner

The project includes several stages of path planning development:

- Manually defined Dijkstra waypoint graph
- Grid-based A* with 4-connectivity
- A* with 8-connectivity
- Wall penalty
- Obstacle inflation
- Distance-field-based centering cost
- Final integration as a `nav_core::BaseGlobalPlanner` plugin for `move_base`

The final planner works on the ROS costmap and returns a path to `move_base`.

The basic A* cost function is:

```text
f(n) = g(n) + h(n)
```

where:

```text
g(n) = cost from the start node to node n
h(n) = heuristic estimate from node n to the goal
f(n) = estimated total path cost through node n
```

---

## Repository Structure

```text
Ais4103-Jetracer-ROS-AI-KIT/
├── config/
├── src/
│   ├── jetbot_pro/
│   ├── jetracer_ros/
│   ├── my_camera_utils/
│   ├── ris/
│   │   ├── config/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── map/
│   │   ├── scripts/
│   │   │   ├── aruco_goal_navigator.py
│   │   │   ├── aruco_processor.py
│   │   │   ├── marker_goal_navigator.py
│   │   │   └── waypoint_saver.py
│   │   ├── src/
│   │   │   ├── graph_planner/
│   │   │   ├── grid_planner/
│   │   │   ├── map_reader/
│   │   │   ├── navigation/
│   │   │   └── slam/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── plugin_description.xml
│   ├── ris_msgs/
│   ├── rplidar_ros/
│   └── slam/
├── .catkin_workspace
├── .gitignore
└── README.md
```

---

## Important Launch Files

The main launch files are located in:

```text
src/ris/launch/
```

Important launch files include:

```text
slam_and_vision.launch
navigation_marker_bootstrap.launch
youssef_slam.launch
auto_mapping.launch
ris_vision.launch
localization_with_map.launch
report_visualization.launch
slam_with_vision.launch
```

Typical use:

```bash
roslaunch ris slam_and_vision.launch
```

for mapping and marker registration, then:

```bash
roslaunch ris navigation_marker_bootstrap.launch
```

for localization and marker-based navigation.

---

## Hardware

The project was tested on the Waveshare JetRacer ROS AI Kit.

Main hardware components:

- NVIDIA Jetson Nano
- Waveshare JetRacer expansion board
- DC gear motors with encoders
- Servo steering
- Raspberry Pi CSI camera module
- LiDAR sensor
- Wi-Fi adapter
- External development laptop or virtual machine

---

## Software Environment

The system was developed and tested using:

- Ubuntu 18.04
- ROS1 Melodic
- Catkin workspace
- OpenCV
- OpenCV ArUco
- RViz
- `slam_gmapping`
- `map_server`
- `amcl`
- `move_base`
- `teb_local_planner`
- `rplidar_ros`
- Python ROS nodes
- C++ ROS nodes

---

## Installation

Clone the repository into a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TheMuffinmuffler/Ais4103-Jetracer-ROS-AI-KIT.git
```

Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```

Source the workspace:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

To avoid sourcing manually every time, add this to `~/.bashrc`:

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Make Python scripts executable if needed:

```bash
chmod +x ~/catkin_ws/src/Ais4103-Jetracer-ROS-AI-KIT/src/ris/scripts/*.py
```

---

## Dependencies

Install the required ROS packages:

```bash
sudo apt update
sudo apt install ros-melodic-navigation
sudo apt install ros-melodic-slam-gmapping
sudo apt install ros-melodic-map-server
sudo apt install ros-melodic-amcl
sudo apt install ros-melodic-move-base
sudo apt install ros-melodic-teb-local-planner
sudo apt install ros-melodic-tf
sudo apt install ros-melodic-tf2-ros
sudo apt install ros-melodic-cv-bridge
sudo apt install ros-melodic-image-transport
sudo apt install ros-melodic-camera-calibration
```

Depending on the JetRacer setup, additional packages for LiDAR, camera, and motor drivers may be required.

---

## Network Setup

The JetRacer and development computer must be on the same network.

Typical ROS networking variables:

On the JetRacer:

```bash
export ROS_MASTER_URI=http://<JETRACER_IP>:11311
export ROS_IP=<JETRACER_IP>
```

On the laptop or VM:

```bash
export ROS_MASTER_URI=http://<JETRACER_IP>:11311
export ROS_IP=<LAPTOP_IP>
```

Replace `<JETRACER_IP>` and `<LAPTOP_IP>` with the correct static IP addresses.

Check communication:

```bash
rostopic list
rostopic echo /scan
rostopic echo /odom
```

---

## Usage

---

### 1. Start Mapping and Vision

Run the mapping and marker-generation phase:

```bash
roslaunch ris slam_and_vision.launch
```

This starts the SLAM pipeline, autonomous exploration, map supervision, ArUco processing, and waypoint saving.

Expected outputs:

```text
mymap.pgm
mymap.yaml
aruco_waypoints.yaml
```

---

### 2. Start Navigation

After the map and marker waypoints are saved, start the navigation phase:

```bash
roslaunch ris navigation_marker_bootstrap.launch
```

This loads the map, starts AMCL, launches `move_base`, starts marker-based localization support, and enables marker-goal navigation.

---

### 3. Bootstrap Localization with an ArUco Marker

The system can use a known marker as a localization reference. The bootstrap marker ID can be changed with:

```bash
rostopic pub -1 /aruco_processor/set_bootstrap_id std_msgs/Int32 "data: 1"
```

Replace `1` with the marker ID that should be used as the localization reference.

---

### 4. Send the Robot to a Marker

Send the robot to marker ID 2:

```bash
rostopic pub -1 /target_marker_id std_msgs/Int32 "data: 2"
```

The marker goal navigator will load the marker pose, construct a navigation goal, and send it to `move_base`.

---

## Key ROS Topics

| Topic | Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/odom` | `nav_msgs/Odometry` | Odometry estimate |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM or loaded map |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command |
| `/exploration_done` | `std_msgs/Bool` | Stops exploration after map completion |
| `/target_marker_id` | `std_msgs/Int32` | Marker ID navigation command |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL pose initialization |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Estimated robot pose |
| `/marker_goal_pose` | `geometry_msgs/PoseStamped` | Selected marker goal visualization |
| `/planned_route` or `/ris_global_plan` | `nav_msgs/Path` | Planned path visualization |

---

## Important Files

---

### SLAM and Exploration

```text
src/ris/src/slam/
```

Contains the custom SLAM supervision and autonomous exploration logic.

Important nodes:

```text
youssef_slam
auto_explorer
```

---

### Computer Vision

```text
src/ris/scripts/aruco_processor.py
src/ris/scripts/waypoint_saver.py
```

The ArUco processor detects markers, estimates pose, supports mapping/localization modes, and publishes marker data. The waypoint saver stores marker poses in YAML format.

---

### Marker-Based Navigation

```text
src/ris/scripts/marker_goal_navigator.py
```

Converts marker IDs into `move_base` goals.

---

### Path Planning

```text
src/ris/src/graph_planner/
src/ris/src/grid_planner/
src/ris/src/navigation/
```

Contains Dijkstra tests, A* planner experiments, and the integrated global planner implementation.

---

### Planner Plugin

```text
src/ris/plugin_description.xml
```

Registers the custom global planner plugin for use with `move_base`.

---

## Planner Testing

Earlier planner experiments can be found in:

```text
src/ris/src/grid_planner/
```

The planner was tested with:

- 4-connectivity A*
- 8-connectivity A*
- obstacle inflation
- wall penalty
- distance field
- path simplification

The original README focused mainly on these planner tests, but the full project is larger than the planner alone.

---

## Known Limitations

This system is a proof-of-concept research project, not a production-ready autonomous driving stack.

Known limitations:

- Autonomous exploration can terminate before full environment coverage.
- The map-stability criterion checks geometric convergence, not semantic completeness.
- ArUco marker detection depends on lighting, distance, camera calibration, and marker visibility.
- Navigation accuracy is affected by accumulated errors from SLAM, AMCL, TF, camera calibration, costmap inflation, and local planner tolerances.
- The robot navigates to stored marker coordinates; it does not perform final visual servoing toward the marker.
- TF timing and startup order can affect system reliability.
- Some experiments required controlled marker placement and manual timing.

---

## Future Work

Possible improvements:

- Add semantic-aware exploration so mapping continues until required markers are detected.
- Combine map-stability checking with landmark-completeness checking.
- Improve TF synchronization and startup sequencing.
- Add a system-readiness checker before accepting navigation goals.
- Add visual servoing for accurate final approach to markers.
- Perform more repeated quantitative testing.
- Improve robustness in cluttered and low-light environments.
- Improve planner and local controller integration.
- Replace purely heuristic exploration with optimization-based or learning-based exploration methods.

---

## Authors

- Khadija Rauf
- William Haugland
- Youssef Abdul Ghafour

Project developed as part of:

```text
AIS4104 - Robotics and Intelligent Systems with Project
Department of ICT and Natural Sciences
NTNU
May 2026
```

---

## Acknowledgements

This repository builds on the Waveshare JetRacer ROS AI Kit ecosystem and standard ROS1 navigation components.

The project uses existing ROS packages such as `slam_gmapping`, `map_server`, `amcl`, `move_base`, `teb_local_planner`, and JetRacer driver components. The project contribution is the integration of these components with custom exploration, SLAM supervision, ArUco-based landmark registration, marker-based navigation, and custom global planning.

---

## AI Assistance Disclosure

AI tools were used as programming support during parts of the project. The assistance included ROS syntax help, debugging support, code structuring suggestions, and implementation guidance for selected modules.

The system design, integration decisions, parameter tuning, testing, and validation were carried out as part of the project work.

---

## License

No explicit license has been provided in this repository. Unless a license file is added, the code should be treated as not openly licensed for reuse beyond viewing and educational reference.






































##################### Planner and Map location #####################

- The map used to test the path planner (A*) is "Ais4103-Jetracer-ROS-AI-KIT/my_map.pgm" and 

- The file constaining the planner and visualizing it in Rviz is "Ais4103-Jetracer-ROS-AI-KIT/src/ris/src/grid_planner/plan_testRviz.cpp"
  
I modified the cost function by assigning a higher cost to cells near obstacles. I also applied inflation, (expanded obstacles by a certain radius so that nearby cells become more costly). As a result, the generated path is more centered and safer, although it is still not perfectly optimal.



##################### Screenshots from Planner #####################

Note: The grid shown in the images does not use the original resolution. The actual resolution is 0.05 m per cell, but a larger resolution is used in RViz for better visualization.


----- Path obtained with 4-connectivity planner -----

<img width="471" height="551" alt="image" src="https://github.com/user-attachments/assets/16f4ab9e-c79d-43f8-adbc-c92c4c6d1961" />


----- Path obtained with 8-connectivity planner -----

<img width="353" height="545" alt="image" src="https://github.com/user-attachments/assets/326722ad-8033-40d8-9f21-d0848a731432" />


----- Path obtained with 8-connectivity planner, inflation radius = 7  -----

<img width="394" height="544" alt="image" src="https://github.com/user-attachments/assets/da64b186-fffc-4f6b-b2a5-31ed0ac20a68" />


----- Path obtained with 8-connectivity planner, inflation radius = 10  -----

<img width="398" height="543" alt="image" src="https://github.com/user-attachments/assets/8bddbe5a-ec06-42c7-8105-491182c37bb2" />
