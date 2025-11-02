# Autonomous Path Planning (ROS 2 Humble – A* Planner)
ROS 2 Humble A* Global Path Planner for Autonomous Robots — deterministic grid-based navigation using occupancy maps, optimized for visualization and reproducibility.

This project implements an A*-based global path planner for autonomous ground robots (tested with TurtleBot3 and AgileX Hunter 2.0 simulations in Gazebo).
It loads occupancy grid maps (.pgm + .yaml), applies Gaussian smoothing and inflation to obstacles, and generates an optimal collision-free path between start and goal poses.
The output path is published as a nav_msgs/Path message and can be visualized directly in RViz2.

⚙️ This branch focuses on the A* global planner. The Dynamic Window Approach (DWA) local planner is currently disabled, to allow testing and visualization of the global path without real-time velocity sampling.

```
ros2_ws/
 └── src/
     └── my_nav_pkg/
         ├── launch/
         │   └── nav_demo.launch.py
         │   └── rviz.rviz
         ├── maps/
         │   ├── map.yaml
         │   └── map.pgm
         ├── my_nav_pkg/
         │   └── astar_planner_node.py
         │   └── adebug_viz_node.py
         │   └── local_nav_node.py
         │   └── utils.py
         ├── package.xml
         └── setup.py
```

## Key Features
Pure Python implementation of A* with heuristic optimization (Euclidean or Manhattan).
Costmap inflation and Gaussian smoothing to improve path feasibility.
Supports ROS 2 Humble nav_msgs/OccupancyGrid maps.
Publishes nav_msgs/Path for visualization in RViz2.
Optional image-based visualization of inflated costmap.
Simple configuration through YAML.

### Why A* Instead of DWA (for now)
Initially, the focus of this project was on implementing a **global planner** that generates optimal paths using map data.  
At the time, I was not yet aware of the **Dynamic Window Approach (DWA)** — a local planner capable of dynamically adjusting velocities and paths in real-time based on sensor feedback.

By the time I learned about DWA, the A* implementation was already complete and performing well for the intended scope — static, known-map navigation and visualization.  
So, instead of restarting from scratch, I decided to proceed with A* for this phase and keep DWA integration as a **future extension**.

This version therefore focuses on:
- Deterministic and reproducible path generation (via A*).  
- Visualization and verification of path alignment with the map.  
- Integration testing of global planning and path publishing in ROS 2.  

Future versions will add **DWA** for dynamic obstacle avoidance, velocity-based trajectory sampling, and smoother real-time motion control.



## Setup & Installation

1️⃣ Prerequisites
  Ensure your environment meets the following:
  Ubuntu 22.04
  ROS 2 Humble Hawksbill
  Gazebo Classic / Fortress
  Python 3.10+
  
2️⃣ Create & Build Workspace

```

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/HimanshuChudasama22/Custom_dwa_planner.git my_nav_pkg

# Build
cd ~/ros2_ws
colcon build

# Source environment
source install/setup.bash

```
## 🚀 Running the Simulation

### 1️⃣ Launch the Environment

```bash
# Launch the A* planner demo
ros2 launch my_nav_pkg nav_demo.launch.py
```

This launch file will:

  Start Gazebo with the turtlebot3_waffle model in the default turtlebot3_world.
  Launch RViz2 with the configuration file rviz.rviz.
  Start the A* global planner node (astar_planner_node.py).
  Bring up visualization and debug nodes.


## 🧭Expected Outputs

  Once launched successfully, you should observe the following:


### 🟢In Gazebo:

  A TurtleBot3 Waffle robot appears in the simulated turtlebot3_world.
  
  The robot will remain idle until a navigation goal is given.

<img width="1858" height="1056" alt="image" src="https://github.com/user-attachments/assets/9281bc85-9102-4253-a9ee-b974eb495a41" />



### 🔵 In RViz2:

  The robot model and laser scan (/scan) data are visible.
  
  The map may not render due to a known error, but the laser scan outlines the environment clearly.
  
  When you publish a goal, a path will appear as a smooth green line.
<img width="1199" height="884" alt="image" src="https://github.com/user-attachments/assets/edeb8144-4f13-41e7-9bfa-03c260ee45d8" />



### 🕹️ Controlling the Robot

  Use RViz’s “**2D Goal Pose**” tool to set a destination within the visible laser scan area.
  
  The A* planner will:
  
  - Compute an optimal collision-free path between the robot and the goal.
  
  - Publish the resulting path as a nav_msgs/Path message.
  
  - Continuously update the path if you change the goal while the robot is moving.
  (Yes — you can change the goal mid-run, and the robot will immediately replan and reroute!)

<img width="1199" height="884" alt="image" src="https://github.com/user-attachments/assets/b498a772-e5d9-462b-bc9b-c2f567391819" />


### ⚙️ Behind the Scenes

  The map and costmap are preprocessed using Gaussian smoothing and inflation to simulate realistic obstacle boundaries.
  
  A* uses an adaptive heuristic (Euclidean or Manhattan) for faster convergence.
  
  The planner ensures continuous replanning if a new goal is issued.

## 🔮 Future Scope

  This version focuses solely on global path generation using A* to verify deterministic map-based planning behavior.
  
  Future iterations will include:
  
  Dynamic Window Approach (DWA) local planner for real-time velocity and trajectory control.
  This will allow smooth acceleration/deceleration, dynamic obstacle avoidance, and time-optimal motion in cluttered environments.

  Addition of sensor fusion (LiDAR + odometry + IMU) for adaptive localization.
  
  Improved map rendering pipeline in RViz2 to resolve current visualization issues.

  Optional depth-camera-based obstacle detection to dynamically update the costmap in real-time.
