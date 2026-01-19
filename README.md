# ROS 2 Obstacle Avoider (Gazebo Simulation)

## Overview
This repository implements a **reactive obstacle avoidance system** for a custom differential-drive robot using **ROS 2**, **Gazebo Sim**, and **LiDAR perception**.

The focus is on **behavior-based robotics**, using sector-based spatial reasoning instead of global planning.

---

## Key Features

- Custom differential-drive robot
- GPU LiDAR sensor with noise model
- Sector-based obstacle perception
- Continuous steering control
- Deadlock handling in cluttered environments
- Empirically tuned parameters
- Custom obstacle-dense Gazebo world

---

## Obstacle Avoidance Strategy

### LiDAR Sectorization
The LiDAR scan is divided into:
- **Left sector**
- **Front sector**
- **Right sector**

Each sector computes the **minimum valid range**, allowing robust spatial reasoning even with noisy data.

---

### Control Logic

- If front sector is clear → move forward
- If obstacle detected ahead:
  - Compare left and right clearance
  - Steer toward the side with more free space
- If clearance is similar:
  - Slow forward motion with forced rotation to escape deadlock

Angular velocity is proportional to the difference between left and right distances, enabling smooth steering.

---

## System Architecture

Gazebo World
↓
GPU LiDAR
↓
Sector-Based Analysis
↓
Reactive Avoidance Controller
↓
cmd_vel
↓
Differential Drive Plugin

---

## Robot & Simulation

- Differential drive mobile base
- Accurate inertial modeling
- Tuned friction coefficients
- Gazebo Sim plugins for motion
- ROS–Gazebo bridging
- RViz visualization support

---

## Repository Structure

.
├── urdf/

│ ├── my_robo_v0.urdf.xacro

│ ├── lidar.xacro

│ └── inertia_descriptions.xacro

├── worlds/

│ └── obstacle_avoider.sdf

├── src/

│ └── object_avoider_node.py

├── launch/

│ └── simulation.launch.xml

└── config/

└── gazebo_bridge.yaml


---

## How to Run

```bash
colcon build
source install/setup.bash
ros2 launch program_bringup my_gazebo_robot.launch.xml
ros2 run <package_name> object_avoider
```
## Concepts Demonstrated

Reactive navigation

LiDAR sector-based perception

Continuous steering control

Deadlock handling

Parameter tuning

ROS 2–Gazebo integration

## Design Philosophy

No global maps

No planners

No Nav2

Emphasis on perception → action loops

## Future Work

Behavior fusion with wall follower

State machine or behavior tree

Velocity smoothing

Real robot testing

## Author

# Chirag Gujrathi
Robotics Engineering | ROS 2 | Autonomous Systems
