# Galbraith Memorial Mail Robot — ROB301 Final Project

Bayesian discrete-state localisation on a TurtleBot3 Waffle Pi in a simulated office building track (ROS2 Jazzy / Gazebo Harmonic).

## Prerequisites

- Ubuntu 24.04 (bare-metal or WSL2 with WSLg)
- ROS2 Jazzy
- Gazebo Harmonic (`gz-sim` 8.x)

## Workspace setup

```bash
mkdir -p ~/gazebo_projects/src
cd ~/gazebo_projects

# Clone this repo
git clone <this-repo-url> .

# Clone TurtleBot3 dependencies (Jazzy branch)
cd src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ..

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Running the simulation

Open four terminals (all sourced with `source install/setup.bash`):

```bash
# Terminal 1 — Gazebo world + robot spawn
ros2 launch bayesian_localization_project turtlebot3.launch.py

# Terminal 2 — Camera perception (colour + line detection)
ros2 run bayesian_localization_project perception_node

# Terminal 3 — Bayesian localisation + delivery controller
ros2 run bayesian_localization_project bayes_loc_node

# Terminal 4 (optional) — Live belief bar chart
ros2 run bayesian_localization_project belief_visualizer
```

## System overview

The robot navigates a closed rectangular loop containing 11 colour-coded office patches.

**Phase 1 — Exploration**: drives one full lap (11 `nothing→colour` transitions detected) to let the Bayesian filter converge before any delivery attempt.

**Phase 2 — Delivery**: delivers mail to target offices (default: offices 6, 8, 10) once the MAP estimate has been stable for 8 consecutive frames (~0.8 s).

### State space

| Index | Office | Colour  | Position (x, y)  |
|-------|--------|---------|------------------|
| 0     | 2      | yellow  | (-1.1, -1.6)     |
| 1     | 3      | green   | (1.0, -1.6)      |
| 2     | 4      | blue    | (2.05, -0.95)    |
| 3     | 5      | orange  | (2.05, -0.15)    |
| 4     | 6      | orange  | (2.05, 0.70)     |
| 5     | 7      | green   | (1.2, 1.6)       |
| 6     | 8      | blue    | (-0.15, 1.6)     |
| 7     | 9      | orange  | (-0.95, 1.6)     |
| 8     | 10     | yellow  | (-1.8, 0.70)     |
| 9     | 11     | green   | (-1.8, -0.20)    |
| 10    | 12     | blue    | (-1.8, -1.0)     |
| 11    | —      | traversal (corridor) | —      |

### Spawn position

Default spawn is `(0.6, -1.6)` facing `+x` (midway along the bottom straight). Change `x_pose` / `y_pose` / `-Y` in `launch/turtlebot3.launch.py` to test cold-start from different positions.
