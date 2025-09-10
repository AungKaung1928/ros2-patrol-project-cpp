# ROS2 Autonomous Patrol Navigation System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-green.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

This project implements an autonomous patrol navigation system using ROS2 and Nav2. The robot autonomously navigates through a series of predefined waypoints in a continuous patrol pattern, utilizing the Nav2 navigation stack for path planning and obstacle avoidance.

### Key Features

- **Autonomous Waypoint Navigation**: Seamlessly navigates through predefined patrol points
- **High-Speed Configuration**: Optimized Nav2 parameters for rapid movement (up to 4.0 m/s)
- **Dynamic Obstacle Avoidance**: Real-time obstacle detection and avoidance using laser scan data
- **Configurable Patrol Routes**: Easy modification of patrol points via YAML configuration
- **Robust Error Handling**: Automatic retry mechanism for failed navigation attempts
- **Gazebo Simulation Support**: Full integration with TurtleBot3 Gazebo simulation

## System Architecture

```
┌─────────────────────┐
│  Patrol Controller  │
│    (Main Node)      │
└──────────┬──────────┘
           │
           ├──► Waypoint Manager (Loads patrol points)
           │
           └──► Nav2 Action Client
                     │
                     ▼
        ┌────────────────────────┐
        │    Nav2 Stack          │
        │  - Planner Server      │
        │  - Controller Server   │
        │  - Recovery Behaviors  │
        └────────────────────────┘
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Gazebo 11 or higher

### ROS2 Dependencies
```bash
# Core ROS2 packages
ros-humble-rclcpp
ros-humble-rclcpp-action
ros-humble-geometry-msgs
ros-humble-nav-msgs
ros-humble-nav2-msgs
ros-humble-tf2
ros-humble-tf2-ros
ros-humble-tf2-geometry-msgs

# Navigation stack
ros-humble-navigation2
ros-humble-nav2-bringup

# TurtleBot3 packages
ros-humble-turtlebot3
ros-humble-turtlebot3-gazebo
ros-humble-turtlebot3-navigation2

# Additional dependencies
libyaml-cpp-dev
```

## Installation

1. **Clone the repository into your ROS2 workspace:**
```bash
cd ~/ros2_ws/src
git clone <repository-url> patrol_navigation_project_cpp
```

2. **Install dependencies:**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package:**
```bash
colcon build --packages-select patrol_navigation_project_cpp
source install/setup.bash
```

## Configuration

### Patrol Points Configuration

Edit `config/patrol_points.yaml` to define custom patrol waypoints:

```yaml
patrol_points:
  - name: "point_1"
    x: 2.0
    y: 0.0
    z: 0.0
  - name: "point_2"
    x: 2.0
    y: 2.0
    z: 0.0
  # Add more points as needed
```

### Navigation Parameters

The `config/nav2_params.yaml` file contains optimized parameters for high-speed navigation:

- **Maximum velocity**: 4.0 m/s forward
- **Maximum angular velocity**: 8.0 rad/s
- **Acceleration limits**: 12.0 m/s² (linear), 15.0 rad/s² (angular)
- **Goal tolerance**: 0.3m (position), 0.3 rad (orientation)

## Usage

### Launch Simulation Environment

1. **Set the TurtleBot3 model:**
```bash
export TURTLEBOT3_MODEL=waffle
```

2. **Launch Gazebo simulation:**
```bash
ros2 launch patrol_navigation_project_cpp patrol_gazebo.launch.py
```

3. **In a new terminal, launch the patrol navigation system:**
```bash
ros2 launch patrol_navigation_project_cpp patrol_navigation.launch.py
```

The robot will automatically:
- Initialize its position at the origin
- Load the patrol waypoints
- Begin autonomous patrol navigation
- Continue patrolling indefinitely through all waypoints

### Launch Arguments

Optional launch arguments for `patrol_navigation.launch.py`:

```bash
ros2 launch patrol_navigation_project_cpp patrol_navigation.launch.py \
  map:=/path/to/custom/map.yaml \
  params_file:=/path/to/custom/nav2_params.yaml
```

## Project Structure

```
patrol_navigation_project_cpp/
├── config/
│   ├── nav2_params.yaml         # Nav2 stack configuration
│   └── patrol_points.yaml       # Patrol waypoint definitions
├── include/patrol_navigation_project_cpp/
│   ├── patrol_controller.hpp    # Main controller header
│   └── waypoint_manager.hpp     # Waypoint management header
├── launch/
│   ├── patrol_gazebo.launch.py  # Gazebo simulation launcher
│   └── patrol_navigation.launch.py # Navigation system launcher
├── src/
│   ├── patrol_controller.cpp    # Main controller implementation
│   └── waypoint_manager.cpp     # Waypoint manager implementation
├── CMakeLists.txt               # Build configuration
├── package.xml                  # Package manifest
└── README.md                    # This file
```

## API Documentation

### PatrolController Class

**Main Methods:**
- `startPatrol()`: Initiates the patrol sequence
- `stopPatrol()`: Stops the current patrol and cancels navigation
- `goToNextPoint()`: Navigates to the next waypoint in sequence

### WaypointManager Class

**Main Methods:**
- `loadPatrolPoints()`: Loads waypoints from YAML configuration
- `getPatrolPoints()`: Returns the vector of patrol points
- `getNextPoint(size_t current_index)`: Returns the next waypoint in sequence

## Troubleshooting

### Common Issues and Solutions

1. **Robot not moving:**
   - Ensure Nav2 stack is fully initialized (wait for "Nav2 is ready!" message)
   - Check if the map is properly loaded
   - Verify TurtleBot3 model is set: `echo $TURTLEBOT3_MODEL`

2. **Navigation failures:**
   - Check for obstacles blocking the path
   - Verify goal tolerances in `nav2_params.yaml`
   - Review costmap configuration for proper obstacle detection

3. **High CPU usage:**
   - Reduce controller frequency in `nav2_params.yaml`
   - Decrease the number of velocity samples
   - Lower the costmap update frequency

4. **YAML loading errors:**
   - Verify patrol_points.yaml syntax
   - Ensure yaml-cpp is properly installed
   - Check file permissions

## Performance Optimization

The current configuration is optimized for high-speed navigation. To adjust for different requirements:

### For Safer/Slower Operation:
- Reduce `max_vel_x` and `max_vel_theta` in nav2_params.yaml
- Increase `inflation_radius` for larger safety margins
- Increase `cost_scaling_factor` for more conservative paths

### For Better Obstacle Avoidance:
- Increase `BaseObstacle.scale` value
- Reduce `sim_time` for shorter planning horizon
- Increase `obstacle_max_range` for earlier detection
