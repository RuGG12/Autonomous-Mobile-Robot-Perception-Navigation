# Real-Time SLAM and Navigation on Husarion ROSbot XL using Ouster OS1

A comprehensive ROS 2 navigation and SLAM package that integrates **Ouster 3D LiDAR** perception with the standard 2D **Nav2** stack for the **Husarion ROSbot XL**.

## Overview

This package provides:
- **3D-to-2D Laser Conversion:** A custom pipeline to convert raw 3D PointCloud2 data from the Ouster OS1 into 2D LaserScan data compatible with SLAM Toolbox.
- **Hardware Time Synchronization:** A custom "Timestamp Fixer" node to resolve synchronization issues between the Ouster sensor hardware clock and the robot's system clock.
- **Nav2 Integration:** Complete integration with the ROS 2 Navigation Stack (Planner, Controller, Recoveries).
- **SLAM Integration:** Real-time 2D mapping and localization using SLAM Toolbox.

## System Architecture & Hardware Setup

This system is designed for a distributed ROS 2 environment where the robot handles sensing and actuation, while a remote laptop handles high-level navigation.



**1. Husarion ROSbot XL (Robot Platform)**
* **Role:** Base platform, motor control, odometry.
* **Connection:** Connected to local WiFi router.

**2. Nvidia Jetson (Sensor Coprocessor)**
* **Role:** dedicated computer mounted on the robot to handle the Lidar driver.
* **Connection:**
    * **To Robot:** Connected via Ethernet/WiFi.
    * **To Ouster Lidar:** Connected via Ethernet.
* **Responsibility:** Launches the `ouster_ros` driver to publish raw points.

**3. Laptop (Navigation Station)**
* **Role:** Runs the heavy computation (SLAM, Path Planning, RViz).
* **Connection:** Connected to the same WiFi network as the robot.
* **Responsibility:** Runs this `inspector` package, SLAM Toolbox, and Nav2.

**Network Configuration:**
* All devices must be on the **same WiFi network**.
* All devices must share the same **`ROS_DOMAIN_ID`** (e.g., `export ROS_DOMAIN_ID=0`).

## Prerequisites

- **Robot Platform:** Husarion ROSbot XL
- **Sensor:** Ouster OS1 (3D LiDAR)
- **OS:** Ubuntu 22.04 (Jammy Jellyfish)
- **ROS Version:** ROS 2 Humble Hawksbill
- **Python:** 3.10+

## Installation (On Laptop)

### Step 1: Install Clearpath Packages

**Important:** This project uses Clearpath configuration tools for the base setup. You must follow the official Clearpath documentation to install the proper packages.

Please visit the [Clearpath Robotics Documentation](https://docs.clearpathrobotics.com/docs/ros2humble/ros/tutorials/simulator/install) and follow the installation steps.

### Step 2: Install Standard Dependencies

Install the required standard ROS 2 navigation and perception packages:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-pointcloud-to-laserscan \
                 ros-humble-robot-localization \
                 ros-humble-tf2-ros
````

### Step 3: Add Robot Configuration

Ensure you have a `robot.yaml` file in your setup directory (default: `~/clearpath/`). This defines your robot's mounts and base configuration. If you haven't created one, use the `robot.yaml` provided in this repository as a template.

### Step 4: Clone the Repository

Navigate to your ROS 2 workspace `src` directory and clone this repository:

```bash
cd ~/ros2_ws/src
git clone https://github.com/RuGG12/Autonomous-Mobile-Robot-Perception-Navigation.git
```

### Step 5: Install Package Dependencies

Install any remaining dependencies using `rosdep`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 6: Build the Workspace

Build the package using `colcon`. Note that the package name is `inspector`.

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select inspector
source install/setup.bash
```

## Usage

To run the full autonomous navigation stack on your laptop, ensure the robot and Jetson are powered on and publishing topics. Then run these two commands in separate terminals on your laptop.

### 1\. Launch Lidar Integration (Converter & TF)

This launch file handles the hardware-specific translations. It runs the **Timestamp Fixer** (to sync sensor time), the **3D-to-2D Converter**, and publishes the static **TF link** between the robot base and the sensor.

```bash
ros2 launch inspector lidar_converter.launch.py
```

*(Note: `use_sim_time` defaults to `false` for real robot operation).*

### 2\. Launch Navigation & SLAM

This launch file starts the core autonomy stack, including **SLAM Toolbox** (for mapping) and **Nav2** (for path planning and control).

```bash
ros2 launch inspector clearpath_standard_demo.launch.py
```

## Operations

### How to Initialize

1.  **Open RViz:** (It will launch automatically with the second command).
2.  **Verify Data:** Ensure the **Fixed Frame** is set to `map` and you see the laser scan and map building.
3.  **Initialize Pose:** If the robot is lost, use the **"2D Pose Estimate"** tool in RViz to set the initial location on the map.
4.  **Send Goal:** Use the **"Nav2 Goal"** tool to send the robot to a target location.

### Configuration

  - **Navigation Tuning:** To adjust speed, inflation radius, or costmap settings, edit:
    `inspector/config/nav2_params.yaml`
  - **Lidar Mounting:** If you change the physical position of the Ouster, update the static transform numbers in:
    `inspector/launch/lidar_converter.launch.py`
