# px4_onboard_control

This repository provides ROS 2--based onboard control and
coverage-planning functionality for PX4 platforms (UGV/RC car or UAV).\
It includes:

-   Keyboard teleoperation\
-   Modular ROS 2 control nodes\
-   Obstacle processing and global coverage planning\
-   PX4 microRTPS communication support\
-   Custom QoS profiles for real-time control

------------------------------------------------------------------------

## 🚀 Getting Started

Below are the required steps to correctly prepare the environment before
using the onboard control and planning modules.

------------------------------------------------------------------------

## **0. Build `px4_msgs` (Required Before Building This Package)**

This repository depends on the PX4 ROS 2 message definitions provided
in:

🔗 https://github.com/PX4/px4_msgs

Follow the build instructions on the px4_msgs GitHub page for your OS
and ROS 2 distribution.

    cd ~/ros2_ws/src
    git clone https://github.com/PX4/px4_msgs.git

    cd ~/ros2_ws
    colcon build --packages-select px4_msgs
    source install/setup.bash

------------------------------------------------------------------------

## **1. Build the Package**

1.  Clone the official PX4 ROS 2 interface library:

    ```
    cd ~/ros2_ws/src
    git clone https://github.com/Auterion/px4-ros2-interface-lib.git
    ```
    

2.  Replace the example manual node inside their repo with this
    repository's `examples/cpp/modes/manual/` implementation.

3.  Build:

    ```
    cd ~/ros2_ws
        colcon build
        source install/setup.bash
    ```
    

------------------------------------------------------------------------

## **2. Run the System**

### **2.1 Source environment**
    ```
    source install/setup.bash
    ```

### **2.2 Start the Micro XRCE-DDS Agent**

Install: https://github.com/eProsima/Micro-XRCE-DDS-Agent

Run:
    ```
    ./start_node.sh
    ```
### **2.3 Switch to manual mode and arm**

### **2.4 Run keyboard teleoperation**
    ```
    python qos_keyboard_ctl.py
    ```
------------------------------------------------------------------------

# 🗺️ Planning Module Usage

Two modules exist in `planning/`:

------------------------------------------------------------------------

## **1. `obstacle_mapper.py`**

-   Subscribes to `/livox/livox_lidar`
-   Produces voxelized obstacle map
-   Publishes to `/planning/obstacles_voxel`

------------------------------------------------------------------------

## **2. `coverage_planner.py` (Global Planner --- Requires `/odom`)**

Requires a robot odometry source on:

    /odom   (nav_msgs/Odometry)

Supports:

-   Global map building (100m × 100m)
-   Frontier/B\* coverage planning
-   Publishes `/cmd_vel` navigation commands

------------------------------------------------------------------------

# ⚠️ Important Notes

-   `/odom` must be present for global planning
-   `/livox/livox_lidar` must publish `PointCloud2`
-   Planner integrates seamlessly with your controller

------------------------------------------------------------------------

# 🧭 Summary

This repository enables:

-   PX4 onboard control\
-   Real‑time teleoperation\
-   LiDAR-based mapping\
-   Global B\* coverage planning using `/odom`

Before building:

1.  Compile `px4_msgs`\
2.  Add `px4_ros2_interface_lib`\
3.  Build with `colcon`\
4.  Ensure `/livox/livox_lidar` + `/odom` are available\
5.  Run mapping and planning nodes

------------------------------------------------------------------------