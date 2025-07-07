# px4_onboard_control

This repository provides ROS 2-based control functionality for onboard PX4 flight controllers. It includes keyboard teleoperation, modular ROS2 nodes for onboard control, and support for QoS configurations.

---

## 🚀 Getting Started

### 1. Build the Package

1. Download the official px4-ros2-interface-lib repo to your workspace/src

```bash
git clone https://github.com/Auterion/px4-ros2-interface-lib.git
```

2. Replace the manual example using this repo's manual mode.

3. colcon build

### 2. Use the mode

1. Source
```bash
source install/setup.bash
```

2. Make sure you have Micro-XRCE-DDS-Agent installed (https://github.com/eProsima/Micro-XRCE-DDS-Agent). And run

```bash
./start_node.sh
```

3. Use your radio controller to switch to manual mode and arm.

4. Run the keyboard control node in control. 