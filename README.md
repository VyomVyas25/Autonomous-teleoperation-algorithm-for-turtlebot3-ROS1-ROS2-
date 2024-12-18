# micro-ROS Setup for ROS 2 Humble

This repository contains the setup and configuration files for working with **micro-ROS** in a ROS 2 Humble environment. micro-ROS is an embedded framework for using ROS 2 concepts on microcontrollers, enabling seamless integration of resource-constrained devices with ROS 2 systems.

---

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (or compatible)
- ROS 2 Humble installed ([Install ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html))
- Basic knowledge of ROS 2 and microcontroller platforms

### Dependencies
Ensure the following are installed:
- `colcon`: Build tool for ROS 2
- `micro_ros_setup` package:
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions
