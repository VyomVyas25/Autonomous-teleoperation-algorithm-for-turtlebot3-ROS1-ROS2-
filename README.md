# Autonomous Teleoperation Algorithm for Turtlebot3 in ROS2 Humble

This repository contains two main directories focusing on different aspects of controlling a Turtlebot3. One directory contains autonomous algorithms, and the other is dedicated to teleoperating the rover using Micro-ROS and ESP32.

## Table of Contents
- [Project Overview](#project-overview)
- [Autonomous Turtlebot3](#autonomous-turtlebot3)
- [Teleoperation of Rover](#teleoperation-of-rover)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Project Overview

This project demonstrates the implementation of two systems for Turtlebot3:
1. **Autonomous Navigation**: Utilizes odometry, yaw, and LiDAR sensors for autonomous movement and obstacle avoidance.
2. **Teleoperation**: Allows remote control of the rover via Micro-ROS and ESP32.

## Autonomous Turtlebot3

### Overview
Implementing autonomous navigation for Turtlebot3 using:
- **Odometry**: Tracks the robot's position.
- **Yaw**: Determines the robot's orientation.
- **LiDAR Sensors**: Detects obstacles for safe navigation.

### Features
- Autonomous navigation
- Obstacle avoidance
- Path planning

### Directory Structure

### How to Run
1. Navigate into your directory.
2. git clone into the source file
3. Build the package:
   ```bash
   colcon build

Teleoperation of Rover

Overview

Teleoperating a rover using an ESP32 microcontroller with Micro-ROS, enabling remote control via network.


Features

-> Remote control using web interface or joystick

-> Seamless communication with Micro-ROS

-> Real-time control and feedback


Installation

Prerequisites

ROS2 Humble

Turtlebot3 hardware

ESP32 microcontroller

Micro-ROS

