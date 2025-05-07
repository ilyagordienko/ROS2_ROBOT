# Autonomous Indoor Navigation and Mapping for a Mobile Robot (Master's Thesis Project)

This repository contains the ROS 2 source code developed as part of a Master's thesis project focused on implementing autonomous indoor navigation and mapping capabilities for a custom-built mobile robot platform.

The project aims to create a functional system enabling the robot to perceive its environment, construct a spatial map, localize itself within that map, and autonomously navigate to designated goals. The software integrates data from sensors like a LiDAR for primary navigation tasks and a camera for vision-based perception, all coordinated within the Robot Operating System 2 (ROS 2) framework.

**Key Features Implemented:**

* **2D Mapping:** Generating environmental maps using Simultaneous Localization and Mapping (SLAM) algorithms based on LiDAR data.
* **Localization:** Determining the robot's precise position and orientation within a previously created map.
* **Autonomous Navigation:** Planning global paths and executing local, collision-free trajectories to achieve goal-driven movement.
* **Low-Level Control Interface:** ROS 2 interface designed to communicate with a microcontroller for managing motor commands and acquiring odometry feedback.
* **Vision-Based Perception:** Modules for basic computer vision tasks, including the detection of colors and geometric shapes using OpenCV.

**Technologies Used:**

* Robot Operating System 2 (ROS 2)
* Standard ROS 2 navigation packages (e.g., `slam_toolbox`, `Nav2` suite - including AMCL, planners, controllers, etc.)
* OpenCV library for image processing
* Sensors: 2D LiDAR, USB Camera
* Designed for integration with a Single Board Computer (like Raspberry Pi 5) and a Microcontroller (like Arduino MEGA) for hardware interfacing.

**Getting Started:**

* `Clone the repository`
* `Install dependencies (ROS 2, needed ROS 2 packages, OpenCV, etc.)`
* `Build the ROS 2 workspace using colcon`
* `Configure the code for specific hardware (e.g., update serial port names, calibrate sensors)`
* `Run the different nodes or launch files (e.g., start LiDAR, start motor control, start mapping, start navigation, run vision nodes)`
* `Hardware requirements (mention specific sensors or types)`
* `Expected topics and message types`

**License:**

This code is provided open-source and is free to use, modify.
