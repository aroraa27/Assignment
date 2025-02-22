# Trajectory Visualization and Storage for AMR Navigation

This project is designed to simplify the process of visualizing and storing the trajectory data of an Autonomous Mobile Robot (AMR) using ROS2. The system comprises several nodes that work together to collect, visualize, and save the robot's trajectory, as well as to read and transform saved trajectory data for later visualization.

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Nodes Description](#nodes-description)
  - [1. random_mover Node](#1-random_mover-node)
  - [2. trajectory_publisher_saver Node](#2-trajectory_publisher_saver-node)
  - [3. trajectory_reader_publisher Node](#3-trajectory_reader_publisher-node)
- [Custom Service](#custom-service)
- [Installation and Build Instructions](#installation-and-build-instructions)
- [Running the Nodes](#running-the-nodes)
- [RViz Visualization](#rviz-visualization)
- [Testing the System](#testing-the-system)
- [Notes and Future Improvements](#notes-and-future-improvements)

## Overview

This project provides a complete ROS2 package to:

- **Collect** the AMR’s trajectory data by subscribing to its pose topic.
- **Visualize** the trajectory in real time using RViz via MarkerArray messages.
- **Save** the recent trajectory data (for a specified time duration) to a CSV file using a custom service.
- **Read** the saved CSV file, apply a coordinate offset to match the RViz coordinate space, transform the data into another frame, and publish it for visualization.

## System Architecture

The package contains three main nodes:

1. **random_mover:** Simulates robot movement by publishing random velocity commands to move the robot (or turtle in turtlesim).

2. **trajectory_publisher_saver:**  
   - Subscribes to the robot’s pose (using turtlesim’s `/turtle1/pose` topic).
   - Buffers the pose data along with timestamps.
   - Publishes a MarkerArray representing the trajectory for live visualization in RViz.
   - Provides a ROS service (`/save_trajectory`) to save a segment of the trajectory (based on a specified time duration) into a CSV file.

3. **trajectory_reader_publisher:**  
   - Reads the saved CSV file (which may include a header).
   - Applies an offset (subtracts 5.5 from both x and y) to convert turtlesim coordinates into the RViz “map” space.
   - Publishes a static transform (using an integrated static transform publisher) between the “map” and “odom” frames.
   - Transforms the saved points from the “map” frame to the “odom” frame.
   - Publishes a MarkerArray (using a LINE_STRIP marker) representing the transformed trajectory.

## Nodes Description

### 1. random_mover Node

- **Purpose:**  
  Simulates robot movement by publishing random velocity commands.
- **Functionality:**  
  Publishes `geometry_msgs/msg/Twist` messages to `/turtle1/cmd_vel`.
- **Usage:**  
  Run this node to generate continuous movement data for turtlesim.

### 2. trajectory_publisher_saver Node

- **Purpose:**  
  Collects the robot’s pose data in real time, publishes the trajectory as a MarkerArray for visualization, and offers a service to save the trajectory data to a CSV file.
- **Functionality:**  
  - Subscribes to `/turtle1/pose` (of type `turtlesim/msg/Pose`).
  - Buffers each pose with a timestamp.
  - Periodically publishes a MarkerArray (displayed as a red line in RViz) using data from the buffer.
  - Provides the `/save_trajectory` service that accepts a filename and a time duration. When called, it saves only the trajectory data from the specified duration (starting from the current time going backward) to a CSV file.
- **Service Example Call:**  
  ```bash
  ros2 service call /save_trajectory amr_custom_msgs/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 10.0}"

### 2. trajectory_publisher_saver Node
- **Purpose:**  
  Reads the saved trajectory CSV file, applies a coordinate offset, transforms the data into the “odom” frame, and publishes it for visualization.
- **Functionality:**  
  - Reads the CSV file (with header or without) and parses timestamp, x, y, and theta values.
  - Applies an offset of -5.5 to x and y to match the RViz “map” space.
  - Uses TF2 to transform the points from the "map" frame to the "odom" frame.
  - Publishes a MarkerArray (displayed as a green line in RViz) representing the transformed trajectory.
  - Internally publishes a static transform between “map” and “odom” so that the transform lookup works correctly.

## Custom Service

The package defines a custom service (in the amr_custom_msgs package) named SaveTrajectory.srv with the following fields:
- **Request:**
  - string filename – The name (and path) of the file where the trajectory will be saved.
  - float32 duration – The time duration (in seconds) for which the trajectory data should  be saved (from the current time going backward).
- **Response:**
  - bool success – Whether the file was saved successfully.
  - string message – A message detailing the result (e.g., number of points saved or error details).

## Installation and Build Instructions

1. **Create and Set Up Your Workspace:**
   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src
   # Clone or copy your package(s) into the src folder.
   ```

2. **Build and Source the Workspace:**
  ```bash
   cd ~/ros_ws
   colcon build --symlink-install
   source install/setup.bash
  ```

## Running the Nodes

### Step-by-Step Instructions

1. **Launch Turtlesim:**
   - Open a terminal and run:
     ```bash
     ros2 run turtlesim turtlesim_node
     ```

2. **Start the Random Mover:**
   - In another terminal, run:
     ```bash
     ros2 run amr_trajectory random_mover
     ```
   - This node simulates movement by publishing random velocity commands to `/turtle1/cmd_vel`.

3. **Start the Trajectory Publisher/Saver Node:**
   - Open a new terminal and run:
     ```bash
     ros2 run amr_trajectory trajectory_publisher_saver
     ```
   - This node collects the turtle's poses, publishes a MarkerArray to `/trajectory_markers`, and provides the `/save_trajectory` service.

4. **Test the Save Service:**
   - After allowing the system to run for a while (e.g., 10–20 seconds), open another terminal and call:
     ```bash
     ros2 service call /save_trajectory amr_custom_msgs/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 10.0}"
     ```
   - Check the terminal response for a success message and verify that `trajectory.csv` is created in your working directory.

5. **Start the Trajectory Reader/Publisher Node:**
   - In a new terminal, run:
     ```bash
     ros2 run amr_trajectory trajectory_reader_publisher
     ```
   - This node reads `trajectory.csv`, applies a coordinate offset, transforms the data from the "map" frame to the "odom" frame, and publishes a MarkerArray to `/read_trajectory_markers`.
   - It also publishes a static transform between "map" and "odom" internally.

## RViz Visualization

### Visualizing Live Trajectory
1. **Open RViz:**
   ```bash
   ros2 run rviz2 rviz2
   ```
2. **Add a Marker Dispplay**
  - Click Add → By topic → Select /trajectory_markers.
  - You should see the live trajectory (red line) as published by the trajectory_publisher_saver node.

### Visualizing Saved and Transformed Trajectory
1. **Add Another Marker Display**
  - Click Add → By topic → Select /read_trajectory_markers.
  - You should see the saved trajectory (green line) after it has been transformed from "map" to "odom".

