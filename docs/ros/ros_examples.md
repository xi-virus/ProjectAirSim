# Project AirSim ROS Bridge Examples

Examples of using the Project AirSim ROS Bridge as part of a ROS network are included in the Project AirSim distribution under the `ros/examples` directory.

## Requirements
The Project AirSim ROS Bridge examples requires the following:

* A version of Python supported by the [Project AirSim Client API](../client_setup.md#python-client).
* The following Python packages in addition to those required by the Project AirSim ROS Bridge: [moveit](http://wiki.ros.org/moveit) and[moveit_msgs](http://wiki.ros.org/moveit_msgs).

## Setup
1. Setup the [Project AirSim ROS Bridge](ros.md#setup).
2. The files for Project AirSim ROS Bridge examples are in the `ros/examples` and `ros/examples_externals` directory of the distribution files and are provided as the ROS catkin package `projectairsim_ros_examples`.  These directories must be added to your ROS catkin workspace and and built.  It's recommended to add a soft link in the `catkin_ws/src` directory to the `ros` directory of the extracted Project AirSim distribution files which will add all Project AirSim ROS packages to your ROS catkin workspace.
4. Activate the [Python virtual environment setup for Project AirSim](../client_setup.md).

## Building
To build the Project AirSim ROS Bridge Examples package run `catkin_make`.

## Running
The Project AirSim ROS Bridge Examples must be run from within a [Python virtual environment setup for Project AirSim](../client_setup.md).

## Individual Examples

[build_map](example_build_map.md): Creates a 3D occupancy map (octomap) using Project AirSim, the MoveIt! motion library, and RViz.

[navigate_map](example_navigate_map.md): Navigate the 3D occupancy map constructed by [build_map](example_build_map.md) using Project AirSim, the MoveIt! motion library, and RViz.

---

Copyright (C) Microsoft Corporation.  All rights reserved.
