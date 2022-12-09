# Mobile Robotics - Thymio Path Finder Project
##### _The best vision based path findind program for Thymio_
***

## Abstract
This project is part of the course Basics of mobile robotics (ME-452). The goal is to create a software enabling the Thymio to find its path through a set of obstacles. To this end, it will need to localize itself as well as its environment, and find the best path to the goal to then drive to it. Another challenge is for it to avoid any unexpected object it meets. This project is thus split in four distinct modules, plus the main file. Those four modules are:
- Vision, camera based function and transformation for position, goal and terrain exctraction
- Kalman Filter, sensor fusion for accurate odometry computation
- Optimal path algorithm, path optimization using Dijkstra
- Motion control, in charge of the Thymio's motion and local avoidance

Those modules are further detailed bellow:

## Table of contents

1. [Vision](#Vision)
2. [Kalman Filter](#kalman-filter)
3. [Optimal path algorithm](#optimal-path-algorithm)
4. [Motion control](#motion-control)

## Vision

This module includes functions converting the information percieved by the camera into a map, a goal and the current position of the robot. To this end, the image obtained from the camera is first filtered using a median filter, useful to remove the noise while preserving clear edges. 

In order to fetch the map, function terrainFetch computing the visibility graph is called. Pixel segmentation is done on the retrieved image, followed by a blob analysis only conserving blobs of a certain size (filtering leftover noise). These blobs are then dilated to account for the robot's size, and the corners for each blob of the dilated mask are computed. The start and end point (i.e. start position of the robot and goal position) are also fetched, and added to the previous nodes (i.e. corners of the dilated image). To check weither the nodes are connected, for each node every node is considered

## Kalman Filter

*todo*

## Optimal path algorithm

*todo*

## Motion control

*todo*
