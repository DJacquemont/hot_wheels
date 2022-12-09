# Mobile Robotics - Thymio Path Finder Project
##### _The best vision based path findind program for Thymio_
***

## Abstract
This project is part of the course Basics of mobile robotics (ME-452). The goal is to create a software enabling the Thymio to find its path through a set of obstacles. To this end, it will need to localize itself as well as its environment, and find the best path to the goal to then drive to it. Another challenge is for it to avoid any unexpected object it meets. This project is thus split in four distinct modules, plus the main file. Those four modules are:
- <b>Vision</b>, camera based function and transformation for position, goal and terrain exctraction
- <b>Kalman Filter</b>, sensor fusion for accurate odometry computation
- <b>Optimal path algorithm</b>, path optimization using Dijkstra
- <b>Motion control</b>, in charge of the Thymio's motion and local avoidance

Those modules are further detailed bellow:

## Table of contents

1. [Vision](#Vision)
2. [Kalman Filter](#kalman-filter)
3. [Optimal path algorithm](#optimal-path-algorithm)
4. [Motion control](#motion-control)

## Vision (HAS TO BE UPDATED !!!!!)

HAS TO BE UPDATED !!!!!

This module includes functions converting the information percieved by the camera into a map, a goal and the current position of the robot. To this end, the image obtained from the camera is first filtered using a median filter, useful to remove the noise while preserving clear edges. 

The obstacles are represented as black shapes. In order to fetch the map, function <b>terrainFetch</b> computing the visibility graph is called. Pixel segmentation is done on the retrieved image, followed by a blob analysis only conserving blobs of a certain size (filtering leftover noise). These blobs are then dilated to account for the robot's size, and the corners for each blob of the dilated mask are computed. The start and end point (i.e. start position of the robot and goal position) are also fetched (using functions bellow), and added to the previous nodes (i.e. corners of the dilated image). To check weither the nodes are connected, each node is iterating over all the node, checking wether it has a direct connection (without intersecting the obstacles' dilated blobs) or not. This function returns the position of the nodes, their connections, and the obstacles' dilated mask (displayed in the visual interface).

Function <b>odoFetch</b> fetches the robots' position and angle thanks to the Thymio's red (angle) and green (position & angle) LEDs. The position is computed by first running a pixel segmentation and blob analysis to only keep the two green blobs. The center of those two blobs is found and the middle between those two points is defined to be the position of the robot. The center of the red blob is obtained with the same transformations (for one blob only), and the angle of the robot is resolved when comparing the two center found (center of the red blob, and center of the two green blobs) with respect to the X axis.

The goal represented by a red square piece of paper is fetched with function <b>goalFetch</b>. It is retreived by performing the same transformation as done for the position : pixel segmentation followed by blob analysis to check the size of the blob and avoid noise. The goal is thus the center of the blob.

In order to get a visual feedback on what the algorithm outputs, function <b>liveFeedback</b> was created, it prints each node and their connections, the dilated obstacles, the robot's current position and angle, and the goal position.

## Kalman Filter

*todo*

## Optimal path algorithm

*todo*

## Motion control

*todo*
