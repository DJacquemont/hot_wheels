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

## Vision

This module includes functions converting the information percieved by the camera into a map, a goal and the current position of the robot. To this end, the image obtained from the camera is first filtered using a median filter, useful to remove the noise while preserving clear edges.

The obstacles are represented by black shapes. In order to fetch the map, function <b>terrainFetch</b> computing the visibility graph is called. Pixel segmentation is done on the retrieved image, followed by a blob analysis only conserving blobs of a certain size (filtering leftover noise). These blobs are then dilated to account for the robot's size, and the corners for each dilated blob are computed. The start and end point (i.e. start position of the robot and goal position) are also fetched (using functions bellow), and added to the previous nodes (i.e. corners of the dilated blobs). To check weither the nodes are connected, each node is iterating over all the node, checking wether it has a direct connection (without intersecting a dilated blobs). This function returns the position of the nodes in meters, their connections, and the dilated obstacles' mask (displayed in the visual interface).

Function <b>odoFetch</b> fetches the robots' position and angle thanks to the Thymio's red (angle) and green (position & angle) LEDs. The position is computed by first running a pixel segmentation and blob analysis to only keep the two green blobs. The center of those two blobs is found and the middle between those two points is defined to be the position of the robot. The center of the red blob is obtained with the same transformations (this time for one blob only), and the angle of the robot is resolved when comparing the two center found (center of the red blob, and center of the two green blobs) with respect to the X axis (i.e. using the arctan). The position of the robot is returned in pixel since this function is used in <b>terrainFetch</b>, but the position in meter is returned by the function <b>fetchOdoMeters</b>, calling <b>odoFetch</b> multiple times and converting the result in meter.

The goal represented by a red square piece of paper is fetched with function <b>goalFetch</b>. It is retreived by performing the same transformation as done for the position : pixel segmentation followed by blob analysis to check the size of the blob and avoid noise. The goal is thus the center of the blob and is returned in meters.

In order to get a visual feedback on what the algorithm outputs, function <b>liveFeedback</b> was created, it prints each node and their connections, the dilated obstacles, the robot's current position and angle, and the goal position.

## Kalman Filter

*todo*

## Optimal path algorithm

To find the optimal path for the robot, we decide to implement Dijkstra’s algorithm on our project. The vision module extract all coordinate of nodes and edges obstacles from obstacles.  This data is sent thanks to the function terrainFetch (vid). We run this function until detect the map correctly. This step is done on the function ‘opt_path ()’. When it’s done, we run the Dijkstra algorithm with our function. 

In this function, we initialize a list of nodes and another one with edges. Note that we have to duplicate edges with different directions, because the vision module gives us only one way. Then, we define two lists to save the length of the path, and its coordinates of each node and one dictionary, which allows us to know the index of each node on its lists.

We initialise three variables to define the current situation (current node, distance between it and the starting point, all nodes of its path).  The ‘iteration’ variable helps us to define the future node after each iteration.

As long as the edges list isn’t empty, we calculate the distance between the start and end point of each edge having as starting point the current node. Then, we update the two array tabLenpath and tabPath. I remove all used edges and ones which, the end point is the current node. At the end, we updated the situation increasing iteration by one and updating the three variables.

When edge list is empty, we extract the optimal path by tabPath and index of goal position. In order for the motion control module to be able to use the output correctly we transform the list into an array.

## Motion control

Because Thymio is a two-wheeled robot it can go strait forward by setting the right and left wheelspeed to the same value, and it can pivot by setting the wheel speed to opposite values. These are the two basic control movement we used to control our Thymio's motion.

We have decided to use a proportional controller for both the forward and pivot motion: the further the robot's state from the desired state, the faster the movement.
| Motion | Inputs | Computes | Outputs |
|---:|---|---|---|
| **Forward** | - Position of the robot, target point | - Distance to the target point | Wheel speed = $K * d + C$|
| **Pivot** | - Angle of the robot, target point | - Difference in angle between the robot and the target point | Wheel speed = $ \pm K \cdot \alpha_{diff} + C$|

> It is important to note that the right and left wheel speeds differ a bit according the the angle between Thymio and the target point during the forward motion in order to track better the target. For example if the angle of Thymio is a bit off by $-\pi/8$ then the right wheel will be a bit faster than the left wheel.

We have designed a finite state machine as follows to decide what movement Thymio should follow. The 


