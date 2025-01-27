[//]: # (Image References)

[image1]: ./visuals/path_planning_1.png  "Path Planning 1"
[image2]: ./visuals/path_planning_2.png  "Path Planning 2"


# CarND-Path-Planning-Project Writeup
Self-Driving Car Engineer Nanodegree Program
   
## Purpose
In this project we were asked to implement path planning algorithm which can drive the car in a three-lanes high way. 

## Trajectory Generation
Simulator controls the car according to the given generated waypoints. Waypoints are generated using the spline generation library: Cubic Spline interpolation (https://kluge.in-chemnitz.de/opensource/spline/)

I mostly used the code in Q&A video developed by Aaron. This code block generates waypoints that are eligible to track lanes and handle lane changes. Code block simply generates 5 waypoints, which I named pivot points, and these waypoints are fed to spline library in order to generate smooth trajectories. Smooth trajectories are needed in order to generate jerk free trajectories. Considering jerk is important in order to provide comfortness to the passengers in the autonomous car. Waypoints that are not consumed by simulator are used in order to increase smoothness. Last two waypoints are used in order to generate tangential splines. 

Pivot points are generated by using Frenet coordinates. Frenet coordinates provides a good highway abstraction and makes the math really easy. Current lane information is used in order to generate the pivot points every time. 

Spline generation is performed in ego vehicle's reference of frame. This makes the math easy. Every time generated waypoints are rotated and translated first waypoint's location. In other words, first waypoint is the origin. Spline is generated in this reference of frame and x and y points are in this reference of frame. After smooth trajectories are generated all the points are transformed to world's reference of frame. 

## Planning
The simulator provides the current position of the car, x,y,s, and d coordinates. Additionally simulator provides the sensor fusion data which consists information about other cars in highway. 

Planner simply checks the current lane of the ego vehicle. If there is a car, generated waypoints becomes more dense, which causes the car to slow down. Ego vehicle's speed is increased whenever the current lane is free. 

In addition to decreasing the speed of the ego vehicle when there is a car in current lane, ego vehicle starts to consider changing line if pre-determined specific time is passed. Sensor fusion data is used in order to determine free lanes. If a free lane cannot be determined, ego vehicle continues to follow the current lane. However if a free lane is found, ego vehicle changes the lane. 

## Results
A video file is attached to repo, [Link to video file](visuals/demo_video.mkv)

Also some pictures are attached in order to show 4 miles threshold is passed over:

![alt text][image1]
![alt text][image2]

