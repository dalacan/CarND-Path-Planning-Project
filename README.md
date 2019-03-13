# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Model Documentation
The source file implements path planning functionality for highway driving. The model is able to plan trajectories that 
will:
1. Keep the vehicle within the lane
2. Change lane to reach optimal speed
3. Stay within the maximum speed, acceleration and jerk constraints.

[//]: # (Image References)

[fsm]: ./images/fsm.png "Finite State Machine"

## Overview
The path planning model implements the following functionality:
1. Ego Vehicle tracking
2. Road vehicle tracking via sensor fusion
3. Prediction of ego and sensed vehicle kinematics in the future
4. Behavior planning
5. Trajectory generation

## Highway module
Prior to running the path planning model, the highway map is loaded into the system via the 
[highway module](src/highway.cpp). The highway module houses the following core functionality:
* Loading of the the highway waypoints file
* Get the lane number for a specified (x,y, theta) or d coordinate
* Get the XY coordinates for a specified (s,d) frenet coordinates
* Get the FreNet coordinates for a specified (x, y, theta) coordinate

## Ego initialization and update
When the path planning is loaded for the first time, the ego vehicle is initialized with the localization data 
recieved from the simulator via the web socket. Every subsequent update, the ego localization data is calculated 
based on the last two previous path points that have yet to be processed by the simulator.

## Road vehicle tracking via sensor fusion
The next step in the path planning involves keeping track of all the vehicles from the sensor fusion output. 
In addition to tracking the vehicles, the position of the vehicle is linearly estimated based on the vehicles'
current velocity and a period of time in the future. The period of time in the future is the number of previous 
path points * 0.02 seconds. The estimated position is used to plan the next behavior and trajectory after the 
previous points.

## Prediction
The prediction follows a linear prediction model for the prediction of each vehicles position in the future. 
It uses the vehicles current fretnet S position, velocity, acceleration to predict the future frenet S position. 
The formula to predict the frenet S position is given by:
```
s_f = s_i + v*t + a*t^2
```
- s_f is the predicted frenet S position
- s_i is the initial frenet S posistion
- v is the vehicle current velocity
- a is the vehicle current acceleration

The future velocity is predicted by the following formula:
```
f_v = v_i + a*t
```
- v_f is the predicted vehicle velocity
- v_i is the vehicle current velocity
- a is the vehicle current acceleration

## Behavior Planning
Once the prediction has been generated for all vehicles, the next step are:
1. Find the next suitable state based on the current ego vehicle state
2. Generate an estimated trajectory for each successor states
3. Calculate the costs for each state based on the estimated trajectory
4. Choose the state with the lowest cost

### States
The valid states of the vehicles are:
- Constant speed/initial state (CS)
- Keep lane (KL)
- Prepare lane change left (PLCL)
- Prepare lane change right (PLCR)
- Lane change left (LCL)
- Lane change right (LCR)

![alt-image][FSM]

#### Constant speed state
Properties:
- The initial state of the vehicle
- The vehicle will maintain a constant speed.

Successor States:
- Keep lane

#### Keep lane state
Properties:
- Stay near the centre line for the lane.
- Drive at maximum legal speed if possible. If there is a vehicle in front, match vehicle speed.

Successor States:
- Keep lane
- Prepare lane change left
- Prepare lane change right

#### Prepare lane change left
Properties:
- Stay near the centre line for the lane.
- Attempt to match speed of left lane

Successor States:
- Keep lane
- Prepare lane change left
- Lane change left

#### Prepare lane change right
Properties:
- Stay near the centre line for the lane.
- Attempt to match speed of right lane

Successor States:
- Keep lane
- Prepare lane change right
- Lane change right

#### Lane change left
Properties:
- Move to the left lane
- Drive at maximum legal speed if possible. If there is a vehicle in front, match vehicle speed.

Successor States:
- Keep lane
- Lane change left

#### Lane change right 
Properties:
- Move to the right lane
  - Drive at maximum legal speed if possible. If there is a vehicle in front, match vehicle speed.

Successor States:
- Keep lane
- Lane change right

### Cost functions
To select be optimal behavior, the following cost functions were implemented:
- Inefficiency cost - Penalizes slow lane relative to neighbouring lane
- Lane change cost - Penalizes lane changing relative to distance to vehicle in front
- Out of lane cost - Penalizes changing to a lane that does not exists
- Safe change lane cost - Penalizes changing lanes with smaller gaps. (Preferred wider gaps)
- Preferred lane cost - Rewards staying in the centre lane.

## Trajectory Generation
Once the state has been selected, the next step of the model is to generate a suitable trajectory to 
realize the choosen state. The implementation of the trajectory generation utilizes the 
[spline](https://kluge.in-chemnitz.de/opensource/spline/) library by:
1. Generating a set of (x,y) coordinates from the 
   1. Previous path
   2. At least 3 (x, y) waypoints of 30m S (Frenet) intervals. Note: Testing with higher velocities requires more 
   waypoints
2. Adding all unprocessed previous path waypoints to the the trajectory
3. Shifting the (x,y) coordinates to the car's reference angle (i.e. the coordinates run along the x-axis). The shifting 
help in generating suitable x points to be used to evaluate y points from the pline.
4. Generate suitable x and y trajectory coordinates by
   1. Selecting an x point (30) and evaluating the y point from the spline.
   2. Evaluate the number of x intervals by dividing the tangent of the x and y by the number of previous points left 
   multiplied by the target velocity.
   ```
   N = tangent / (velocity * 0.02)
   ```
   If accelerating to a target velocity from a current velocity, iteratively calculate the N value whilst incrementing 
   the `velocity` variable. Followed by calculating the X interval by dividing the X (30) by N and adding the X interval
   to each previous X interval.
   3. Parse the x into the spline to get the y coordinates
5. Shift the collected x and y coordinates back to the original map heading.
6. Add the shifted coordinates to the end of the previous path trajectory.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


