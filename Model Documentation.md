# Model Documentation

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