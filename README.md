# Prolog Path Planning

## TODO

- ~~aggregate instead of findall~~
- ~~acceleration should be average of cur and nxt~~
- ~~separate node and state~~
- ~~don't re-evaluate states~~
- trajectory model
- obstacles
- ~~visited~~
- ~~frontier~~
- encourage values in range instead of reject
- encourage
- ~~remove c++ linear algebra lib~~
-  polish c++ code
- go over prolog best practices and style guide
- ~~separate prolog code into modules~~
- polish prolog code
- write documentation

### Clarifications

This is basically a pacman search. We plan ahead somewhat, at each step we calculate the possible moves we can make, then choose best one according to heuristic. We can possibly backtrack somewhat if needed, but not strickly required. An goal state is not strickly required. The grid should be discretized somehow, to avoid infinite possibilities - e.g 1m, 2m, 3m, 4m, etc. Alg will select best move out of all frontier moves. 

## Sensor Fusion

The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.

`0 1056.683 1159.969 14.97111 6.122647 274.0449 9.965927`

## Path Planning

The overall plan i



## World Model

- The world is a 6946m length highway loop.
- MapGateway coordinates are in meters.
- The controller will visit all output coordinates every 0.2 seconds
- The number of output coordinates is up to us.
- The increment between coordinates determines the vehicle velocity, yaw, etc. 
- there are 3 lanes in each direction of the highway
- lanes are 4m apart
- vehicle starts at the center lane
- center of center lane is 6m from left edge of left lane in direction of travel.

## Goals and Limits

- max speed of 50 mph (80.4672 km/h, 22.352 m/s)
- max acceleration of 10 m/s^2
- max jerk of 10 m/s^3





## Overview

Prolog based autonomous vehicle path planning expert system. 

[Link to paper 1](./prolog_autonomous_dynamic_paper.pdf)

[Link to paper 2](./prolog_autonomous_road_rules_paper.pdf)

[Link to paper 3](./prolog_autonomous_ferenet_trajectory.pdf)

[Link to udacity project base](https://github.com/udacity/CarND-Path-Planning-Project)

[Link to udacity simulator (term-3)](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

![term_3_sim](./term_3_sim.png)

- ==better explain how it all works and why==
- ==upload and attach a video to showcase==



## Perception

Sensor fusion data is provided (without noise) by the simulator:

- The map of the highway is in `data/highway_map.txt`
- ["x"] The car's x position in map coordinates
- ["y"] The car's y position in map coordinates
- ["s"] The car's s position in frenet coordinates
- ["d"] The car's d position in frenet coordinates
- ["yaw"] The car's yaw angle in the map
- ["speed"] The car's speed in MPH
- ["previous_path_x"] The previous list of x points previously given to the simulator
- ["previous_path_y"] The previous list of y points previously given to the simulator
- ["end_path_s"] The previous list's last point's frenet s value
- ["end_path_d"] The previous list's last point's frenet d value
- Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
  ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

_Note:_ frenet coordinates i.e define center of lane as y axis. 



##  Command and Control

Mostly handeled by simulator. C++ interaction interface provided. 

We'll iimplement sensor fusion data pre-processing, SWI-Prolog bridge to interact with path planning expert system, path-planner output post-processing and forwarding to simulator control. 

[Link to SWI-Prolog C++ interface](http://www.swi-prolog.org/pldoc/man?section=cpp-intro)



## Extrapolation and Prediction

Descision making based on prediction into the future (estimate future positions based on velocity, and acceleration). Otherwise, our vehicle will live ~2s in the past due to aggregation of various latencies. 

- ==start without extrapolation, then impl later. API stays constant anyway==

- ==we could probably use some very simple regression model or HMM like in the paper, perceptron, etc==



## Logic and Planning

In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

- throttle control to stay near 50 MPH target speed. 
- frontal, lateral, rear collision avoidance. 
- lane switch pipeline

![top down view](./top_down.png)

- ==we should define all the possible states after running the simulator and seeing what's what==
- ==we should do a state machine illustration to simplify working==
- ==implement jerk, acceleration handling==



## Trajectory

We'll calculate the trajectory based on rule-based decision.













