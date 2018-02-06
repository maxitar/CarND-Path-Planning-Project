# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Model Documentation

### Code Overview

The code is organized in the following files

* `main.cpp` -- handles the communication with the simulator
* `map.h/map.cpp` -- definition of the Map class, used to store the map waypoints and the transformations between XY and Frenet coordinates
* `car.h` -- definitions of the Car and Agent classes. Class Car stores the data for the main car as well functions to transform between map and car coordinates. Class Agent is used to store data for other cars.
* `helper_functions.h` -- several helpful functions for angle transformations, as well as the function for checking if the data recieved from the server is not empty
* `trajectory_generator.h/.cpp` -- these files contain the main algorithms for generating the trajectory
* `spline.h` -- a GPL licensed spline library sourced from http://kluge.in-chemnitz.de/opensource/spline/

### Trajectory Generation

The code for trajectory generation is mainly in the TrajectoryGenerator class and the process follows these steps

1. `main.cpp` calls the method `getOptimalTrajectory()` on its `gen` object
2. A state machine defined in the subclass FSM updates its state and returns the target lane and speed
3. A spline path is generated between current position and the target lane
4. Using this path a trajectory that respects the maximum allowed speed and accleration is generated
5. We evaluate this trajectory for possible collision with the other vehicles
6. If there are no collisions, we return this trajectory. Otherwise check if the first possible collision occurs within 2 seconds and decrease the speed of the car to that of the other vehicle

Note that since we need to preserve the state of the FSM, we define the `TrajectoryGenerator` object outside the main loop and capture it in the lambda.(_Lines 25, 27 in main.cpp_) 

Let us now go into a little more detail for steps 2.--6.

#### Finite State Machine

The FSM has three possible states - Keep Lane (`KL`), Prepare Lane Change (`PLC`) and Lane Change (`LC`).

* Keep Lane _(Lines 38--63 in trajectory_generator.cpp)_

In this state, we monitor the slowest speed in each lane 50 meters ahead. If we find a lane that is at least 1 meter per second faster than our current lane, we try to move into its direction and change the state to `PLC`. If this lane is not next to the current lane, we first set as target the middle lane.
If there is no faster lane, we try to maintain at least 10 meters distance to the car in front. 

* Prepare Lane Change _(Lines 74--119 in trajectory_generator.cpp)_

Since the car must stay in this state until it finds an opening to switch lanes, we first check if our ultimate target lane (named `final_lane` in the code) is still the fastest. If it is not, then we revert back `KL` state. 
After that we check if we have enough room to change lanes, as well as if given current speeds this space will be free for the next 3 seconds. To do this check, we find the speeds and the relative positions of the nearest agents in the target lane that are in front of and behind our car. If speeds of the agent in our current lane and either one of the agents (front or back) are close, we try to slow down a little in order to complete the lane change. Otherwise, the car moves as fast as possible

* Lane Change _(Lines 120--142 in trajectory_generator.cpp)_

If the car is still partially in its original lane, we set the speed, so as not to hit cars in either one of the lanes. Once it crosses entirely into the target lane, we set the speed the respect only the target lane speeds and switch the state. If the target lane is the same as the final lane, we go to `KL` state. If the target and final lanes are different, we set the target to final lane and switch to `PLC` state.

For path generation in `KL` and `PLC` state, we return the original lane. For `LC` we return the target lane.

#### Path Generation _(Lines 181--228 in trajectory_generator.cpp)_

The path is generated using spline interpolation. The spline is built on some of the points that are left from the previous trajectory as well as several new points. Specifically, we take 3 points that are 33 meters apart in s coordinate, starting from the last point that we copied from the previous path. Their d coordinate is set to the center of the lane that we get from the FSM. All calculations are performed in car coordinates (i.e. the car is the origin and its heading points to the x-axis).

#### Trajectory Generation _(Lines 230--287 in trajectory_generator.cpp)_

The trajectory is generated using some points from the previous trajectory (to ensure smooth paths), as well as new points found using the spline described above. Since the car visits each point alogn the trajectory every `dt=0.02` seconds, we must space them apart so as not to violate the constraints of the problem (max speed = 22m/s, max acceleration = 10m/s<sup>2</sup>, max jerk = 10m/s<sup>3</sup>). To estimate the speed, we first compute a small step in `x` direction and compute the spline value there (let us call it `y=f(x)`). Then the linearization of the path curve gives us the length of this section as l<sup>2</sup> = (x<sub>2</sub>-x<sub>1</sub>)<sup>2</sup> + (y<sub>2</sub>-y<sub>1</sub>)<sup>2</sup>, where (x<sub>1</sub>, y<sub>1</sub>) and (x<sub>2</sub>, y<sub>2</sub>) are the beginning and the end coordinates of the section, respectively. Then, using our desired speed we calculate what fraction of this length we travel (given by speed\*dt) and then also take this fraction to compute x and y (y is compute with the spline, not with the fraction). Since, the spline is a piece-wise cubic polynomial and we are using a linearization, the process is performed many times.

Now, the speed itself can only change between two points by a maximum of 0.8\*dt\*max_acceleration, which is by about 0.16m/s. The 0.8 coefficient is used as a safety margin, to make sure that we are sufficiently away from the maximum acceleration.

#### Collision detection _(Lines 151--180 in trajectory_generator.cpp)_

Even though the FSM should provide relatively safe targets, each trajectory is evaluated for possible collisions with other vehicles. For example there are cases, where some of the agents change lanes suddenly and have much lower speed than our car. In such cases we keep only the first five points from the previous trajectory to ensure smooth transition and rapidly decelerate to the speed of the agent. The collision detection function gives us the time to collision and the speed of the other vehicle.

In case we do not detect a collision, we return the original trajectory.
