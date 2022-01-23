# Self-Driving Car Engineer Nanodegree

---

## Project: **Highway driving** 

Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

[Rubric](https://review.udacity.com/#!/rubrics/1971/view) points to be adressed for this writeup:
* The code compiles correctly.
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes
* There is a reflection on how to generate paths.

[//]: # (Image References)

[image1]: ./RunEnd.png "End of longest run"

## Setup & Useful notes

### Compilation and run

* Ensure GPU mode is activated
* Start VM and simulation
* mkdir build && cd build
* cmake .. && make
* ./path_planning

### Baseline code
* h.onMessage
  * Contains the lambda that is called around every 100ms to generate a path
  * Is where variables that should persist between path generation should be passed
* Telemetry variables
  * car_x, car_y: Vehicle position in cartesian coordinates
  * car_s, car_d: Vehicle position in Frenet coordinates
  * car_yaw: Vehicle yaw / where it is heading
  * car_speed: Vehicle speed, averaged over 1s
  * previous_path_x, previous_path_y: Non-consumed points of the previous path given to the Planner  
  * end_path_s, end_path_d: Previous path's end s and d values
  * sensor_fusion: Information on other vehicles around, going the same direction
* Return variables
  * next_x_vals, next_y_vals: Path in cartesian coordinates to be published in msgJson["next_x"] and msgJson["next_y"]

## Reflection / Trial & Error

### Algorithm
* Check for vehicle in front
  * Request a lane change if vehicle is in similar lane and distance is < 60 meters
  * Request braking if vehicle is in similar lane and distance is < 30 meters
    * Memorize front vehicle speed
  * Request emergency braking if vehicle is in similar lane and distance is < 15 meters
* Check if lane change is possible left or right
  * If at leftmost/rightmost edges, forbid left/right lane change
  * Find rear/front vehicle in left/right lane
    * If ego vehicle is less than +/-15 meters from rear/front vehicles, forbid left/right lane change
    * Memorize closest front vehicle distance
  * If closest left/right lane vehicle is closer than 60 meters, forbid left/right lane change
    * Note: Could change to comparison between current lane and left/right lane vehicle distance instead of 60m
* Determine best lane to change (if left/right change not forbidden)
  * If both left/right have no vehicle up front, privilege fast lane (left)
  * Else privilege lane with no vehicle
  * Else, privilege lane with furthest vehicle
* Accelerate/Decelerate
  * If emergency braking is requested, apply full brakes
  * If too close to front vehicle and ego vehicle speed is above that of fron vehicle, smooth deceleration
  * Else, accelerate up to maximum speed
  * Note: Step acceleration has been calculated not to exceed limit
* Determine path
  * Determine vehicle location
    * If not enough path points (2) to compute previous vehicle location, use available reference information
    * Else, compute previous location
  * Construct path
    * Spline construction
      * Add ego vehicle location to spline points collection
      * Determine 3 spline points in front of the vehicle that will be located in the correct targeted lane
      * Shift spline points map to vehicle coordinates and create the spline (containing 4 points now)
    * Add previous non-consumed path points to path
    * Compute next path points, up to path size
      * Compute spline bird's eye view length
      * Split length in segments of distance travelled by the ego vehicle in 0.20ms according to its current speed
      * Given the computed X distance, obtain Y from the spline function (vehicle coordinates)
      * Compute map coordinates of points
      * Add points to the path
  * Send path back for execution

### Trial & Error
Majors issues met during development:
* x_ref / ref_x & y_ref / ref_y mixup:
  * In the Q&A video, the tutors provide a piece of code with similar names.
  * Variables were mixed when reusing code, resulting in vehicle not moving at all.
* Vehicle won't change to right lane when it is the best choice
  * Right/Left copy paste error resulted in the vehicle changing left lane when the right lane was a better choice
* Vehicle cuts turns/Smooths turns too much
  * In the Q&A video, collision sets car_s to end_path_s  but never sets it back to j[1]["s"] for path determination
* Vehicle won't brake fast enough when cut by another vehicle
  * Deceleration was lowered to smooth vehicle speed when following another vehicle
  * This resulted in edge cases where ego vehicle would collide with other vehicles cutting its lane
  * Added an emergency braking portion when extremely close to use the maximum allowable deceleration
* Left and right distance for lane change distance
  * Tuned rear/front left and right minimum vehicle distance to allow lane change: 5 7.5 10 15
  * 15 resulted in no observable collisions over the duration of driving 

### Observations & Todo
1 There is an edge case when 1 vehicle is in front of the car and another in the adjacent lane:
  * Ego vehicle will change lane multiple times resulting in being stuck in the middle of 2 lanes
  * This could be fixed by adding a state machine and making sure the line change is complete before changing lane again

2 Speed of vehicle in adjacent lanes is not taken into account
  * This could result in the vehicle changing lane when the next lane vehicle is further but ending up being in a worse situation

3 Acceleration/Deceleration when behind a vehicle isn't smooth
  * The vehicle speed occilates when following another vehicle
  * Effort should be put on aligning with front vehicle speed

4 Vehicle is not able to perform advanced/sharp maneuvers
  * Cannot perform sharp maneuvers to change lane
  * Cannot slow down if stuck for too long to evaluate other lane change opportunities

### Vehicle run
* Best run
  * Distance: 5.7 miles
  * Time: 7 min 45 s
  * See video: Run.mp4
![End of run][image1]
