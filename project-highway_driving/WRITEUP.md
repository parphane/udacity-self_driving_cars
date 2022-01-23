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
* * Contains the lambda that is called around every 100ms to generate a path
* * Is where variables that should persist between path generation should be passed
* Telemetry variables
* * car_x, car_y: Vehicle position in cartesian coordinates
* * car_s, car_d: Vehicle position in Frenet coordinates
* * car_yaw: Vehicle yaw / where it is heading
* * car_speed: Vehicle speed, averaged over 1s
* * previous_path_x, previous_path_y: Non-consumed points of the previous path given to the Planner  
* * end_path_s, end_path_d: Previous path's end s and d values
* * sensor_fusion: Information on other vehicles around, going the same direction
* Return variables
* * next_x_vals, next_y_vals: Path in cartesian coordinates to be published in msgJson["next_x"] and msgJson["next_y"]

## Reflection / Trial & Error

### Algorithm

### Trial & Error



### Observations & Todo
1 There is an edge case when 1 vehicle is in front of the car and another in the adjacent lane:
* * Ego vehicle will change lane multiple times resulting in being stuck in the middle of 2 lanes
* * This could be fixed by adding a state machine and making sure the line change is complete before changing lane again
2 Speed of vehicle in adjacent lanes is not taken into account
* * This could result in the vehicle changing lane when the next lane vehicle is further but ending up being in a worse situation
3 Acceleration/Deceleration when behind a vehicle isn't smooth
* * The vehicle speed occilates when following another vehicle
* * Effort should be put on aligning with front vehicle speed
4 Vehicle is not able to perform advanced/sharp maneuvers
* * Cannot perform sharp maneuvers to change lane
* * Cannot slow down if stuck for too long to evaluate other lane change opportunities

### Vehicle run
* Best run
* * Distance: 5.7 miles
* * Time: 7 min 45 s
* * See video: Run.mp4
![End of run][image1]