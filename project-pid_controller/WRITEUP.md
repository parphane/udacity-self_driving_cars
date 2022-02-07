# Self-Driving Car Engineer Nanodegree

---

## Project: **PID Controller** 

Implement a PID controller in C++ to maneuver the vehicle around the track!.

[Rubric](https://review.udacity.com/#!/rubrics/1972/view) points to be adressed for this writeup:
* Your code should compile.
* The PID procedure follows what was taught in the lessons.
* Describe the effect each of the P, I, D components had in your implementation.
* Describe how the final hyperparameters were chosen.
* The vehicle must successfully drive a lap around the track.

[//]: # (Image References)

[image1]: ./someimage.png "Some image..."

## Setup & Useful notes

### Compilation and run

* Ensure GPU mode is activated
* Start VM and simulation
* mkdir build && cd build
* cmake .. && make
* ./pid

### Baseline code

#### main.cpp
* main()
  * Initialization of PID variables
  * h.onMessage
    * Lambda called around every 100ms to calculate steering value
    * Passes variables that persist between each onMessage event
#### pid.cpp
* UpdateError
  * Calculate PID parameter respective error based on lane center
    * Setpoint (SP) is lane center
    * Process Variable (PV) is distance from lane center   
* TotalError
  * Calculate steer angle based on PID parameters and respective errors
    * Manipulated variable (MV) or Control Variable (CV) is steer angle

## Reflection / Trial & Error

### Algorithm
* Some step
  * Some explanation

### Trial & Error
Majors issues met during development:
* Not understanding that TotalError is equivalent to calculating steer angle
  * Added then removed variables once code logic was understood...
  * First compilation and run with PID all set to 1
* Where to begin from...
  * Added a logic to pass PID as parameters to avoid recompiling at each manual iteration
  * Tuned, P, I and D manually to obtain an "acceptable" behavior, performing a manual Twiddle algorithm
    * Set P to 0.1 and I and D to 0.
    * Increase/Decrease P by 0.1 and observe behavior
    * Keep best P value and Increase/Decrease again until acceptable
    * Repeat for I and D...

### Observations & Todo
1 Some observation
  * Some thing TODO in the future if you are not lazy

### Vehicle run
* Some relevant metric

 Some image "![End of run][image1]"