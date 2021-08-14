# Self-Driving Car Engineer Nanodegree

---

## 1. Project: **Kidnapped Vehicle** 

The goals / steps of this project are the following:

* Implement a particle filter
* Localize the vehicle within the desired accuracy
* Run within the specified time of 100 seconds 

[Rubric](https://review.udacity.com/#!/rubrics/1965/view) points to be adressed for this writeup

[//]: # (Image References)

[image1]: ./output_images/success.png "Final successful run"

---

## 2. Libraries import & Constants definition  

This section gathers imports and constants for the project.
* Source https://github.com/udacity/CarND-Kidnapped-Vehicle-Project/
* "using std::normal_distribution;" requires "#include <random>" 

---

## 3. Implementation

  * Initialization
    * Create particles
    * Initialize particles X, Y, Theta given GPS coordinates
    * Add Gaussian noise according to GPS X, Y, Theta noise
  * Prediction
    * Update particles X, Y, Theta based on
      * dT, Velocity, Yaw rate
    * Add Gaussian noise according to X, Y, Theta measurement noise
  * Weight update: According to particle location
    * Filter landmarks out of sensor range using point to point distance
    * Reset particle weight to 1.0
    * Transform vehicle centric observations into particle centric observation
      * E.g. Vehicle observed LandMark 1 meter 45 degrees to the right is transformed to particle observed LandMark 1 meter 45 degrees to the right (same observation, but different reference source)
    * Find each observations closest (in range) map landmark
    * Compute each observations weight according to the Multivariate-Gaussian formula
    * Multiply each observations weight together to obtain particle weight
  * Resample
    * Retrieve maximum particle weight
    * Choose a random particle index and initialize beta to 0
    * Create each particle using the resample wheel
      * Generate a beta chosen between 0 and 2 * maximum weight
      * Decrement beta by current index weight until beta is inferior to the current index
      * Add current index particle to new sample
      * Repeat resample wheel until desired number of particle has been created

---

## 4. Report

## 4.1. Execution & Optimization

Execution: 
* Excessive X,Y error: Fixed manual distance calculation using dist helper function
* Excessive duration: Updated particle numbers, optimized for loops and comparisons

Optimize code: 
* Base working code: Failure (around 0.1 x/y 0.03 yaw error 130s)
* Include closest particle weight directly in map_observations / predictions loop: Failure
* Reduce number of particles to 500: Success (around 0.1 x/y 0.03 yaw error 70s)
* Nested for loops (around 0.3 x/y 0.03 yaw error 98s)
  * Don't know why we lost in precision... might be RNG... to investigate
* Remove forgotten "break;" in update weights: Failure (around 0.1 x/y 0.003 yaw error 116s)
* Add predictions to observations loop: Failure (around 0.1 x/y 0.003 yaw error 130s)
* Add pointer to map single_landmark_s: Failure precision issue, cannot fix
* Avoid using square roots and recomputing sin, cos or other operations: Failure (around 0.1 x/y 0.003 yaw error 110s)
* Tried to use vector index and pointers: Failure (around 0.1 x/y 0.003 yaw error 105s)
* Revert ans change to 750 particles: Success (around 0.1 x/y 0.003 yaw error 98s)

## 4.2. Final comment

The optimization task was challenging and led to a huge decrease of the number of lines of code.
There must be other optimizations that I do not know to reach below 100s with 1000 particles.
