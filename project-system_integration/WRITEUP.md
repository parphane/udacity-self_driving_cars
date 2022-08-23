# Self-Driving Car Engineer Nanodegree

---

## Project: **System integration** 

Drive the vehicle successfully in the simulator.

[Rubric](https://review.udacity.com/#!/rubrics/1972/view) points to be adressed for this writeup:
* Running the Code
  * The code is built successfully and connects to the simulator.
* Control and Planning
  * Controller commands are published to operate Carla’s throttle, brake, and steering.
  * Waypoints are published to plan Carla’s route around the track.
* Successful Navigation
  * Successfully navigate the full track more than once.
* Suggestions to make your project stand out
  * Utilize a detection and classification model for stop lights. The model should appropriately detect and classify stop lights at intersections at least 80% of the time. Make sure to provide instructions for obtaining the necessary model used, as well as any additional necessary dependencies and setup.
  * Waypoints and controls are adjusted based on detections. When approaching a red light, the vehicle should slow and come to a complete stop (if the light is still red). When the light switches back to green, the vehicle should accelerate back up to the desired speed. When approaching a green light, the vehicle should continue normally on its path.
  * Include a write-up concerning different models you tried out for the stop light detection, and results on those models.
  * Include a write-up concerning any data gathering efforts you performed for the stop lights, as well as any data augmentation.
  * Get as close to 100% accuracy as you can with stop light detection and classification - without just overfitting the simulator!

[//]: # (Image References)

[image1]: ./someimage.png "Some image..."

## Setup & Useful notes
 ### Workspace configuration
 #### Environment variables
 * Before we begin using ROS in a terminal, ensure that all of the environment variables are present.
   * Source the setup script provided by ROS: source /opt/ros/kinetic/setup.bash (or /home/workspace/ros_setup.sh)
 #### Project Python requirements
 * cd /home/workspace/CarND-Capstone
 * pip install -r requirements.txt 

 #### Project ROS requirements
 * Install missing dependency
   * rosdep check styx   
     * ... does not work
   * apt-get update
   * apt-get install ros-kinetic-dbw-mkz-msgs
     * E: Failed to fetch http://packages.ros.org/ros/ubuntu/pool/main/r/ros-kinetic-dbw-mkz-msgs/ros-kinetic-dbw-mkz-msgs_1.2.3-1xenial-20191214-055208+0000_amd64.deb  404  Not Found [IP: 64.50.236.52 80]"
   * To fix IP issue, added ROS package server (as per recommended here: https://stackoverflow.com/questions/61460527/how-can-solve-the-ros-installation-error-on-ubuntu)
     * sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
     * sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
     * sudo apt update (apt-get update)
   * Run install again, with new package repository:
     * apt-get install ros-kinetic-dbw-mkz-msgs
   * Install catkin_tools package to use catkin_build
     * sudo apt-get install python-catkin-tools
   * rosdep check styx
   * rosdep install -i styx 
   * rosdep check styx
   * cd ros
   * catkin_make
   * source devel/setup.sh
   * roslaunch launch/styx.launch


### Compilation and run

* Ensure GPU mode is activated
* Start VM and simulation
* Terminal
  * cd ros
  * catkin_make
  * source devel/setup.sh (needed only once per terminal)
  * roslaunch launch/styx.launch
* Monitor messages
  * Open a new terminal (and re-run source devel/setup.sh)
  * rostopic list
  * rostopic info /final_waypoints
  * rosmsg info styx_msgs/Lane

### Baseline code

#### waypoint_updater.py

## Reflection / Trial & Error

### Algorithm

### Trial & Error
* Where to begin from...
  * Updated waypoint_updater.py from course guidance
  * Fixed issue with missing "ros-kinetic-dbw-mkz-msgs" package
  * Run and fixed misc. issues... car is moving and points are displayed 

### Observations & Todo

### Vehicle run
