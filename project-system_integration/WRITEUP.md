# Self-Driving Car Engineer Nanodegree

---

## Project: **System integration** 

Drive the vehicle successfully in the simulator.

[Rubric](https://review.udacity.com/#!/rubrics/3058/view) points to be adressed for this writeup:
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

<details>
  <summary>Local run configuration</summary>
 
   ### Workspace configuration
 
Note: Skip this part if using online workspace, but be sure to activate GPU mode, else catkin_make will not work

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
   * Source ROS comands
     * source ~/../opt/ros/kinetic/setup.bash
   * rosdep check styx
   * rosdep install -i styx 
   * rosdep check styx
   * cd ros
   * catkin_make
   * source devel/setup.sh
   * roslaunch launch/styx.launch
</details>

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
  * Listen to topics
    * rostopic echo /final_waypoints
    * rostopic echo /rosout --filter "m.level == 2" (for log INFO messages)

### Architecture & Code

#### Architecture 


#### waypoint_updater.py
* Function: Publish waypoints to the Car/Simulator to control throttle/braking/steering
* LOOKAHEAD_WPS: Maximum number of waypoints to compute and publish for navigation
  * Set to 50 instead of the default 200 to prevent execution slow down, delayed messages and thus car control failing
* MAX_DECEL: Maximum allowed deceleration
* loop:
  * Publishes computed waypoints, if any
  * slow_warn: Variable added to publish warning messages every 10 iterations
* get_closest_waypoint_id: Get the closest waypoint received from base_waypoint topic to start lane generation from
* generate_lane: Generate the lane starting from the closest waypoint and adding up to LOOKAHEAD_WPS points
  * Decelerates if the light stop line index is between the closest and furthest waypoints 
* decelerate_waypoints: Stops the car 2 waypoints before the stop line by updating the base waypoints
* distance: Computes distance between 2 waypoints for use in velocity determination
* publish_waypoints: Publish the waypoints
#### tl_detector.py
* Function: Detects traffic lights and their color
* image_cb: Receives images from the camera for traffic light deteciton
* process_traffic_lights: 
  * Determines the closest traffic light and its color from "process_traffic_lights"
  * Confirms traffic light color after 10 loops to prevent glitches
* process_traffic_lights: 
  * Using the base waypoints and traffic light positions, determine which traffic light is the closest in front of the vehicle 
  * Determines the color of the traffic light using internal light state (TODO: Use camera image instead)
#### waypoint_updater.py

## Trial & Error

* Where to begin from...
  * Updated waypoint_updater.py from course guidance
  * Fixed issue with missing "ros-kinetic-dbw-mkz-msgs" package
  * Run and fixed misc. issues... car is moving and points are displayed 
  * Fixed LOOKAHEAD_WPS to 50 down from 200 to prevent "lag" in car control and car swerving out of the lane

## Observations & Todo
* Update waypoint_follower as suggested by tutorial
* Update traffic light detection with model developed in previous classes
