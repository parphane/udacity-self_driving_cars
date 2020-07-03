# **Finding Lane Lines on the Road** 

## Objectives
[//]: # (Image References)

[initial_test_img]: ./test_images/solidWhiteRight.jpg "Initial test image"
[initial_test_img_processed]: ./examples/laneLines_thirdPass.jpg "Initial test image processed"
[image_process]: ./examples/image_process.png "Image process"

* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report

The initial and processed images will look like this:

| Initial | Processed |
| :---: | :---: |
| ![alt text][initial_test_img] | ![alt text][initial_test_img_processed] |


## Side notes

* Imported all files fromt he notebook using the zip function
  * Jupyter > New > Terminal
  * zip -r project-finding_lane_lines CarND-LaneLines-P1
* Used a local Anaconda / Jupyter notebook to work
  * Installed non Anaconda (e.g. moviepy) packages
    * Environments > <environment> > Open terminal > pip install moviepy
* Worked on my private gitHub repository
  * https://github.com/parphane/udacity-self_driving_cars

---

## Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of the following major steps:
* Convert the image to grayscale
* Apply a Gaussian blur to smooth gray transitions (suppress gray scale glitches/noise)
  * Blur kernel set to average on a 5 pixels matrix
* Apply a Canny edge detection to detect grayscale transitions in the image and hence, detect edges
  * Gray low threshold set to 100  and high threshold to 150
* Define a quadrilateral zone of interest on the image and ignore the rest
  * Ignore 60% of the image top
  * Top width set to 15% of the image width
  * Top centered on the image width
  * Bottom set to each bottom edge of the image  
* Apply a Hough transform to get the lines
  * Distance sampling rate Rho=2
  * Angle sampling theta=np.pi/180
  * Number of (x, y space) points that have to intersect (Hough space) Threshold 15
  * Minimal acceptable line length 30
  * Maximum line gap (Hough space circle area for intersection) 20
* Find and average left and right lanes
  * Filter lines with an inappropriate flat slope between -0.4 or 0.4 x/y pixels
    * Also filter too slopes above 0.8 or below -0.8 to prevent -inf / +inf slope or offset values
    * Filters were found by displaying slopes and visually determining an average for either left or right 
  * Put lanes in right/left bucket depending on positive/negative slope
  * Compute the respective right/left average slope by summing / dividing slopes
  * Compute the respective right/left average Y offset by summing / dividing each offset
    * y = ax+b --> b = y - ax
  * Once slope and offset calculated, compute the lane top and bottom X
    * Use top Y (lowest Y from either left or right lines) value to compute top X
    * Use bottom Y (maximum Y value, from image Y size) value to compute bottom X  
    * y = ax+b --> x = (y - b)/a 
* Overlay the lanes on the initial image
  * Using the default overlay parameters α=0.8, β=1., γ=0.
* Return the finall image 

| Image process |
| :---: |
| ![alt text][image_process] |

### 2. Identify potential shortcomings with your current pipeline

Potential shortcomings: 
* Overall image is very dark/bright and grayscale cannot filter properly
* Steep turn and lane is not straight but curved
* No lanes!?

### 3. Suggest possible improvements to your pipeline

Potential improvements:
* Separate left/right lane detection
  * Compute the average gray scale and improve the gray filter to float around the average
* Use HSV domain
* Use a Kalman filter on the lane slope and offset to filter detection glitches