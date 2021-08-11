# Self-Driving Car Engineer Nanodegree

---

## 1. Project: **Advanced lane finding** 

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

Note: Please refer to the Jupyter notebook for more detail and code on each section

[Rubric](https://review.udacity.com/#!/rubrics/571/view) points to be adressed for this writeup

[//]: # (Image References)

[image1]: ./output_images/display_chessboard_corners.png "Display chessboard cormers"
[image2]: ./output_images/undistorted.png "Undistorted image"
[image3]: ./output_images/undistorted_bridge.png "Undistorted bridge"
[image4]: ./output_images/blurred.png "Blurred image"
[image5]: ./output_images/dark_gray_binary.png "Dark gray filter"
[image6]: ./output_images/sx_sy_mag_dir_binary.png "Sobel X Y, Mag Dir filters"
[image7]: ./output_images/gradient_binary.png "Gradient combined filter"
[image8]: ./output_images/h_l_s_binary.png "H, L, S filter"
[image9]: ./output_images/hls_binary.png "HLS combined filter"
[image10]: ./output_images/l_u_v_binary.png "L U V filter"
[image11]: ./output_images/luv_binary.png "LUV combined filter"
[image12]: ./output_images/l_a_b_binary.png "L A B filter"
[image13]: ./output_images/lab_binary.png "LAB combined filter"
[image14]: ./output_images/color_binary.png "Color combined filter"
[image15]: ./output_images/combined_binary.png "Combined gradient and color filter"
[image16]: ./output_images/zone_of_interest.png "Zone of interest"
[image17]: ./output_images/warped_zoi.png "Warped zone of interest"
[image18]: ./output_images/warped_zoi_straight.png "Straight zone of interest"
[image19]: ./output_images/initial_histogram.png "Histogram L/R lane start"
[image20]: ./output_images/identified_lanes.png "Warped identified lanes"
[image21]: ./output_images/lane_curvature.png "Lane curvature"
[image22]: ./output_images/weighted_lanes.png "Overlay on image"
[image23]: ./output_images/project_video_overlay.png "Video project information overlay"
[image24]: ./output_images/challenge_video_overlay.png "Video challenge information overlay"

---

## 2. Libraries import & Constants definition  

This section gathers imports and constants for the notebook.

Tune the following:
* RUN_TEST_IMG = True
  * Activate the notebook single image processing / display
* RUN_TEST_VIDEO = True
  * Activate the notebook video processing / display
* RUN_TEST_VIDEO_HARD = False
  * Activate the notebook harder video processing / display (Spoiler alert: It might be disappointing!)

---

## 3. Camera calibration

Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.

The code for this step is contained in the first code cell of the IPython notebook located in "./examples/example.ipynb" (or in lines # through # of the file called `some_file.py`).  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result:

**Calibration preparation and algorithm explanation**
  * objectPoints are reference points in "world coordinates" that never "move" (because the reference is the pattern plan)
  * objectPoints coordinates are based on the pattern plan reference (0,0,0) point and a reference length
    * Top left corner of the chessboard is taken as reference in most tutorials
    * Reference length is "a square's side length" for the chessboard pattern
    * Z-coordinates are always 0 because the pattern is planar and the chessboard points depth do not change in the chessboard plan
  * objectPoints coordinates are put in relation to their imagePoints pixel coordinates counterpart during the camera calibration
  * cameraCalibration estimates the extrinsic ([R|t] matrix) parameters for camera motion around the static scene
  * cameraCalibration estimates the intrinsic parameters for focal length (fx, fy) and image center (cx, cy)
  * cameraCalibration tries to minimize the runs the re-projection error, that is, the total sum of squared distances between the observed feature/image points and the projected object points
  * **Note**<br/>
    * Currently, initialization of intrinsic parameters (when CV_CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for planar calibration patterns (where Z-coordinates of the object points must be all zeros). 3D calibration rigs can also be used as long as initial cameraMatrix is provided.<br/>
    * So if not providing camera focal length (fx, fy) and image center (cx, cy) intrinsic parameters, you have to use a planar (Z=0) calibration pattern.<br/>
    * Link to own comment, result of digging around https://stackoverflow.com/a/63432041/685996

**Comment**<br/>
Calibration image 1, 4 and 5 cannot be processed because the full chess is not visible.<br/>
Tried to reduce height by one for those images specifically but still did not work.

### 3.2. Display chessboard corners

![alt text][image1]

---

## 4. Distortion correction 
Apply a distortion correction to raw images.

### 4.1. Image selection

Select the image of your choice or let it be random by changing "img_test_path" variable

### 4.2 Undistort image

![alt text][image2]
![alt text][image3]

**Comment**<br/>
You can especially notice how the bridge you pass under is straightened in "error_0.jpg" image

## 5. Image filtering
Use color transforms, gradients, etc., to create a thresholded binary image.

**Image filters and algorithm explanation**<br/>
  * Tried out warping before filtering at the begining but abandonned idea (due to issues in filtering)
  * **Gradient filterings & combining** Sobel X/Y and Magnitude/Direction filters over a grayscale image
    * Filtered out darker colors using average grayscale to prevent road reparations from being picked up
    * --> ((mag_binary & dir_binary) | (grad_x_binary & grad_y_binary)) & gray_binary
  * **Color properties filterings & combining** 
    * Gathered the idea from another projects using LAB/LUV: https://github.com/JustinHeaton/Advanced-Lane-Finding BUT did not remove the gradient as he did in his project!
    * Final filter HLS | LUV | LAB with only HLS S (brightness), LUV L (white/brightness) and LAB B (yellow) channels filters
  * **HLS color scheme** Easier to filter shape contours, no matter their color, using lightness/saturation gradients
      * **Hue:** Degree on the color wheel from 0 to 360. 0 is red, 120 is green, 240 is blue.
      * **Saturation** Percentage value; 0% means a shade of gray and 100% is the full color.
      * **Lightness** Percentage; 0% is black, 100% is white.
  * **(CIE)LUV color scheme** Efficient at detecting white
      * See components determination there: https://en.wikipedia.org/wiki/CIELUV
  * **(CIE)LAB color scheme** Efficient at detecting yellow
      * See components determination there: https://en.wikipedia.org/wiki/CIELAB_color_space
  * **Final combination** Gradient | Color

### 5.1. Global pre-processing

Functions to apply Gaussian blur.
Note: Average saturation functions were taken from my traffic light ckassifier project but not used here

#### 5.1.1 Compute pre-processing

Kernel size for all filtering set to 3

#### 5.1.2 Display pre-processing

![alt text][image4]

### 5.2. Gradient filters

Functions for gradient filtering on image
* Average lower 3rd of image gray scale and filter anything below average*1.15
* Combine X AND Y sobel
* Combine Magnitude AND Direciton threshoolds
* Combine both with a OR
* Filter out darker zones by combining the resulting filter AND the gray filter
**Note:** Update directional threshold from 0.7,1.3 to -0.3,0.3 if using bird's eye view (0=vertical line, from -pi/2 to pi/2)

#### 5.2.1 Compute gradient filter

Values for thresholds: 
* abs_thresh_x=(35, 100)
* abs_thresh_y=(30, 255)
* mag_thresh=(30, 255)
* dir_thresh=(0.7, 1.3)
* gray treshold is set by default to average lower 3rd of image grayscale times 1.15 after testing on bright images with road reparation marks

#### 5.2.2 Display gradient filter

![alt text][image5]
![alt text][image6]
![alt text][image7]

### 5.3. Color channels filters

Got LUV and LAB additional filters idea from https://towardsdatascience.com/computer-vision-for-lane-finding-24ea77f25209

The code here combines each color channel of HLS, LUV, LAB using a AND then combines all the color coding filters using a OR

#### 5.3.1 Compute color filter

Thresholds chosen from above mentioned sources images that were not well filtered (road repair, bridge):
* hls_thresh=[(0, 255), (0, 255), (105, 255)] # [(Hue), (Lightness), (Saturation)]
* luv_thresh=[(205, 255), (0, 255), (0, 255)] # [(L), (u), (v)]
* lab_thresh=[(0, 255), (0, 255), (135, 200)] # [(L), (a), (b)]

#### 5.3.2 Display color filter
##### HLS
![alt text][image8]
![alt text][image9]
##### LUV
![alt text][image10]
![alt text][image11]
##### LAB
![alt text][image12]
![alt text][image13]
##### Combined HLS LUV LAB
![alt text][image14]

### 5.4. Complete filter pipeline

The code here just applies the preprocessing (blur) combines gradient and color filter

#### 5.4.1 Display pipeline result filter
![alt text][image15]

## 6. Region of interest & Perspective transform

The code for the perspective transform uses determine_region_of_interest which has parameters:
* apex_width_pct: Width of the apex in PCT (0 to 1)
* apex_height_pct: Height of the apex, from the bottom, in PCT (0 to 1)
* bottom_width_pct: Width of the bottom in PCT (0 to 1)

The region_of_interest_mask function is from the course but is not used here

####  6.2.1. Determine region of interest

Parameters for zone of interest:
* bottom_width_pct=0.85
* apex_height_pct=0.36
* apex_width_pct=0.113

![alt text][image16]

### 6.3. Perspective transform

The code for the perspective transform uses the zone of interest top and bottom coordinates and stretches them to an image the same size as the initial image.
* determine_dst_region_of_interest: Determine the destination points
* birds_eye_view_warp: Warp the zone of interest using source and destination coordinates

####  6.3.1. Determine transforms

Code executed to determine destination vertices and warp image

####  6.3.2. Display bird's ete view

 ![alt text][image17]
 
 **Note:** To ensure that the ZOI is correclty warped, I used a straight line image
 
 ![alt text][image18]
  
## 7. Lane detection and boundaries

* find_lr_x_base: Returns the position of the left / right lane based on a histogram analysis
  * window: Window to be searched for in the image (e.g. search only window 3 from bottom)
  * nwindows: Number of windows to divide image for histogram

* find_lane_pixels: Finds the lane pixels and associated rectangles
  * nwindows: Number of windows to divide image for sliding rectangles 
  * margin: Margin of the rectangles
  * minpix: Minimum number of pixels to be found to recenter rectangle
    * If minpix could not be found from previous rectangle center, retries using find_lr_x_base for L and/or R
    * If minpix could not be found, increases a left or right "missed rectangle" counter use to search further and further left or right for the lane after each iteration
    * "missed rectangle" counter is reset when minimum amount of pixel for respectively L or R is found

* find_lane_pixels_from_prior: Search lanes around already provided polynomial factors

### 7.1. Detect lanes

Parameters for lane finding
* nwindows=9
* margin=60
* minpix=50

### 7.2. Display lanes lanes

![alt text][image19]
![alt text][image20]

## 8. Lane curvature
Determine the curvature of the lane and vehicle position with respect to center.

* measure_curvature_real: Uses real life Y and X pixels meter value to compute curvature

### 8.1 Determine lane curvature

Almost same height as course, reusing parameters
* ym_per_pix=30/720
* xm_per_pix=3.7/675 Based on the fact that average lane width in birds eye view is 675px

### 8.2 Overlay lane curvature

![alt text][image21]

## 9. Vehicle position
Detect the vehicle position against the center in meters

* determine_vehicle_position: 
 * Compares the center of the lanes against the center of the zone of interest to find the distance of the car compared to the middles of the road
 * Uses the fact that USA lanes are 3.7 meters wide to determine a more precise X pixel per meter value
 
### 9.1 Determine vehicle position

In the presented image, vehicle is 0.36 metters away from lane center

## 10. Original image and lane superposition
Warp the detected lane boundaries back onto the original image.
Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

* unwarp_lanes: Uses the inverse tranformation matrix to put the computed lane points (plot_y, left_fitx, right_fitx) back onto the undistorted image
* overlay_text: Used in the video to create a curvature and center distance text mask
* weighted_img: Used to overlay undistorted image, text and lanes onto each other

![alt text][image22]

## 11. Full pipeline for successive processing

Contains the code for video processing

* average_prev_x: Maintains a rolling buffer of lanes X last <depth> values and returns the average X position
  * Used to minor the effects of glitches (hard images to filter) during videos 

* compute_fitx: Function to compute the X based on polynomial fits and Y values

* process_image: Whole video processing pipeline with "memory" using globals between iterations
  * Computes the source and destination vertices once
  * Undistorts image
  * Applies gradient/color filter
  * Warps filtered image
  * If polynomial fits are already present, computes left and right lane pixels positions using find_lane_pixels_from_prior
    * If any of the lanes do not have at least the minpix*nwidows pixels, falls back to find_lane_pixels 
    * If NO polynomial fits are already present, uses find_lane_pixels
  * Fits polynomial parameters to the found lanes pixels
  * Feed the rolling buffer and computes average/smoothed left and right X positions
  * average_prev_x_depth = 20
    * If rolling buffer has attained desired size, use average left and right X positions to compute polynomial parameters
  * Measure curvature and distance from center
  * lanes_dst_std_dev_max = 80
    * Maximum allowed standard deviation of lane width to reset find_lane_pixels_from_prior to find_lane_pixels 
  * lanes_avg_rel_dst_min = 625
    * Minimum allowed width of lane for the bottom 3rd of warped ZOI to reset find_lane_pixels_from_prior to find_lane_pixels
  * lanes_avg_rel_dst_max = 725
    * Maximum allowed width of lane for the bottom 3rd of warped ZOI to reset find_lane_pixels_from_prior to find_lane_pixels
  * Unwarp lanes and text onto undistorted image
  * Return image with text and lanes

* save_images: Function used to save and debug at different point in the video processing

## 12. Video processing

### 12.1. Normal video

**See: project_video_processed.mp4**
![alt text][image23]

### 12.2. Challenge Harder video

**See: challenge_video_processed.mp4**
![alt text][image24]

### 12.3. Harder video

There is still work to be done here... not useful to display

## Final comment

This was a difficult exercice and I spent more than 3 weeks on it:
- Coming up with the first version of the full pipeline
- Pondering if I should do image filtering before or after the perspective transform
- Trying to tune the parameters so as to find lanes when under the bridge or when the pavement line crosses the view
- Trying to find new filtering methods to help find lanes:
  * New color patterns
  * Average luminosity
  * Reset rectangle position close to last rectangle, with increasing magnitude

Issues encountered:
- Incorrect direction filter when trying to filter on warped image
- Incorrect BGR / RGB beween image / video processing
- Incorrect filter channel when using color filtering
- Full reset of rectangle position based on histogram put rectangle too far from each other, detecting the other lane


## Wayforward
As mentionned along the notebook, here are the topics that should be considered to enhance the lane finding:
- Use HLS / LUV / LAB channels averages on the warped zone of interest to better detect lanes in sunny / shade situations
- Perform the filtering on a per rectangle basis, for example, divide the zone of interest in X rows and performs specific filtering based on average color channels values
- Reduce zone of interest length when there are steep curves
- Non functional
  * Display image and filters as seen on some other projects
  * Refactor whole code to make it more readable