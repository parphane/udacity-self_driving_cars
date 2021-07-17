# Self-Driving Car Engineer Nanodegree

---

## 1. Project: **Advanced lane finding** 

The goals / steps of this project are the following:

* Follow the general Kalman filter steps:
    * Initialize: Use the first measurements to initialize the state vectors and covariance matrices.
    * Prediction update: Predict object position to the current timestep
    * Measurement update: Update the prediction using the new measurement
        *  Call the correct measurement function for a given sensor type: LIDAR/RADAR
* Compile without errors with cmake and make. 
* Meet the target RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt"
* Avoid unnecessary calculations
    * Running the exact same calculation repeatedly when you can run it once, store the value and then reuse the value later.
    * Loops that run too many times.
    * Creating unnecessarily complex data structures when simpler structures work quivalently.
    * Unnecessary control flow checks.

[Rubric](https://review.udacity.com/#!/rubrics/1962/view) points to be adressed for this writeup

[//]: # (Image References)

[image1]: ./output_images/display_chessboard_corners.png "Display chessboard"

---

## 2. Environment setup

* Workspace
   * Udacity: [Udacity VM](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/3feb3671-6252-4c25-adf0-e963af4d9d4a/concepts/a5facabd-2879-4b98-b7fd-f47b75026069)
   * Local - Windows (NOT WORKING - ABANDONNED):
       * [MinGW-w64](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download)
           * Options
               * Version: 8.1.0
               * Architecture: **x86_64**
               * Threads: posix
               * Exception: seh
               * Build revision: 0
           * Test environment:
               * g++ --version
               * gdb --version
       * [CMake](https://cmake.org/download/)
           * Version: 3.21.0
           * Scope: Add to PATH for all users
       * [Visual studio code](https://code.visualstudio.com/download)
         *  Install [C/C++ extension for VS Code](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (Ctrl+Shift+X)
       * [uWebSockets](https://github.com/uNetworking/uWebSockets)           
           * Clone content (outside of this repository)
           * Add the [CMakeLists.txt](https://github.com/jchdng/uWebSockets/blob/master/CMakeLists.txt)
           * Build
               * cmake .. -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=g++ -DCMAKE_CC_COMPILER=gcc -DCMAKE_C_COMPILER=gcc -DCMAKE_MAKE_PROGRAM=mingw32-make
        * [openssl]
            * Install [Chocolatey](https://chocolatey.org/)
            * Install openssl with command `choco install openssl`
        * [zlib](https://zlib.net/)
            * Could not compile zlib with CMake 
        * [libuv](https://libuv.org/)
            * To be investigated
        
---
## 3. Useful notes

* Visual Studio Code
    * Markdown side by side view: Ctrl+K V
---
## 4. Processing flow
* u: Motion vector = 0 (Captured by process noise)

## 4.1. Measurement initialization
* Initialize Lidar measurement function H [14. Laser Measurements Part 4](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/252f0093-48ac-4122-aaae-f10214d30320)
* Declare Radar measurement function Hj and leave it empty. Matrix has to be computed with measurements [21. EKF Algorithm Generalization](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/f3b2b918-00d5-4af0-9363-410d01b0a1a7)
* Declare process noise/covariance matrix Q and leave it empty. Matrix has to be computed with time elapsed between measurements. [10. Process Covariance Matrix](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/1ac6e0ac-1809-4864-b58f-870d6bda9b25)
* Measurement noise/covariance matrix R are already given. [13. Laser Measurements Part 3](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/f1cef488-6a2d-484d-a30d-8caa4fd082fd)
* Declare estimate matrix x and leave it empty. Matrix has to be filled with first state and updated during measurement and prediction updates. [14. Laser Measurements Part 4](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/252f0093-48ac-4122-aaae-f10214d30320)
* Declare uncertainty covariance matrix P and leave it empty. Matrix has to be updated with time elapsed between measurements.
* Declare state transition matrix F and leave it empty. Matrix has to be updated during measurement update.

## 4.2. Prediction update
* $$x' = Fx$$
* $$P' = FPF^{T} + Q$$

* x: Estimate matrix, x and y position and, x and y velocity position and velocity
$$x = \begin{bmatrix}
​p_{x}\\
​p_{y}\\
​p_{x}\\
​p_{y}\\
\end{bmatrix}$$

* P: Uncertainty covariance matrix, x and y speed and position uncertainty
  * Lidar: Measures position, not speed
  * Radar: Measures position and speed
$$P_{Lidar} = \begin{bmatrix}
1 & 0 & 0 ​& 0\\
0 & 1 & 0 & 0 ​\\
0 & 0 & 1000 & 0 ​\\
0 & 0 & 0 & 1000 ​\\
\end{bmatrix}$$

$$P_{Radar} = \begin{bmatrix}
1 & 0 & 0 ​& 0\\
0 & 1 & 0 & 0 ​\\
0 & 0 & 1 & 0 ​\\
0 & 0 & 0 & 1 ​\\
\end{bmatrix}$$

* F/Fj: State transition matrix, used to update:
  * Position x based on speed (we assume velocity is constant)
  * Uncertainty covariance P to update estimation (update x based on speed)
$$F = \begin{bmatrix}
1 & 0 & Δt ​& 0\\
0 & 1 & 0 & Δt ​\\
0 & 0 & 1 & 0 ​\\
0 & 0 & 0 & 1 ​\\
\end{bmatrix}$$

* Q: Process noise/covariance matrix
$$Q = \begin{bmatrix}
\frac{Δt^{4}}{4}\sigma^{2}_{ax} & 0 & \frac{Δt^{3}}{2}\sigma^{2}_{ax} ​& 0\\
0 & \frac{Δt^{4}}{4}\sigma^{2}_{ay} & 0 & \frac{Δt^{3}}{2}\sigma^{2}_{ay} ​\\
\frac{Δt^{3}}{2}\sigma^{2}_{ax} & 0 & Δt^{2}\sigma^{2}_{ax} & 0 ​\\
0 & \frac{Δt^{3}}{2}\sigma^{2}_{ay} & 0 & Δt^{2}\sigma^{2}_{ay} ​\\
\end{bmatrix}$$

## 4.3. Measurement update
[21. EKF Algorithm Generalization](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/f3b2b918-00d5-4af0-9363-410d01b0a1a7)
* $$Error(Lidar): y = z - Hx'$$
* $$Error(Radar): y = z - h(x')$$
* $$S = HP'H^{T} + R$$
* $$Kalman gain: K = P'H^{T}S^{-1}$$
* $$Estimate: x = x' + Ky$$
* $$Uncertainty: P = (I-KH)P'$$
* z: Measurement
$$z_{Lidar} = \begin{bmatrix}
p_{x}\\
p_{y}\\
\end{bmatrix}$$
$$Polar: z_{Radar} = \begin{bmatrix}
\rho\\
\phi\\
\dot\rho\\
\end{bmatrix}
\rarr Cartesian: z_{Radar} = \begin{bmatrix}
p_{x}=cos(\phi) * \rho\\
p_{y}=sin(\phi) * \rho\\
v_{x}=cos(\phi) * \dot\rho\\
v_{y}=sin(\phi) * \dot\rho\\
\end{bmatrix}
$$
* H/Hj: Measurement function
$$H_{(Lidar)} = \begin{bmatrix}
1 & 0 & 0 ​& 0\\
0 & 1 & 0 & 0\\
\end{bmatrix}$$

$$H_{j (Radar)} = \begin{bmatrix}
\frac{p_{x}}{\sqrt{p^{2}_{x}+p^{2}_{y}}} & \frac{p_{y}}{\sqrt{p^{2}_{x}+p^{2}_{y}}} & 0 ​& 0\\
-\frac{p_{y}}{p^{2}_{x}+p^{2}_{y}} & \frac{p_{y}}{p^{2}_{x}+p^{2}_{y}} & 0 & 0\\
\frac{p_{y}(v_{x}p_{y}-v_{y}p_{x})}{(p^{2}_{x}+p^{2}_{y})^{3/2}} & \frac{p_{x}(v_{y}p_{x}-v_{x}p_{y})}{(p^{2}_{x}+p^{2}_{y})^{3/2}} & \frac{p_{x}}{\sqrt{p^{2}_{x}+p^{2}_{y}}} & \frac{p_{y}}{\sqrt{p^{2}_{x}+p^{2}_{y}}}\\
\end{bmatrix}$$

* R: Measurement noise
$$R_{Lidar} = \begin{bmatrix}
0.0225 & 0\\
0 & 0.0225\\
\end{bmatrix}$$

$$R_{Radar} = \begin{bmatrix}
0.09 & 0 & 0\\
0 & 0.0009 & 0\\
0 & 0 & 0.09\\
\end{bmatrix}$$

* I: identity matrix
$$I = \begin{bmatrix}
1 & 0 & 0 & ...\\
0 & 1 & 0 & ...\\
0 & 0 & 1 & ...\\
... & ... & ... & ...\\
\end{bmatrix}$$

## 4.4. Utility functions
## 4.4.1 Jacobian matrix
Taken from course [20. Jacobian Matrix Part 2](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/08fc65c1-04d9-45d3-8a98-abf7bb072dc2)

## 4.4.2 RMSE calculation 
Taken from course [24. Evaluating KF Performance 2](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/95d62426-4da9-49a6-9195-603e0f81d3f1/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/c46a47f0-7cdc-4e49-b225-5134b438255a)

## 4.4.3 Debug print
Added for 1 liner debug messages that can be set on/off
---
## 5. Test
## 5.1. Compilation
* Compile without warnings
* Example of warning fixed during debugging:
  * Comparison between int and unsigned int
  * Unused variables
  * 
    
## 5.2. Execution
* Added debug function in tools to find runtime bug at first measurement: Identity matrix had wrong size
  * Used size of x
  * Fixed to use size of P
* Estimation would drift off actual position
  * Played with px and py flooring in UpdateEKF
  * Removed px flooring for atan2
* RMSE
  * px: 0.0974
  * py: 0.0855
  * vx: 0.4517
  * vy: 0.4404

## 5.3. Efficiency
* Declared most of the vectors once so they do not need to be allocated/de-allocated in memory at each run
* Comuted some values once only (e.g. Identity matrix, dt powers)

## 5. Wayforward
* Optimize local variable delcaration and use