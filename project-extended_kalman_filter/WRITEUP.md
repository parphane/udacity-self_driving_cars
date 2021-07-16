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
   * Local - Windows (NOT WORKING ABANDONNED):
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
        

## 3. Useful notes

* Visual Studio Code
    * Markdown side by side view: Ctrl+K V

## 4. Processing flow

## 4.1. Measurement initialization
## 4.2. Prediction update
## 4.3. Measurement update
## 4.4. Utility functions
## 4.4.1 RMSE calculation 
Taken from course
## 4.4.1 Jacobian matrix
Taken from course

## 5. Test
## 5.1. Compilation

    
## 5.2. Execution
## 5.3. Efficiency



## 5. Final comment

## 5. Wayforward