#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /** TODO: Finish initializing the FusionEKF. **/
  // 14. Laser Measurements Part 4
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // Jacobian Matrix Part 1
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; 

  /** TODO: Set the process and measurement noises **/
  // Set the process noise/covariance matrix Q (10. Process Covariance Matrix)
  // Values will be calculated during prediction update, depending on time
  MatrixXd Q = MatrixXd(4, 4);

  // Set measurement noise/covariance matrix R (13. Laser Measurements Part 3)
  // Values will be calculated during measurement update, depending on sensor
  // Already set above R_radar/R_laser
  // Using laser as default for init.

  // Estimate matrix, x and y position and speed
  VectorXd x = MatrixXd(4);

  // Uncertainty covariance matrix, x and y speed and position uncertainty
  MatrixXd P = MatrixXd(4, 4);

  // State transition matrix (9. State Prediction), used to update:
  // - Position x based on speed (we assume velocity is constant)
  // - Uncertainty covariance P to update estimation (update x based on speed)  
  MatrixXd F = MatrixXd(4, 4);

  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);  

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */   

    // first measurement
    cout << "EKF: " << endl;
    // REMOVED: Already initialized in constructor
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
