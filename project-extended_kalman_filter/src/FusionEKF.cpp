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
  // (14. Laser Measurements Part 4)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // (Jacobian Matrix Part 1)
  // Hj_ will be computed at measurement update step

  /** TODO: Set the process and measurement noises **/
  // Set the process noise/covariance matrix Q (10. Process Covariance Matrix)
  // Values will be calculated during prediction update, depending on time
  MatrixXd Q = MatrixXd(4, 4);

  // Set measurement noise/covariance matrix R (13. Laser Measurements Part 3)
  // Values will be calculated during measurement update, depending on sensor
  // Already set above R_radar/R_laser
  // Using laser as default for init.

  // Estimate matrix, x and y position and, x and y velocity
  VectorXd x = VectorXd(4);

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

      // Retrieve radar polar coordinates (15. Radar measurements)
      float rho =  measurement_pack.raw_measurements_[0]; // Range
      float phi =  measurement_pack.raw_measurements_[1]; // Bearing
      float rho_dot =  measurement_pack.raw_measurements_[2]; // Radial velocity

      // Convert polar to cartesian coordinates
      ekf_.x_ << cos(phi) * rho, // px
                 sin(phi) * rho, // py
                 cos(phi) * rho_dot, // vx
                 sin(phi) * rho_dot; // vy                 

      /** TODO: Create the covariance matrix. */
      // Certainty on position (Emit/Receive duration) and speed (Doppler, frequency change) (Derived ~ 14. Laser Measurements Part 4 - file tracking.cpp)
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      // (14. Laser Measurements Part 4)
      ekf_.x_ << measurement_pack.raw_measurements_[0], // Position px
                 measurement_pack.raw_measurements_[1], // Position py
                 0, // Lidars don't give velocity component, no vx
                 0; // Lidars don't give velocity component, no vy

      /** TODO: Create the covariance matrix. */
      // Certainty on position, high uncertainty on speed (14. Laser Measurements Part 4 - file tracking.cpp)
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;
    }

    // Initialize/Set last measurement timestamp (14. Laser Measurements Part 4)
    previous_timestamp_ = measurement_pack.timestamp_;

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
   */
  // Calculate time elapsed between the current and previous measurement (14. Laser Measurements Part 4)
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // dt - expressed in seconds
  // Initialize/Set last measurement timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  /**
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Precompute dt2noise and minimize multiplications to the cost of readability on Q
  float noise_ax = 9;
  float  noise_ay = 9;
  float dt2 = dt * dt;
  float dt2noise_ax = dt2 * noise_ax;
  float dt2noise_ay = dt2 * noise_ay;

  // Set the process noise/covariance matrix Q (10. Process Covariance Matrix)
  ekf_.Q_ << dt2 * dt2noise_ax / 4,                     0,  dt * dt2noise_ax / 2,                    0,
                                 0, dt2 * dt2noise_ay / 4,                     0, dt * dt2noise_ay / 2,
              dt * dt2noise_ax / 2,                     0,           dt2noise_ax,                    0,
                                 0,  dt * dt2noise_ay / 2,                     0,          dt2noise_ay;

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
    // (21. EKF Algorithm Generalization)
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    // (14. Laser Measurements Part 4 - file tracking.cpp)
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
