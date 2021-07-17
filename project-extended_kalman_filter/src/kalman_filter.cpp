#include "kalman_filter.h"
#include "tools.h"
#include "math.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
  verbose_ = false;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {  
  tools.DebugMessage(verbose_, "KalmanFilter: Initialize inputs");
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  I_ = MatrixXd::Identity(P_.rows(), P_.cols());
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  tools.DebugMessage(verbose_, "KalmanFilter: Predict");
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  tools.DebugMessage(verbose_, "KalmanFilter: Update");
  // Initialize y
  VectorXd y = z - (H_ * x_);

  MeasurementUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  tools.DebugMessage(verbose_, "KalmanFilter: UpdateEKF");
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  
  // Avoiding x=0 for atan
  // Creates anomalies when active
  //if (px < 0.00007) {
  //  px = 0.00007;
  //}
  
  // Compute rho phi rho_dot (18. Multivariate Taylor Series Expansion)
  float rho = sqrt(px*px + py*py);  
  // Avoiding division by zero (using snippet from 20. Jacobian Matrix Part 2)
  // 0.0001^2 = 0.00000001 then 0.00000001/2 = 0.000000005 then sqrt(0.000000005) = 0.00007
  if (rho < 0.0001) {
    px = 0.00007;
    py = 0.00007;
    rho = sqrt(px*px + py*py);
  } 
  float phi = atan2(py, px);
  float rho_dot = (px*vx + py*vy) / rho;
  
  // Initialize h(x_) (18. Multivariate Taylor Series Expansion)
  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  // Initialize y
  VectorXd y = z-h;
  
  // Normalize the angle between -Pi and Pi
  // Inspired by https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
  // Get remainder (-2Pi<=h<=2Pi) from division by 2*Pi rad (360deg)
  y[1] = remainder(y[1],2.0*M_PI);
  // If angle is above Pi rad (180deg)
  if (y[1] > M_PI)
      // Remove 2Pi rad (360deg) to obtain -Pi<=h<=Pi domain angle
      y[1] -= (2.0 * M_PI);
  // If angle is below -Pi rad (-180deg)
  if (y[1] < -M_PI)
      // Add 2Pi rad (360deg) to obtain -Pi<=h<=Pi domain angle
      y[1] += (2.0 * M_PI);

  MeasurementUpdate(y);
}

void KalmanFilter::MeasurementUpdate(const VectorXd &y) {
  tools.DebugMessage(verbose_, "KalmanFilter: MeasurementUpdate");
  // Compute Ht only once
  MatrixXd Ht = H_.transpose();
  // Perform final operations of measurement update (21. EKF Algorithm Generalization)    
  MatrixXd S = H_ * P_ * Ht + R_;
  // Kalman gain
  MatrixXd K =  P_ * Ht * S.inverse();

  // Estimate update
  x_ = x_ + (K * y);
  // Uncertainty update
  P_ = (I_ - K * H_) * P_;

}
