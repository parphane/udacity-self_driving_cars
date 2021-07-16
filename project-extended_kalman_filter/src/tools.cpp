#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  // 24. Evaluating KF Performance 2
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  int n = estimations.size();
  if(n == 0) {
      cout << "Estimations vector is empty" << endl;
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if(n != ground_truth.size()) {
      cout << "Estimations / Ground truth vector size must match" << endl;
      return rmse;
  }   

  // Accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // Coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse/estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  // 20. Jacobian Matrix Part 2
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
