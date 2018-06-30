#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   TODO:
  * predict the state
  */
  x_ = F_ * x_;
  P_ = F_* P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   TODO:
  * update the state by using Kalman Filter equations
  */
  VectorXd y(2);
  MatrixXd S(2, 2);
  MatrixXd K(4, 2);
  
  y = z - H_ * x_;
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();

  MatrixXd I = MatrixXd::Identity(4, 4);
  x_ = x_ + (K*y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd y(3);
  y = z - CartesianToPolar(x_);
  y[1] = Tools::ThetaValueCorrection(y[1]);

  // Tools tool = Tools();
  MatrixXd Hj(3, 4);
  Hj = Tools::CalculateJacobian(x_);

  MatrixXd S(3, 3);
  MatrixXd K(4, 3);

  S = Hj * P_ * Hj.transpose() + R_;
  K = P_ * Hj.transpose() * S.inverse();

  MatrixXd I = MatrixXd::Identity(4, 4);
  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
}

VectorXd KalmanFilter::CartesianToPolar(const VectorXd &cartesian) {
  float px = cartesian[0];
  float py = cartesian[1];
  float vx = cartesian[2];
  float vy = cartesian[3];

  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float theta = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;
  
  VectorXd polar(3);
  polar << rho, theta, rho_dot;
  return polar;
}