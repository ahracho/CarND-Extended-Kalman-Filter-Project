#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground truth data" << endl;
    return rmse;
  }

  for (unsigned int i = 0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];

  float dist = pow(px, 2) + pow(py, 2);
  if (dist == 0) {
    dist = 0.0000001;
  }

  MatrixXd Hj = MatrixXd(3, 4);
  Hj << px / sqrt(dist), py / sqrt(dist), 0, 0, 
          -py / dist, px / dist, 0, 0,
          py * (vx * py - vy * px) / pow(dist, 3 / 2),
          px * (vy * px - vx * py) / pow(dist, 3 / 2), 
          px / sqrt(dist),
          py / sqrt(dist);
  return Hj;
}

float Tools::ThetaValueCorrection(float theta) {
  while (1) {
    if (theta <= M_PI && theta >= -M_PI) {
      break;
    }

    if (theta > M_PI) {
      theta -= (2 * M_PI);
    }

    if (theta < -M_PI) {
      theta += (2 * M_PI);
    }
  }
  return theta;
}