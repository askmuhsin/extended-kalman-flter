#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using std::cout;

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159265

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;  //state vector
  P_ = P_in;  //state covariance matrix
  F_ = F_in;  //state transition matrix
  H_ = H_in;  //process covariance matrix
  R_ = R_in;  //measurement matrix
  Q_ = Q_in;  //measurement covariance matrix
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y_ = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho_dot;
  float theta;

  float rho = sqrt(px*px + py*py);

  if(rho < .00001) {
    px += .001;
    py += .001;
    rho = sqrt(px * px + py * py);
  }

  theta = atan2(py, px);
  rho_dot = (px*vx + py*vy)/rho;

  VectorXd z_pre = VectorXd(3);
  z_pre << rho, theta, rho_dot;

  VectorXd y_ = z - z_pre;
  y_[1] = atan2(sin(y_[1]), cos(y_[1]));  //normalize

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
