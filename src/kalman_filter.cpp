#include <iostream>
#include "kalman_filter.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  FUNC_IN;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  auto x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
  FUNC_OUT;
}

void KalmanFilter::Predict()
{
  FUNC_IN;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  FUNC_OUT;
}

void KalmanFilter::Update(const VectorXd &z)
{
  FUNC_IN;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
  FUNC_OUT;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  FUNC_IN;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  VectorXd z_pred = tools.CartesianToPolar(x_);
  VectorXd y = z - z_pred;
  // adjust until the angle is between [-pi,pi]
  while (y[1] < -M_PI)
  {
    y[1] += 2 * M_PI;
  }
  while (y[1] > M_PI)
  {
    y[1] -= 2 * M_PI;
  }
  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * Hj) * P_;
  FUNC_OUT;
}
