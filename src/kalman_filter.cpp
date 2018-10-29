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
  cout << "====>" << __FUNCTION__ << endl;
  x_ = x_in;
  cout << "state x: " << x_ << endl;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  cout << "<====" << __FUNCTION__ << endl;
}

void KalmanFilter::Predict()
{
  cout << "====>" << __FUNCTION__ << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  cout << "<====" << __FUNCTION__ << endl;
}

void KalmanFilter::Update(const VectorXd &z)
{
  cout << "====>" << __FUNCTION__ << endl;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout << "<====" << __FUNCTION__ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  cout << "====>" << __FUNCTION__ << endl;
  cout << "z = " << z << endl;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  VectorXd z_pred = tools.CartesianToPolar(x_);
  VectorXd y = z - z_pred;
  cout << "y = " << y << endl;

  while (y[1] < -M_PI)
  {
    y[1] += 2 * M_PI;
  }
  while (y[1] > M_PI)
  {
    y[1] -= 2 * M_PI;
  }
  MatrixXd Hjt = Hj.transpose();
  cout << "Hjt = " << Hjt << endl;
  cout << "Hj: " << Hj.rows() << "," << Hj.cols() << " P:" << P_.rows() << "," << P_.cols() << " Hjt:" << Hjt.rows() << "," << Hjt.cols() << " R:" << R_.rows() << "," << R_.cols() << endl;

  MatrixXd S = Hj * P_ * Hjt + R_;

  cout << "S = " << Hjt << endl;

  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;

  cout << "<====" << __FUNCTION__ << endl;
}
