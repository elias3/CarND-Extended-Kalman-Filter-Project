#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  cout << "====>" << __FUNCTION__ << endl;
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for (auto i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  cout << "<====" << __FUNCTION__ << endl;
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
  cout << "====>" << __FUNCTION__ << endl;

  MatrixXd Hj(3, 4);
  //recover state parameters
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  auto c1 = px * px + py * py;
  auto c2 = sqrt(c1);
  auto c3 = (c1 * c2);

  //check division by zero
  if (fabs(c1) < 0.0001)
  {
    cout << __FUNCTION__ << "Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  cout << "<====" << __FUNCTION__ << endl;
  return Hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd &x_state)
{
  cout << "====>" << __FUNCTION__ << endl;
  VectorXd h(3);
  auto px = x_state[0];
  auto py = x_state[1];
  auto vx = x_state[2];
  auto vy = x_state[3];

  h[0] = sqrt(pow(px, 2) + pow(py, 2));
  h[1] = atan2(py, px);
  h[2] = (px * vx + py * vy) / h[0];
  
  cout << "h = " << h << endl;

  cout << "<====" << __FUNCTION__ << endl;
  return h;
}