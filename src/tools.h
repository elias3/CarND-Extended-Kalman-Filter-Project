#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// #define _DEBUG

#ifdef _DEBUG
#define FUNC_IN  cout << "====>" << __FUNCTION__ << endl;
#define FUNC_OUT  cout << "<====" << __FUNCTION__ << endl;
#else
#define FUNC_IN
#define FUNC_OUT
#endif

class Tools
{
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd &x_state);

  /**
  * A helper method to convert cartesian coordinates to polar coordinates.
  */
  VectorXd CartesianToPolar(const VectorXd &x_state);
};

#endif /* TOOLS_H_ */
