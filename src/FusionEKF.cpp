#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  FUNC_IN;

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

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

    //state covariance matrix
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  Q_ = MatrixXd(4, 4);


  FUNC_OUT;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::init(const MeasurementPackage &measurement_pack)
{
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      auto ro = measurement_pack.raw_measurements_[0];
      auto phi = measurement_pack.raw_measurements_[1];
      auto ro_dot = measurement_pack.raw_measurements_[2];
      auto x = ro * cos(phi);
      auto y = ro * sin(phi);
      auto vx = ro_dot * cos(phi);
      auto vy = ro_dot * sin(phi);

      ekf_.x_ << x, y, vx, vy;
      ekf_.Init(ekf_.x_, P_, F_, Hj_, R_radar_, Q_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      /**
      Initialize state.
      */
      auto x =  measurement_pack.raw_measurements_[0];            
      auto y =  measurement_pack.raw_measurements_[1];

      ekf_.x_ << x, y, 0, 0;
      ekf_.Init(ekf_.x_, P_, F_, H_laser_, R_laser_, Q_);
    }
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
}

void FusionEKF::predict(float dt)
{
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;


  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  auto noise_ax = 9;
  auto noise_ay = 9;
  //set the process covariance matrix Q
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
      0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
      dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
      0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;


  ekf_.Predict();
}

void FusionEKF::update(const MeasurementPackage &measurement_pack)
{
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
   FUNC_IN;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {
    init(measurement_pack);
    FUNC_OUT;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  update(measurement_pack);

  FUNC_OUT;
}
