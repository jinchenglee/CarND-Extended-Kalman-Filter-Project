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
FusionEKF::FusionEKF() {
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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // px = rho * cos(phi)
      ekf_.x_(0) = measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1));
      // py = rho * sin(phi)
      ekf_.x_(1) = measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1));
      // vx = rho_dot * cos(phi)
      ekf_.x_(2) = measurement_pack.raw_measurements_(2)*cos(measurement_pack.raw_measurements_(1));
      // vy = rho_dot * sin(phi)
      ekf_.x_(3) = measurement_pack.raw_measurements_(2)*sin(measurement_pack.raw_measurements_(1));
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // px, py, vx, vy
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }

    // state transfer matrix
    Eigen::MatrixXd F_(4, 4);
    F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    //set the process covariance matrix P_
    Eigen::MatrixXd P_(4, 4);
    // Get px and py from 1st measurement, decent variance value.
    // Have no idea on vx and vy at all, start with very large variance.
    P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // state process noise matrix Q_
    Eigen::MatrixXd Q_(4, 4);
    Q_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

    // Laser measurement H_laser_
    H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

    // Initialize EKF filter matrices - note H and R are different for laser/radar
    ekf_.Init(P_, F_, H_laser_, R_laser_, Q_);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Delta time elapsed
  long dt = 0;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // time stamp
    dt = measurement_pack.raw_measurements_(3) - previous_timestamp_;
    previous_timestamp_ = measurement_pack.raw_measurements_(3);
    // Radar R
    ekf_.R_ = R_radar_;
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // time stamp
    dt = measurement_pack.raw_measurements_(2) - previous_timestamp_;
    previous_timestamp_ = measurement_pack.raw_measurements_(2);
    // Laser R
    ekf_.R_ = R_laser_;
  }

  // F update w/ dt
  /**
   F = 1, 0, dt, 0,
       0, 1, 0, dt,
       0, 0, 1, 0,
       0, 0, 0, 1;
   */
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Q update w/ dt
  /**
   * Q = dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
        0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
        dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
        0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
   */
  long dt_2 = dt*dt;
  long dt_3 = dt*dt_2;
  long dt_4 = dt*dt_3;
  ekf_.Q_(0,0) = noise_ax*dt_4/4;
  ekf_.Q_(0,2) = noise_ax*dt_3/2;
  ekf_.Q_(1,1) = noise_ay*dt_4/4;
  ekf_.Q_(1,3) = noise_ay*dt_3/2;
  ekf_.Q_(2,0) = noise_ax*dt_3/2;
  ekf_.Q_(2,2) = noise_ax*dt_2;
  ekf_.Q_(3,1) = noise_ay*dt_3/2;
  ekf_.Q_(3,3) = noise_ay*dt_2;

  // Prediction
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
