#include <iostream>
#include "tools.h"
#include <assert.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
    *
    * RMSE = SQRT(Sigma((estimation - ground_truth)^2))/num_samples
    *
  */
  // Data sizes
  int est_size = estimations.size();
  int gr_size = ground_truth.size();
  assert (est_size == gr_size);
  std::cout << "Total number of estimations: " << est_size << "\n";

  // RMSE
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  for(int i=0; i< est_size; i++) {
    VectorXd tmp;
    // Element-wise operations
    tmp = estimations[i] - ground_truth[i];
    tmp = (tmp.array()*tmp.array());
    // Accumulation into rmse
    rmse += tmp;
  }
  rmse = rmse.array()/est_size;
  return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
