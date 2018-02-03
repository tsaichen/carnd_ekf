#include <iostream>
#include "tools.h"
#include "math.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float px_py_rms = pow(px*px + py*py, 0.5);
  
  if (px == 0  && py ==0){
	  return Hj;
  }
  Hj << px/px_py_rms, py/px_py_rms,0,0,
	    -py/(px_py_rms*px_py_rms), px/(px_py_rms*px_py_rms),0,0,
	    py*(vx*py-vy*px)/(px_py_rms*px_py_rms*px_py_rms),px*(vy*px-vx*py)/(px_py_rms*px_py_rms*px_py_rms),px/px_py_rms, py/px_py_rms;
  return Hj;
}
