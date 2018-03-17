#include "kalman_filter.h"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
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
  cout << "x_: " << x_ << endl;
  cout << "H_: " << H_ << endl;

  VectorXd z_pred = H_ * x_;
  cout << "z_pred: " << z_pred << endl;
  cout << "z: " << z << endl;

	
  VectorXd y = z - z_pred;
  cout << "P_: " << P_ << endl;
  cout << "y: " << y << endl;
  cout << " R_: " <<  R_ << endl;
  MatrixXd Ht = H_.transpose();
  cout << " Ht: " <<  Ht << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << " S: " <<  S << endl;
  MatrixXd Si = S.inverse();
  cout << " Si: " <<  Si << endl;
  MatrixXd PHt = P_ * Ht;
  cout << " PHt: " <<  PHt << endl;
  MatrixXd K = PHt * Si;
  cout << " K: " <<  K << endl;
  
  //new state eq
  x_ = x_ + K*y;
  cout << " x_: " <<  x_ << endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout << " P_: " <<  P_ << endl;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  
  
}
