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
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  cout << "rho = " << rho << endl;
  cout << "phi = " << phi << endl;
  cout << "rho_dot = " << rho_dot << endl;
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  if( y(1) > 3.14159265 )
    y(1) -= 2.f*3.14159265;
  if( y(1) < -PI )
    y(1) += 2.f*3.14159265;
  cout << "y = " << y << endl;
  MatrixXd Ht = H_.transpose();
  cout << "Ht = " << Ht << endl;
  cout << "P_ = " << P_ << endl;
  cout << "R_ = " << R_ << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << "S = " << S << endl;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
