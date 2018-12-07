#include "kalman_filter.h"
#include <iostream>
#include <math.h>

#define PI acos(-1)
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
  MatrixXd F_transpose = F_.transpose();
  P_ = F_ * P_ * F_transpose + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd H_transpose = H_.transpose();
  MatrixXd S = H_ * P_ * H_transpose+ R_;
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = P_ * H_transpose * S_inverse;
    
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h = VectorXd(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float theta = atan2(py, px);
  h << sqrt(px * px + py * py), theta, (px * vx + py * vy) / (sqrt(px * px + py * py));
  VectorXd y = z - h;
  do
  {
    if(y(1) < -PI){
      y(1) = y(1) + (2 * PI);
    }
    if(y(1) > PI){
      y(1) = y(1) - (2 * PI);
    }
  }while( y(1) < -PI || y(1) > PI );
  MatrixXd H_transpose = H_.transpose();
  MatrixXd S = H_ * P_ * H_transpose+ R_;
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = P_ * H_transpose * S_inverse;
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
