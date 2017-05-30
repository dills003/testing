#include "kalman_filter.h"
#include <iostream>
#define PI 3.14159265359
#define TINY_NUMBER .2 

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

//int counter = 0;

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
	x_ = F_ * x_; // new state prediction - From Quiz
	MatrixXd Ft = F_.transpose(); //to get math to work
	P_ = F_ * P_ * Ft + Q_; //new state covariance matrix
	//cout << "P_: " << endl << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - (H_ * x_); //raw measurement subtract the guess, called error

	//dump error into UpdateShared, got rid of duplicate code, all fancy nonlinear does
	//is find error
	UpdateShared(y);
	//cout << "y from KF: " << endl << y << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	//we need to take the state(x) and convert it back into polar
	//opposite of what we did with initialize
	float Px = x_(0); //position of x
	float Py = x_(1); //position of y
	float Vx = x_(2); //velocity in x
	float Vy = x_(3); //velocity in y
	
	float rho = sqrt((Px * Px) + (Py * Py)); //from lecture math
	float theta = atan2(Py, Px); //help says to use atan2
	float rhoDot = (((Px * Vx) + (Py * Vy)) / (rho));

	//atan2 can return values larger than PI or smaller than -PI, fix it here
	while (theta > PI || theta < -PI)
	{
		if (theta > PI)
		{
			theta = theta - (2 * PI);
		}
		else if (theta < -PI)
		{
			theta = theta + (2 * PI);
		}
	}
	
	VectorXd hx = VectorXd(3); //creating my H * x nonlinear, squished to linear vector
	hx << rho, theta, rhoDot;
	//cout << "Hx " << hx << endl;
	//cout << "H xz " << z << endl;

	VectorXd y = z - hx; //error

	//check if rho is close to zero, if so, all sorts of hell breaks loose, no vel
	if (fabs(Px) > TINY_NUMBER && fabs(Py) > TINY_NUMBER)
	{
		UpdateShared(y);
	}
	
}

void KalmanFilter::UpdateShared(const VectorXd &y) {
	//Copied from Quiz
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	float oldy = x_(0);
	x_ = x_ + (K * y);
	float newy = x_(1);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - (K * H_)) * P_;
	//counter++;
	//cout << "Counts: " << counter << endl;
	//cout << "xval : " << oldy << "yval: " << newy << endl;
}
