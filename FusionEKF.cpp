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

	//measurement covariance matrix - laser, usually given by manufacturer
	R_laser_ << 0.0225, 0,
		0, 0.0225;

	//measurement covariance matrix - radar, usually given by manufacturer
	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	/**
	TODO:
	  * Finish initializing the FusionEKF.
	  * Set the process and measurement noises
	  R is measurement process is noise
	*/
	//Set the H_laser values, Hj is the Jacobian we must calculate each time
	//H_laser is just so the math works, keeping  only position from state
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	//Create the State Covariance Matrix - Copied from quiz
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
	
	//Create Transition matrix
	ekf_.H_ = MatrixXd(4, 4);
	ekf_.H_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	//Create Process Covariance Matrix
	ekf_.Q_ = MatrixXd(4, 4);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	/*****************************************************************************************
	 *  Initialization - We need a state(x), state covariance matrix(P), and a timestamp
	 *****************************************************************************************/
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
		ekf_.x_ << 1, 1, 1, 1; //state postion and velocities, kept to make sure first read does its thing
		//cout << "x_ = " << ekf_.x_ << endl;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			//we can only get position, because we don't have enough info yet, leave velocities at 0
			float rho = measurement_pack.raw_measurements_[0]; //rho, the distance from car
			float theta = measurement_pack.raw_measurements_[1]; //phi, the angle from x
			float Px = rho * cos(theta); //polar to cartesian to find x
			float Py = rho * sin(theta); //polar to cartesian to find y

			ekf_.x_ << Px, Py, 2, 2; //initialized state
			//cout << "x_R = " << ekf_.x_ << endl;

		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			//can only gather position, leave velocity at zeros
			ekf_.x_ << measurement_pack.raw_measurements_, 2, 2; //initialized state
			//cout << "x_L = " << ekf_.x_ << endl;
		}

		//grab the first timestamp
		previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************************************
	 *  Prediction - Calculate delta t, Set up State Transition (F), Process Covariance (Q), before predict
	 ****************************************************************************************************/

	 /**
	  TODO:
		* Update the state transition matrix F according to the new elapsed time.
		 - Time is measured in seconds.
		* Update the process noise covariance matrix.
		* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	  */
	  //Set noise for Q matrix
	float noise_ax = 9;
	float noise_ay = 9;

	//delta t - copied from quiz
	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt; //used for less typing
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//Modify the F matrix so that the time is integrated - From Quiz
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	//set the process covariance matrix Q - From Quiz
	ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

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
		// Radar updates, uses Jacobian to linearzie
		// Find Hj and put into ekf H
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_); //H matrix
		// Put correct R in
		ekf_.R_ = R_radar_;
		// Call correct Update with sensor data
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

	}
	else {
		// Laser updates
		// Find Hj and put into ekf H
		ekf_.H_ = H_laser_; //H matrix
		// Put correct R in
		ekf_.R_ = R_laser_;
		// Call correct Update with sensor data
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
