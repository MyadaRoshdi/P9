#include "PID.h"

using namespace std;

/*
* Complete implementation of the PID class
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this.Kp = kp;
	this.Ki = ki;
	this.Kd = kd;
	this.p_error = 0.0;
	this.d_error = 0.0;
	this.i_error = 0.0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error; //update the Derivative error and the difference between the current cte and previous cte
	p_error = cte; // update the Proportional error as the current cte
	i_error += cte; // update the Integral error as the accumaled cte among all steps call
	// TODO: USE TWIDDLE ALGORITHM TO ADJUST THE GAIN VALUES (kp,kd,ki)
}

double PID::TotalError() {
	// Calculate the total error used for computing the steering angle (probortional part + differential part + Integral part)

	return -Kp * p_error - Kd * d_error - Ki * i_error
}

