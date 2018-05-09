#include "PID.h"

using namespace std;

/*
* Complete implementation of the PID class
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp = Kp;
	Ki = Ki;
	Kd = Kd;
	p_error = 0.0;
	d_error = 0.0;
	i_error = 0.0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error; //update the Derivative error and the difference between the current cte and previous cte
	p_error = cte; // update the Proportional error as the current cte
	i_error += cte; // update the Integral error as the accumaled cte among all steps call
	// TODO: USE TWIDDLE ALGORITHM TO ADJUST THE GAIN VALUES (kp,kd,ki)
	Twiddle(0.2);
}

double PID::TotalError() {
	// Calculate the total error used for computing the steering angle (probortional part + differential part + Integral part)

	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

void PID::Twiddle(double tol) {
	//Using Twiddle Algorithm to adjust the gain values (Kp,Kd,Ki)
	double p[] = { Kp, Kd, Ki };
	double dp[] = { 1.0, 1.0, 1.0 };
	double best_err = TotalError();
	double err = 0.0;
	
	while ((dp[0] + dp[1] + dp[2]) > tol) {
		
		for (int i = 0; i < 3; i++) {

			p[i] += dp[i];
			Kp = p[0];
			Kd = p[1];
			Ki = p[2];
			err = TotalError();
			if (err < best_err) {
				best_err = err;
				dp[i] *= 1.1;

			}
			else {

				p[i] -= 2 * dp[i];
				Kp = p[0];
				Kd = p[1];
				Ki = p[2];
				err = TotalError();

				if (err < best_err) {

					best_err = err;
					dp[i] *= 1.1;
				}
				
				else {

					p[i] += dp[i];
					dp[i] *= 0.9;
				}
				
			}
			
				

		}
		Kp = p[0];
		Kd = p[1];
		Ki = p[2];
		
	}




}

