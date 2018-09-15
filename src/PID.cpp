#include "PID.h"

#include <algorithm>
// using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // Previous cte.
  prev_cte = 0.0;

}

void PID::UpdateError(double cte) {
  // Proportional error.
  p_error = cte;

  // Integral error.
  i_error += cte;

  // Diferential error.
  d_error = cte - prev_cte;
  prev_cte = cte;

  
}

double PID::TotalError() {
  return p_error * Kp + i_error * Ki + d_error * Kd;
}



void PID::Twiddle(double tol) {
	//Using Twiddle Algorithm to adjust the gain values (Kp,Kd,Ki)
	double p[] = { Kp, Kd, Ki };
	double dp[] = { 0.05, 0.05, 0.05 };
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
				dp[i] *= 1.05;

			}
			else {

				p[i] -= 2 * dp[i];
				Kp = p[0];
				Kd = p[1];
				Ki = p[2];
				err = TotalError();

				if (err < best_err) {

					best_err = err;
					dp[i] *= 1.05;
				}
				
				else {

					p[i] += dp[i];
					dp[i] *= 0.95;
				}
				
			}
			
				

		}
		Kp = p[0];
		Kd = p[1];
		Ki = p[2];
		// For debugging purpose
		//cout << "kp = " << Kp << "  kd = " << Kd << "  ki = " << Ki << endl;
		
	}




}

