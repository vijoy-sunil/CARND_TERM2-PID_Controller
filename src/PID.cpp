#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;
double tol = 0.01;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	p_error = 0.1;
	d_error = 0.1;
	i_error = 0.0001;

	// Previous cte.
	prev_cte = 0.0;
}

void PID::UpdateError(double cte) {

	double p[] = {Kp, Kd, Ki};
	double dp[] = {p_error, d_error, i_error};

	if(TotalError() > tol)
	{
		switch(state)
		{
			case 0:
				//up
				p[parameter_index] += dp[parameter_index];
				state = 1;
				break;

			case 1:
				//up good
				if(fabs(cte) < fabs(besterror))
				{
					besterror = cte;
					dp[parameter_index] *= 1.1;
					state = 3;
				}
				//down
				else 
				{
          			p[parameter_index] -= 2 * dp[parameter_index];
          			state = 2;
				}
				break;

			case 2:
				//down good
				if(fabs(cte) < fabs(besterror))
				{
					besterror = cte;
					dp[parameter_index] *= 1.1;
				}

				//retain original
				else
				{
					p[parameter_index] += dp[parameter_index];
					dp[parameter_index] *= 0.9;
				}
				state = 3;
				break;

			case 3:
				//next index
				parameter_index = (parameter_index + 1) % 2;
				state = 0;
				break;
		}
	}
	//cout << "Params " << p[0] << " "<< p[1] << " " << p[2]<< endl;
	p_error = dp[0];
	d_error = dp[1];
	i_error = dp[2];

	Kp = p[0];
	Kd = p[1];
	Ki = p[2];
}

double PID::TotalError() {
	double err;
	err = fabs(p_error) + fabs(i_error) + fabs(d_error);
	return err;
}

double PID::PidControl(double cte) {
	// Proportional error.
	p_error = cte;

	// Integral error.
	i_error += cte;

	// Diferential error.
	d_error = cte - prev_cte;
	prev_cte = cte;
	return (-(Kp * p_error) -(Kd * d_error) -(Ki * i_error))* M_PI / 180;;
}



