#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, int skip_initial_samples) {
	this->Kp = Kp;
  	this->Ki = Ki;
  	this->Kd = Kd;
	this->skip_initial_samples = skip_initial_samples;
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	sample_count = 0.0;
	rmse_sum = 0.0;
	twiddle_direction = 0;
	twiddle_index = 0;
	twiddle_step = 0;

	twiddle_enable = false;
	twiddle_tolerance = 1e-3;
	samples_per_twiddle = 750;
	
	twiddle_params[0] = 0.0;
	twiddle_params[1] = 0.0;
	twiddle_params[2] = 0.0;
	rmse_best = 1e10;
}


void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

	// Also update RMSE:

	// Don't count first 50 updates, because the influence
	// of the intial error (0 to the desired controlvalue) is too dominant
	if (sample_count > skip_initial_samples) {
		rmse_sum += cte*cte;
	}

	sample_count++;
}


double PID::TotalError() {
	double control_value = -Kp*p_error - Kd*d_error - Ki*i_error;
	if (control_value < -1.) {
		control_value = -1.;
	}
	else if (control_value > 1.) {
		control_value = 1.;
	}

	// check if twiddle was turned on and if we can start a new twiddle step
	if (twiddle_enable && (sample_count % samples_per_twiddle == 0)) {
		Twiddle();
	}

	return control_value;
}


double PID::GetRMSE() {
	return sqrt(rmse_sum / sample_count);
}


void PID::ResetRMSE() {
	rmse_sum = 0.0;
}


void PID::ActivateTwiddle(double Kp_p, double Ki_d, double Kd_d, double twiddle_tolerance, int samples_per_twiddle) {
	twiddle_enable = true;
	twiddle_step = 0;
	this->twiddle_tolerance = twiddle_tolerance;
	this->samples_per_twiddle = samples_per_twiddle;
	
	twiddle_params[0] = Kp_p;
	twiddle_params[1] = Ki_d;
	twiddle_params[2] = Kd_d;
	
	rmse_best = 1e10;
}


void PID::Twiddle() {
	std::cout << "Twiddle step: " << twiddle_step << std::endl;
	twiddle_step++;

	// get the current RMSE
	double current_rmse = GetRMSE();
	bool go_to_next = false;
	double increment_param = 0.0;

	// initialize before first twiddle run
	if (rmse_best >= 1e10) {
		rmse_best = current_rmse;
		twiddle_direction = 0;
		twiddle_index = 0;
	}

	// only keep twiddling if tolerance has not yet met
	if (twiddle_params[0] + twiddle_params[1] + twiddle_params[2] > twiddle_tolerance) {
		switch (twiddle_direction) {
			case 0:
				// start adjusting the current twiddle parameters
				increment_param = twiddle_params[twiddle_index];
				twiddle_direction = 1;
				break;
			case 1:
				// keep increasing twiddle parameter
				if (current_rmse < rmse_best) {
					rmse_best = current_rmse;
					twiddle_params[twiddle_index] *= 1.1;
					go_to_next = true;
				} else {
					// otherwise decrease
					increment_param = -2 * twiddle_params[twiddle_index];
					twiddle_direction = -1;
				}
				break;
			case -1:
				// keep decreasing twiddle parameter
				if (current_rmse < rmse_best) {
					rmse_best = current_rmse;
					twiddle_params[twiddle_index] *= 1.1;
				} else {
					// otherwise increase
					increment_param = twiddle_params[twiddle_index];
					twiddle_params[twiddle_index] *= 0.9;
				}
				go_to_next = true;
				break;
		}

		// update PID parameters after twiddling
		switch (twiddle_index) {
			case 0:
				Kp += increment_param;
				break;
			case 1:
				Ki += increment_param;
				break;
			case 2:
				Kd += increment_param;
				break;
		}

		// if we are switching parameters (happens after we tried increasing and, if that doesn't help, decreasing)
		if (go_to_next) {
			// reset the twiddleing direction
			twiddle_direction = 0;
			twiddle_index = (twiddle_index + 1) % 3;
		}

		ResetRMSE();
		std::cout << "PID params after twiddle: " << Kp << ", " << Ki << ", " << Kd << std::endl;
	}
	else
	{
		// deactivate twiddle once we reached a RMSE lower than the set tolerance
		twiddle_enable = false;
		std::cout << "Twiddle is finished!"<< std::endl;
	}
}
