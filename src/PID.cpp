#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
  	this->Ki = Ki;
  	this->Kd = Kd;
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	double control_value = -Kp*p_error - Kd*d_error - Ki*i_error;
	std::cout << "control_value: " << control_value << std::endl;
	if (control_value < -1.) {
		control_value = -1.;
	}
	else if (control_value > 1.) {
		control_value = 1.;
	}
	return control_value;
}

