#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  int skip_initial_samples;
  int sample_count;
  bool twiddle_enable;
  double rmse_sum;
  double rmse_best;
  double twiddle_tolerance;
  double twiddle_params[3];
  int samples_per_twiddle;
  int twiddle_direction, twiddle_index;
  int twiddle_step;
  
  /*
  * Constructor
  */
  PID();

  PID(double Kp, double Ki, double Kd) {
  	Init(Kp, Ki, Kd);
  };

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, int skip_initial_samples = 50);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the RMSE of all total PID errors so far.
  */
  double GetRMSE();

  /*
  * Reset internal RMSE value
  */
  void ResetRMSE();

  /*
  * Parameterize twiddle, which optimizes the P, I and D gains by reducing the RMSE.
  */
  void ActivateTwiddle(double Kp_d, double Ki_d, double Kd_d, double twiddle_tolerance, int samples_per_twiddle);

  /*
  * Twiddle function which keeps changing the P, I and D gains in order to recude the RMSE.
  */
  void Twiddle();
};

#endif /* PID_H */
