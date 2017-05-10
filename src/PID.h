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

  double velo;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error and delta_t.
  */
  void UpdateError(double cte, double dt, double diff, double velo);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
