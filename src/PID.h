#ifndef PID_H
#define PID_H

#include <math.h>
#include <iostream>
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /** 
   * twiddle the parameters
   */
//  std::vector<double> Twiddle(std::vector<double> &dp, double& best_cte, double cte);
  void Twiddle(double& best_cte, double cte);
  double TolCheck();
  void PrintParams();
  void RetrainPID();

 private:
  /** 
  * twiddle step counter and parameter ctr
  */
  int twiddle_step;
  int dp_i;
  std::vector<double> dp;


  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  std::vector<double> ps;
};

#endif  // PID_H