#include "PID.h"
#include <numeric>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */

   Kp = Kp_;
   Kd = Kd_;
   Ki = Ki_;
   i_error = 0.0;
   ps = {Kp, Ki, Kd};
   twiddle_step = 1;
   dp_i = 0;
   dp = {Kp/10.0, Ki/10.0, Kd/10.0};

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */

  double alpha = - ps[0] * p_error - ps[1] * i_error -  ps[2] * d_error;
  return alpha;  
}

double PID::TolCheck() {

  double dp_sum = 0.0;
  dp_sum = std::accumulate(dp.begin(), dp.end(), decltype(dp)::value_type(0));
  return dp_sum;
}

void PID::PrintParams() {
  std::cout << "twiddle step: " << twiddle_step << " -> p:";
  for (auto p : ps) { 
    std::cout << p << " ";
  }
  std::cout << " " << std::endl;
  // std::cout << "              " << Kp << " " << Kd << " " << Ki <<std::endl;
  std::cout << "              " << twiddle_step << " -> dp:";
  for (auto p : dp) { 
    std::cout << p << " ";
  }
  std::cout << " " << std::endl;
}

void PID::RetrainPID() {
  for (int i = 0; i < dp.size(); i++) {
    dp[i] = dp[i] * 1000; 
  }
  
}

//std::vector<double> PID::Twiddle(std::vector<double> &dp, double& best_cte, double cte) {
void PID::Twiddle(double& best_cte, double cte) {
  PrintParams();
  std::cout << "param " << dp_i <<std::endl;  
              
  // twiddle step 1: adjust parameters by dp
  // run some more

  // twiddle step 2: check error aginst best error
  // increase step size (dp) or adjust (p) in opposite direction

  // if p adjusted, twiddle step3: check error and increase step size 
  // or decrease step size 

  bool next_param = false;
  if (twiddle_step == 1) {
    ps[dp_i] += dp[dp_i];
    twiddle_step = 2;
    // std::cout << "twiddle step: " << twiddle_step << std::endl;
  }
  else if (twiddle_step == 2) {
    if (fabs(cte) < best_cte) {
      best_cte = fabs(cte);
      // std::cout << best_cte << std::endl;
      dp[dp_i] *= 1.1;
      twiddle_step = 1;
      next_param = true;
    }
    else {
      ps[dp_i] -= 2*dp[dp_i];
      twiddle_step = 3;
    }
  }
  else if (twiddle_step == 3) {
    if (fabs(cte) < best_cte) {
      best_cte = fabs(cte);
      dp[dp_i] *= 1.1;
      twiddle_step = 1;
      next_param = true;
    }
    else {
      ps[dp_i] += dp[dp_i];
      dp[dp_i] *= 0.9;
      twiddle_step = 1;
      next_param = true;
    }
  }

  if (next_param) {
    dp_i += 1;
    if (dp_i >= 3) {dp_i = 0;}
  }


  // for (auto p : ps) { 
  //   std::cout << p << " ";
  // }
  // std::cout << " " << std::endl;

}