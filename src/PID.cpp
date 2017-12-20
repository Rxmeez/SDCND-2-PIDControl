#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /*
  * Initialize all the PID controller Coefficients with the input values as well as start all the error at 0.0
  */

  // Coefficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // Errors for each Coefficients Initialized
  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  // Twiddle values
  p.resize(3);
  dp.resize(3);
  iter = 0;

  //Initialize Twiddle dp values
  dp[0] = Kp * 0.1;
  dp[1] = Ki * 0.1;
  dp[2] = Kd * 0.1;

  // Twiddle PID Coefficient that will be fine tuned
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
}

void PID::UpdateError(double cte) {
  /*
  * Update error values for each controls Coefficient (Kp, Ki, Kd)
  */

  // d_error the difference from cte and previous cte labeled as p_error
  d_error = cte - p_error;

  // p_error set it to new cte
  p_error = cte;

  // i_error continue to sum up all ctes
  i_error += cte;

  // iterate to the next stage
  iter += 1;
}

double PID::TotalError(double p_coeff, double i_coeff, double d_coeff) {
  // Returns the total error/steer_value by combining all PID controller with their errors
  return -p_coeff * p_error - i_coeff * i_error - d_coeff * d_error;
}

void PID::Twiddle(double tol){

  double best_err = abs(TotalError(Kp, Ki, Kd));
  double best_cte = p_error;
  double err;

  // while still below tol
  while(abs(dp[0]) + abs(dp[1]) + abs(dp[2]) > tol){
    for (int i=0; i<p.size(); i++){

      p[i] += dp[i];
      err = abs(TotalError(p[0], p[1], p[2]));

      if (err < best_err){
        best_err = err;
        dp[i] *= 1.1;
      }
      else{
        p[i] -= 2 * dp[i];
        err = abs(TotalError(p[0], p[1], p[2]));

        if (err < best_err){
          best_err = err;
          dp[i] *= 1.1;
        }
        else{
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
  for (int j=0; j < p.size(); j++){
    if (p[j] < 0.0){
      p[j] = abs(p[j]);
    }
  }

  iter += 1;

  // Change coefficients of p values back to PID, after loop.
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  // Reset dp values for another iteration
  dp[0] = Kp * 0.1;
  dp[1] = Ki * 0.1;
  dp[2] = Kd * 0.1;

}
