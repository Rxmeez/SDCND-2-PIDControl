#include "PID.h"
#include <cmath>

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
  dp[0] = 1.0;
  dp[1] = 1.0;
  dp[2] = 1.0;

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

  iter += 1;
}

double PID::TotalError() {
  // Returns the total error/steer_value by combining all PID controller with their errors
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::Twiddle(double tol = 0.0001){

  double best_err = TotalError();
  double best_cte = p_error;
  double err;

  while(abs(dp[0]) + abs(dp[1]) + abs(dp[2]) > tol){
    for (int i=0; i<p.size(); i++){

      p[i] += dp[i];
      err = TotalError();

      if (err < best_err){
        best_err = err;
        dp[i] *= 1.1;
      }
      else{
        p[i] -= 2 * dp[i];
        err = TotalError();

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

    iter += 1;

  }
}
