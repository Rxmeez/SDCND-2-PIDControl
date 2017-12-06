#include "PID.h"

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

  Kp = Kp;
  Ki = Ki;
  Kd = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
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

}

double PID::TotalError() {
  // Returns the total error by combining all PID controller with their errors
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
