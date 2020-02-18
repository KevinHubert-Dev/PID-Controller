#include "PID.h"
#include <math.h>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  // Set all varaibles to default values
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID coefficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  std::cout << "Inited" << std::endl;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  // D of PID is the error-derivation (the delta of the previous to the current error taking into account the elapsed time)
  d_error = cte - p_error;
  
  // Proportional of PID is the cross-track error itself
  p_error = cte;
  
  // Intergral of PID is the sum of all cte-values, this will help with systematic-bias-problem 
  // which e.g. occures when the wheels are not perfectly aligned
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  //     |------P------|   |------I-----|   |------D-----|    
  return (p_error * -Kp) - (i_error * Ki) - (d_error * Kd);
}