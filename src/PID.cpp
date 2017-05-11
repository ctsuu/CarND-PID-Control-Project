#include "PID.h"
#include <iostream>
#include <math.h>


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

  // set all error to 0.0
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

}


void PID::UpdateError(double cte, double dt, double diff,double velo) {

  // Integration term: keep adding all CTE's
  i_error += cte;

  
  if (velo !=0){
  // Proportional term: CrossTrack Error 
  p_error = cte/velo;

  // Differential term: rate of change by delta time 
  d_error = diff/velo*velo;
  }
  // Debug
  cout << "Kp = "<<"\t"<< Kp << "\t"<<"P_error= " << Kp*p_error << endl;
  cout << "Ki = "<<"\t"<< Ki << "\t"<<"I_error= " << Ki*i_error << endl;
  cout << "Kd = "<<"\t"<< Kd << "\t"<<"D_error= " << Kd*d_error << endl;
    

}


double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;

}



