/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi) {
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;
}


static std::vector<double> prev_cte;

void PID::UpdateError(double cte) {
    double tau_p = Kp;
    double tau_d = Kd;
    double tau_i = Ki;
    double diff_cte;
    double int_cte;

    //difference between the previous and current CTE for D controller
    if(prev_cte.size()!=0){diff_cte = cte - prev_cte.back();}
    else{diff_cte = cte ;}
    prev_cte.push_back(cte);


    ////sum of all past CTE for I controller
    for (auto& n : prev_cte)
        int_cte += n;

    p_error = tau_p * cte ;
    d_error = tau_d * diff_cte;
    i_error = tau_i * int_cte;
}

double PID::TotalError() {
    double steer = -p_error - d_error - i_error;
    return steer;


}
