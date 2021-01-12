/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/


#include "pid_controller.h"
#include <iostream>
#include <fstream>
using namespace std;


int main() {
    // create instance pid
    PID pid = PID();

    // create file to save values
    ofstream myfile;
    myfile.open ("pid_data.txt");

    // initialize values for pid
    double Kpi = 0.7;
    double Kii = 0.1;
    double Kdi = 0.1;
    pid.Init(Kpi, Kii, Kdi);

    double val = 10;
    for (int i = 0; i < 100; i++) {
        pid.UpdateError(val);
        double inc = pid.TotalError();
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val += inc;
        myfile  << i ;
        myfile  << " " << val;
        myfile  << " " << inc << endl;
    }
    myfile.close();
    return 0;
}