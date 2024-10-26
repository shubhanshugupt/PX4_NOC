/****************************************************************************
 * Date: July 5, 2024
 * Author: Shubhanshu Gupta
 *
 * Vehicle parameters file
****************************************************************************/

#include <math.h>
#include <iostream>

// vehicle parameters
double m = 2.72;
double Ixx = 0.21;
double Iyy = 0.14;
double Izz = 0.11;

// Wing paramters
double CL0 = 0;
double CLa = 5.6715;
double CD0 = 0.119;
double rho = 1.2041;
double c_bar = 0.22;
double span = 1;
double area = span * c_bar;
double AR = (span * span) / (area);
double stall_rate = 10;
double alpha_0 = 15*M_PI/180;
