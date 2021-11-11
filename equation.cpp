//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <string>
#include <cmath>
#include "equation.h"
#include "mainwindow.h"

#define PI 3.141592
using namespace std;
typedef std::vector< double > state_type;              //State_type definition


void Vehicle::operator() (const state_type& x, state_type& dxdt, const double t)      //Void equation system declaration
{
    dxdt[0] = (Fyf + Fyr) / MASS - x[1] * (VX/3.6);    //Lateral velocity derivative   (m/s)
    dxdt[1] = (Fyf * LF - Fyr * LR) / IZZ;             //Yaw acceleration              (rad/s²)
    Fyf = af * CF;                                     //Front tire lateral force def  (N)
    Fyr = ar * CR;                                     //Rear tire lateral force def   (N)
    ar = -(x[0] - x[1] * LR) / (VX/3.6);               //Rear tire slip angle def      (rad)

if (t > 3)                                             //Front tire slip angle def     (rad)
{
    if (ST_T == "step")
    {delt = (AMPL*0.0174);                             //Step steer definition
    af = -(x[0] + x[1] * LF) / (VX/3.6) + delt;}

    if (ST_T == "ramp")
    {delt = (AMPL*0.0174)*((t-3)/7) ;                  //Ramp steer definition
    af = -(x[0] + x[1] * LF) / (VX/3.6) + delt;}

    if (ST_T == "sine")
    {delt = (AMPL*0.0174)*sin(FREQ*2*PI*(t-3));        //Sine wave steer definition
    af = -(x[0] + x[1] * LF) / (VX/3.6) + delt;}
    else
    {delt =  0 ;}                                      //Steering null by default
}
   else
    {af = -(x[0] + x[1] * LF) / (VX/3.6);}             //Steering null by default

}

