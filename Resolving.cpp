//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#include "integ_meth.h"
#include "mystruct.h"
#include "QProgressBar"
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;


std::tuple<vector<double>, vector<double>, vector<double>, size_t> resolution() //Send three vectors (yaw rate, lat vel and time) and one size (step)
{
    Vehicle ode(1);                                            //Call ODE function
    state_type x(2);
    x[0] = 0;                                                  //Lateral velocity value at t=0
    x[1] = 0;                                                  //Yaw rate at t=0

    vector<state_type> x_vec;                                  //X_vec initialisation (part of state_type vector which contains yaw rate and lat vel)
    vector<double> times;                                      //Time vector initilisation

    size_t steps = integrate(ode,                              //Steps calculation
    x, 0.0, 10.0, 0.1,
    push_back_state_and_time(x_vec, times));

    vector<double> lat_vel(steps + 1);                         //Transport vector for lateral velocity initialisation
    vector<double> yaw_rate(steps + 1);                        //Transport vector" for yaw rate initialisation

   for (size_t i = 0; i <= steps; i++)
    {
       lat_vel[i] = -x_vec[i][0];                              //Copy lateral velocity in transport vector
       yaw_rate[i] = x_vec[i][1];                              //Copy yaw rate in transport vector
    }
   return  std::make_tuple(lat_vel, yaw_rate, times, steps);   //Output (lateral velocity, yaw_rate, time and number of steps)
}
