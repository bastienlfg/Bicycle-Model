//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "resolving.h"
#include "equation.h"
#include "integ_meth.h"

using namespace std;
using namespace boost::numeric::odeint;
const double dt = 0.01;

void integ::Euler(vector<double> x)
{
    Vehicle ode(1);
    size_t steps = integrate(ode,                               //Step size initialisation
        x, 0.0, 10.0, 0.1);

    euler< state_type > stepper;                                //Integration method

    integrate_const(stepper, ode, x, 0.0, 10.0, 0.01);

    for (double t = 0.0; t < 10.0; t += dt)
        stepper.do_step(ode, x, t, dt);
}

void integ::RK4(vector<double> x)
{
    Vehicle ode(1);
    size_t steps = integrate(ode,                               //Step size initialisation
        x, 0.0, 10.0, 0.1);

    runge_kutta4< state_type > stepper;                         //Integration method

    integrate_const(stepper, ode, x, 0.0, 10.0, 0.01);

    for (double t = 0.0; t < 10.0; t += dt)
        stepper.do_step(ode, x, t, dt);
}
