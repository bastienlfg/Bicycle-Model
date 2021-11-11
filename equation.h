//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#ifndef EQUATION_H_
#define EQUATION_H_

#include <iostream>
#include <vector>

#include <boost/numeric/odeint.hpp>

const int param = 1;

typedef std::vector< double > state_type;		//State_type definition

class Vehicle {

int m_param;

public:

Vehicle(int) : m_param(param) { }

void operator() (const state_type& x, state_type& dxdt, const double t);

private:

//-------Car parameters--------

double Fyf;                           //Front tire lateral force init (N)
double Fyr;                           //Rear tire lateral force  init (N)
double af;                            //Front tire slip angle init    (rad)
double ar;                            //Rear tire slip angle init     (rad)
double t;                             //Time                          (s)

//-------Inputs parameters------

double delt;                          //Front steering angle init            (rad)
};
#endif
