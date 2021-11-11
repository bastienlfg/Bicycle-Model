//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#ifndef DEF_INTEG_METH
#define DEF_INTEG_METH

#include <vector>
#include <tuple>
#include <iostream>
#include "string"
#include "equation.h"

class integ

{
public:

    void Euler(std::vector<double> x);
    void RK4(std::vector<double> x);
};

#endif
