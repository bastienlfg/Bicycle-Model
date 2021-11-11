//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#ifndef DEF_RESOLVING
#define DEF_RESOLVING

#include <vector>
#include <tuple>
#include <iostream>
#include "equation.h"
#include"mystruct.h"

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, size_t> resolution();    //Send three vectors (yaw rate, lat vel and time) and one size (step)

#endif
