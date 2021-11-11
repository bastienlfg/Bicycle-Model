//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#include "autoscale_graph.h"
using namespace std;

double kmin11;
double kmax11;
double kmin21;
double kmax21;

std::tuple<double,double,double,double>graph(double kmin1,double kmax1,double kmin2,double kmax2)
{
if (kmin1 > 0)
{kmin11 = kmin1 - 0.5 * kmin1;}       //Add (+/-)5% to the lowest value of yaw rate to adjust graph size
else                                  //depending on value sign
{kmin11 = kmin1 + 0.5 * kmin1;}

if ( kmax1 > 0)
{kmax11 = kmax1 + 0.5 * kmax1;}       //Add (+/-)5% to the highest value of yaw rate to adjust graph size
else                                  //depending on value sign
{kmax11 = kmax1 - 0.5 * kmax1;}

if (kmin2 > 0)
{kmin21 = kmin2 - 0.5 * kmin2;}       //Add (+/-)5% to the lowest value of lateral velocity to adjust graph size
else                                  //depending on value sign
{kmin21 = kmin2 + 0.5 * kmin2;}

if ( kmax2 > 0)
{kmax21 = kmax2 + 0.5 * kmax2;}       //Add (+/-)5% to the highest value of lateral velocity to adjust graph size
else                                  //depending on value sign
{kmax21 = kmax2 - 0.5 * kmax2;}

return std::make_tuple(kmin11,kmax11,kmin21,kmax21);  //Output (Graph 0 min scale,Graph 0 max scale,Graph 1 min scale,Graph 1 max scale)
}
