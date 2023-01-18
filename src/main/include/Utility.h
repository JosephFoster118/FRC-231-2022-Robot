#pragma once
#include <cmath>
#include <algorithm>

struct Coordinate
{
    double x; 
    double y;
};

double deadZone(double input, double dz);
double getRotationOffset(double current, double target);
double minMax(double value, double min, double max);


// Struck: Similar to class -> encapsulate data for everywhere to acesss, by default
// Struck: Can contain data, function (not recommened) (public); class (private) 
// Data structure: struct, class, union, emun 