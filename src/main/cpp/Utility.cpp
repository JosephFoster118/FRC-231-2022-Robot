#include "Utility.h"

double deadZone(double input, double dz)
{
  return (std::abs(input) >= dz) ? input : 0.0; //?: Conditional operator
}

double getRotationOffset(double current, double target)
{
    double abs_current = current - (360.0*int(current/360));
    if(abs_current > 180)
    {
      abs_current -= 360;
    }
    else if (abs_current < -180)
    {
      abs_current += 360;
    }

    double abs_target = target - (360.0*int(target/360));
    if(abs_target > 180)
    {
      abs_target -= 360;
    }
    else if (abs_target < -180)
    {
      abs_target += 360;
    }
      
    double distance = abs_target - abs_current;
    if(distance > 180)
    {
      distance -= 360;
    }
    else if(distance < -180)
    {
      distance += 360;
    }
      
    return (distance);
}

double minMax(double value, double min, double max)
{
  return std::max(std::min(value, max), min);
}


/* (input >= dz) -> return boolean 
if bool is true -> return the left of the :
if bool is false -> return the right of the :
? can be use in Java and C++

Variable resistor: Potentia meter
*/