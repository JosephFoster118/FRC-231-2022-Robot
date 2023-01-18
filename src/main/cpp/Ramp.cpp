#include "Ramp.h"

Ramp::Ramp(double m_a)
{
    max_acceleration = m_a;
    last_time = std::chrono::high_resolution_clock::now();

}

double Ramp::getSpeed(double target)
{
    auto current_time = std::chrono::high_resolution_clock::now();
    if(current_value != target)
    {
        double time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
        if(current_value < target)
        {
            current_value += (time_passed/1000) * max_acceleration;
            if(current_value > target)
            {
                current_value = target; //overshoot -> set it to target
            }
        }
        else
        {
            current_value -= (time_passed/1000) * max_acceleration; 
            if(current_value < target)
            {
                current_value = target; //overshoot -> set it to target
            }
        }
        
    }
    last_time = current_time;
    return current_value;
}

void Ramp::reset()
{
    last_time = std::chrono::high_resolution_clock::now(); //reset the time 
}
