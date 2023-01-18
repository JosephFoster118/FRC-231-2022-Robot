#pragma once
#include <chrono>

class Ramp
{
public:
    Ramp() = delete;
    Ramp(const Ramp&) = delete; //Tell the compiler we do not want the copy_constructor 
    Ramp(double m_a);
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    double getSpeed(double target);
    void reset();

private:

    double max_acceleration; //how fast we are going to acce in 1sec
    TimePoint last_time{};
    double current_value{0};


};