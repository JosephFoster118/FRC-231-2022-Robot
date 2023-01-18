#pragma once 
#include <frc/BuiltInAccelerometer.h>
#include <chrono>
#include "PeriodicThread.h" 
#include <memory>
#include <iostream>
#include <iomanip>


class Position 
{
public: 
    Position ();
    void readAcceleration(); 
    void start();
private:
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    frc::BuiltInAccelerometer accelerometer{}; //Curly bracket is the new standard to program
    std::unique_ptr<PeriodicThread> thread; 
    
    double velocity[3]{0,};
    TimePoint last_time{};
    bool calibrating{true};
    static const int CALIBRATION_SIZE{50};
    int calibrating_count{0};
    double acceleration_drift[3]{0,};
};


 
/* Notes:
A class can't point to itself until it had been innitialized
Inherit: class going to include thread
Lamda: declare a variable as a function
Static: one value accross every instant of the class, const: constant
*/