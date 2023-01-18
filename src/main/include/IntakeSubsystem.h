#pragma once

#include <cmath> //Math library for C language
#include "PeriodicSubsystem.h"
#include "PID.h"
#include "ctre/Phoenix.h"
#include "RobotContext.h"
#include <algorithm>
#include "Ramp.h"
#include <mutex> 
#include <frc/Solenoid.h>
#include "Utility.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/DigitalInput.h>

class IntakeSubsystem: public PeriodicSubsystem
{
public:
    IntakeSubsystem() = delete;
    IntakeSubsystem(const IntakeSubsystem&) = delete;//Removes the copy constructor
    IntakeSubsystem(std::shared_ptr<RobotContext> context);
    virtual ~IntakeSubsystem(){};

    enum IntakeMode{STOP, IN, OUT};


    void setStop();
    void setFeeder(double s);
    void setIntake(double s);
    void toggleIntakeDeployed();
    void setIntakeDeployed(bool v);
    bool hasBall();

private:
    void disabledInit();
    void teleopInit();
    void autonInit();

    void disabledPeriodic();
    void teleopPeriodic();
    void autonPeiodic(); 
    
    std::shared_ptr<RobotContext> robot_context;
    TalonFX intake_motor{CANMap::FEEDER_MOTOR};
    TalonFX feeder_motor{CANMap::INTAKE_MOTOR};
    frc::Solenoid intake_solenoid{CANMap::PNEUMATIC_CONTROLLER, frc::PneumaticsModuleType::REVPH, SolenoidMap::INTAKE};
    frc::DigitalInput ball_linebreak{DigitalIOMap::FEEDER_BALL_LINEBREAK};


    //Ramps
    Ramp intake_ramp{32};
    Ramp feeder_ramp{32};

    std::mutex lock{};

    void doIntake();
    IntakeMode mode{STOP};  
    void stopMode();

    float intake_speed{0.0};
    float feeder_speed{0.0};
    bool intake_deployed{false};

};