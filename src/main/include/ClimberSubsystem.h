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

class ClimberSubsystem: public PeriodicSubsystem
{
public:
    ClimberSubsystem() = delete;
    ClimberSubsystem(const ClimberSubsystem&) = delete;//Removes the copy constructor
    ClimberSubsystem(std::shared_ptr<RobotContext> context);
    virtual ~ClimberSubsystem(){};

    void setClimberPower(double power);
    void setLeftPower(double power);
    void setRightPower(double power);
    void setTraversalDeployed(bool state);
    void setOverride(bool state);
    
    static constexpr double CLIMBER_RANGE{534685};
    static constexpr double CLIMBER_LIMIT{5000};

private:
    void disabledInit();
    void teleopInit();
    void autonInit();

    void disabledPeriodic();
    void teleopPeriodic();
    void autonPeiodic(); 

    void doClimber();

    TalonFX left_climb_motor{CANMap::LEFT_CLIMB_MOTOR};
    TalonFX right_climb_motor{CANMap::RIGHT_CLIMB_MOTOR};

    frc::Solenoid traversal_solenoid{CANMap::PNEUMATIC_CONTROLLER, frc::PneumaticsModuleType::REVPH, SolenoidMap::TRAVERSAL};

    std::mutex lock{};
    std::shared_ptr<RobotContext> robot_context;

    double climber_power_left{0.0};
    double climber_power_right{0.0};
    bool traversal_solenoid_state{false};
    bool override_saftey{false};
    double left_climber_offset{0.0};
    double right_climber_offset{0.0};
};