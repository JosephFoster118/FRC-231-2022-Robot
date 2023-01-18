#include <chrono>
#include "ClimberSubsystem.h"

using namespace std::chrono_literals;

ClimberSubsystem::ClimberSubsystem(std::shared_ptr<RobotContext> context): PeriodicSubsystem(20ms,"Drive Subsystem")
{
    robot_context = context;
    left_climber_offset = left_climb_motor.GetSelectedSensorPosition();
    right_climber_offset = right_climb_motor.GetSelectedSensorPosition();
}

void ClimberSubsystem::disabledInit()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    climber_power_left = 0.0;
    climber_power_right = 0.0;

    traversal_solenoid_state = false;
    
}

void ClimberSubsystem::teleopInit()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    climber_power_left = 0.0;
    climber_power_right = 0.0;
}

void ClimberSubsystem::autonInit()
{
    climber_power_left = 0.0;
    climber_power_right = 0.0;

}

void ClimberSubsystem::disabledPeriodic()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    doClimber();
}

void ClimberSubsystem::teleopPeriodic()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    doClimber();
}

void ClimberSubsystem::autonPeiodic()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    doClimber();
   
}

void ClimberSubsystem::setClimberPower(double power)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    climber_power_left = power;
    climber_power_right = power;

}

void ClimberSubsystem::doClimber()
{

    double current_left_position = left_climb_motor.GetSelectedSensorPosition() - left_climber_offset;
    double current_right_position = right_climb_motor.GetSelectedSensorPosition() - right_climber_offset;
    double left_climb_power = climber_power_left;
    double right_climb_power = -climber_power_right;
    if(!override_saftey)
    {
        if(current_left_position < 0.0 + CLIMBER_LIMIT)
        {
            left_climb_power = minMax(left_climb_power, 0.0, 1.0);
        }
        else if(current_left_position > CLIMBER_RANGE - CLIMBER_LIMIT)
        {
            left_climb_power = minMax(left_climb_power, -1.0, 0.0);
        }
        if(current_right_position < 0.0 + CLIMBER_LIMIT)
        {
            right_climb_power = minMax(right_climb_power, -1.0, 0.0);
        }
        else if(current_right_position > CLIMBER_RANGE - CLIMBER_LIMIT)
        {
            right_climb_power = minMax(right_climb_power, 0.0, 1.0);
        }
    }
    //left_climb_motor.Set(ControlMode::PercentOutput, left_climb_power);
    right_climb_motor.Set(ControlMode::PercentOutput, right_climb_power);
    traversal_solenoid.Set(traversal_solenoid_state);
    frc::SmartDashboard::PutNumber("Left Climber", current_left_position);
    frc::SmartDashboard::PutNumber("Right Climber", current_right_position);
}

void ClimberSubsystem::setTraversalDeployed(bool state)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    traversal_solenoid_state = state;
}

void ClimberSubsystem::setOverride(bool state)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    override_saftey = state;
}

void ClimberSubsystem::setLeftPower(double power)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    climber_power_left = power;
}
void ClimberSubsystem::setRightPower(double power)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    climber_power_right = power;    
}
