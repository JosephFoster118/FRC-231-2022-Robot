#include <chrono>
#include "IntakeSubsystem.h"

using namespace std::chrono_literals;

IntakeSubsystem::IntakeSubsystem(std::shared_ptr<RobotContext> context): PeriodicSubsystem(20ms,"Drive Subsystem")
{
    robot_context = context;
    frc::SmartDashboard::SetDefaultBoolean("Intake Deployed", false);
} 

void IntakeSubsystem::disabledInit()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_motor.Set(ControlMode::PercentOutput,0);
    feeder_motor.Set(ControlMode::PercentOutput,0);
    
    intake_solenoid.Set(false);
    intake_deployed = false;
    frc::SmartDashboard::PutBoolean("Intake Deployed", intake_deployed);
}

void IntakeSubsystem::teleopInit()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_ramp.reset();
    feeder_ramp.reset();
}

void IntakeSubsystem::autonInit()
{

}

void IntakeSubsystem::disabledPeriodic()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_motor.Set(ControlMode::PercentOutput,0);
    feeder_motor.Set(ControlMode::PercentOutput,0);
}

void IntakeSubsystem::teleopPeriodic()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_motor.Set(ControlMode::PercentOutput,minMax(-intake_speed, -1.0, 1.0));
    feeder_motor.Set(ControlMode::PercentOutput, feeder_ramp.getSpeed(minMax(feeder_speed, -1.0, 1.0)));
    intake_solenoid.Set(intake_deployed);
}

void IntakeSubsystem::autonPeiodic()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_motor.Set(ControlMode::PercentOutput, intake_ramp.getSpeed(minMax(intake_speed, -1.0, 1.0)));
    feeder_motor.Set(ControlMode::PercentOutput, feeder_ramp.getSpeed(minMax(feeder_speed, -1.0, 1.0)));
    intake_solenoid.Set(intake_deployed);
}


void IntakeSubsystem::stopMode()
{
    intake_motor.Set(ControlMode::PercentOutput,0);
    feeder_motor.Set(ControlMode::PercentOutput,0);
}


void IntakeSubsystem::setStop()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_speed = 0.0;
    feeder_speed = 0.0;
}

void IntakeSubsystem::setFeeder(double s)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    feeder_speed = s;
}

void IntakeSubsystem::setIntake(double s)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_speed = s;
}

void IntakeSubsystem::toggleIntakeDeployed()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_deployed = !intake_deployed;
    frc::SmartDashboard::PutBoolean("Intake Deployed", intake_deployed);
}
void IntakeSubsystem::setIntakeDeployed(bool v)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    intake_deployed = v;
    frc::SmartDashboard::PutBoolean("Intake Deployed", intake_deployed);
}

bool IntakeSubsystem::hasBall()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    //return ball_linebreak.Get();
    return false;//todo: revert

}