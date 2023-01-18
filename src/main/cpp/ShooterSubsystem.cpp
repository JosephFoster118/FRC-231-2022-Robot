#include <chrono>
#include "ShooterSubsystem.h"

using namespace std::chrono_literals;

ShooterSubsystem::ShooterSubsystem(std::shared_ptr<RobotContext> context): PeriodicSubsystem(20ms,"Drive Subsystem")
{
    robot_context = context;
    turret_offset = turret_motor.GetSelectedSensorPosition() - (TURRET_RANGE / 2);

} 
void ShooterSubsystem::populateDashboard()
{
    frc::SmartDashboard::PutNumber("turret position", turret_motor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutBoolean("Turret Switch", turret_limit_switch.Get());
    frc::SmartDashboard::PutNumber("turret start", turret_start_position);
	frc::SmartDashboard::PutNumber("turret end", turret_end_position);
    frc::SmartDashboard::PutNumber("Turret Absolute Angle", getTurretAbsoluteAngle());
    frc::SmartDashboard::PutNumber("Turret Relative Angle", getTurretRelativeAngle());
    frc::SmartDashboard::PutString("Turret Mode", turretModeToString(turret_mode));
    frc::SmartDashboard::PutBoolean("Turret Calibrated", isTurretCalibrated());
}

void ShooterSubsystem::disabledInit()
{

    shooter_motor_left.Set(ControlMode::PercentOutput, 0);
    shooter_motor_right.Set(ControlMode::PercentOutput, 0);
    backspin_motor.Set(ControlMode::PercentOutput, 0);
    keep_shooter_running = false;
    turret_motor.Set(ControlMode::PercentOutput, 0);
    turret_speed = 0;
    setStop();


}

void ShooterSubsystem::teleopInit()
{
    if(!isTurretCalibrated()) 
    {
        setTurretOffset();
    }
    
    shooter_ramp.reset();
    turret_mode = MANUAL;
}

void ShooterSubsystem::autonInit()
{
    if(!isTurretCalibrated()) 
    {
        setTurretOffset();
    }
}

void ShooterSubsystem::setTurretOffset()
{
    turret_start_position = turret_motor.GetSelectedSensorPosition();
    turret_end_position = turret_start_position + TURRET_RANGE;
    turret_offset = (turret_start_position + turret_end_position) / 2;
}

void ShooterSubsystem::disabledPeriodic()
{
    //doShooter();
    //doTurret();
    calibrateTurret();
    populateDashboard();
    
}


void ShooterSubsystem::teleopPeriodic()
{
    doShooter();
    doTurret();
   populateDashboard();
   //double back_speed = frc::SmartDashboard::GetNumber("BackSpeed", 0.0);
   //backspin_motor.Set(ControlMode::PercentOutput, back_speed);

}

void ShooterSubsystem::autonPeiodic()
{
    doShooter();
    doTurret();
    populateDashboard();
}

void ShooterSubsystem::doShooter()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    switch(mode)
    {
        case STOP:
        {
            stopMode();   
        }break;
        case SHOOT:
        {
            shootMode();
        }break;
    }
    frc::SmartDashboard::PutBoolean("Keep Intake Running", keep_shooter_running);
   
}

void ShooterSubsystem::stopMode()
{
    shooter_motor_left.Set(ControlMode::PercentOutput,0);
    shooter_motor_right.Set(ControlMode::PercentOutput,0);
    double current_velocity = shooter_motor_left.GetSelectedSensorVelocity();
    backspin_motor.Set(ControlMode::PercentOutput, 0.0);
    frc::SmartDashboard::PutNumber("speed target", speed);
    frc::SmartDashboard::PutNumber("actual speed", current_velocity);
    frc::SmartDashboard::UpdateValues();    
}


void ShooterSubsystem::setStop()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    mode = STOP;
}

void ShooterSubsystem::shootMode()
{
    double new_target_speed = speed;
    double current_velocity = shooter_motor_left.GetSelectedSensorVelocity();
    if(speed < 500.0 && keep_shooter_running)
    {
        new_target_speed = 500.0;
    }
    if(new_target_speed > 50.0 )
    {
        double new_speed;
        if(new_target_speed > 8000.0)
        {
            new_speed = shooter_pid_high.calculatePID(current_velocity, new_target_speed);
        }
        else
        {
            new_speed = shooter_pid_low.calculatePID(current_velocity, new_target_speed);
        }
        
        
        new_speed = minMax(new_speed, -1.0, 1.0);
        shooter_motor_left.Set(ControlMode::PercentOutput, new_speed);
        shooter_motor_right.Set(ControlMode::PercentOutput, -new_speed);
    }
    else
    {
        shooter_motor_left.Set(ControlMode::PercentOutput, 0.0);
        shooter_motor_right.Set(ControlMode::PercentOutput, 0.0);
    }
    backspin_motor.Set(ControlMode::PercentOutput, backspin_speed);
    frc::SmartDashboard::PutNumber("speed target", speed);
    frc::SmartDashboard::PutNumber("actual speed", current_velocity);
    frc::SmartDashboard::PutNumber("LSV", shooter_motor_left.GetMotorOutputPercent());
	frc::SmartDashboard::PutNumber("RSV", shooter_motor_right.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("I", shooter_pid_high.getErrror());
    frc::SmartDashboard::UpdateValues();
}

void ShooterSubsystem::setShootSpeed(double s)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    if (speed != s)
    {
        shooter_pid_high.resetError();
        shooter_pid_low.resetError();
    }
    mode = SHOOT;
    speed = s;
}
void ShooterSubsystem::changeSpeedBy(double s)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    speed += s;
    if(speed < 0)
    {
        speed = 0;
    }
}

double ShooterSubsystem::getSpeed()
{
    return shooter_motor_left.GetSelectedSensorVelocity();

}

void ShooterSubsystem::toggleKeepShooterRunning()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    keep_shooter_running = !keep_shooter_running;
}

void ShooterSubsystem::setTurretSpeed(double s)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    turret_speed = s;
}

void ShooterSubsystem::setTurretMode(TurretMode m)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    if(m == AUTO)
    {
        turret_pid.resetError();
        robot_context->limelight->PutNumber("camMode", 0);//Auto
    }
    else
    {
        robot_context->limelight->PutNumber("camMode", 1);//Driver
    }
    turret_mode = m;
}

void ShooterSubsystem::doTurret()
{
    double power{0.0};
    double turret_position = turret_motor.GetSelectedSensorPosition();
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    if(turret_mode == MANUAL)
    {
        power = turret_speed * TURRET_SPEED_FACTOR;
    }
    else if(turret_mode == AUTO)
    {
        double target_offset = robot_context->limelight->GetNumber("tx",0.0);
        power = turret_pid.calculatePID(-target_offset, 0.0);
        power = minMax(power, -1.0, 1.0);

    }
    else if(turret_mode == SET)
    {
        double current_angle = getTurretRelativeAngle();
        double remaining = getRotationOffset(current_angle, turret_target);
        power = turret_set_pid.calculatePID(-remaining, 0.0);
        power = minMax(power, -1.0, 1.0);
        frc::SmartDashboard::PutNumber("turret position", current_angle);
    }
    else if(turret_mode == FIXED)
    {
        double current_angle = getTurretAbsoluteAngle();
        double remaining = getRotationOffset(current_angle, turret_fixed_target);
        power = turret_fixed_pid.calculatePID(-remaining, 0.0);
        power = minMax(power, -1.0, 1.0);
    }
    if(turret_position < turret_start_position + turret_limit_soft)
    {
        if(turret_position < turret_start_position + turret_limit)
        {
            power = minMax(power, 0.3, 1.0);
        }
        else
        {
            power = minMax(power, 0.0, 1.0);
        }
    }
    else if(turret_position > turret_end_position - turret_limit_soft)
    {
        if(turret_position > turret_end_position - turret_limit)
        {
            power = minMax(power, -1.0, -0.3);
        }
        else
        {
            power = minMax(power, -1.0, 0.0);
        }
    }
    
    turret_motor.Set(ControlMode::PercentOutput, power);

}

bool ShooterSubsystem::shooterAtSpeed()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    double current_velocity = shooter_motor_left.GetSelectedSensorVelocity();
    return std::abs(current_velocity - speed) < speed_tolerance;
}

bool ShooterSubsystem::aimingAtTarget()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    double target_offset = robot_context->limelight->GetNumber("tx",0.0);
    return std::abs(target_offset) < AIM_TOLERANCE;
}

void ShooterSubsystem::setTurretPosition(double p)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    turret_mode = SET;
    turret_target = p;
}

void ShooterSubsystem::setTurretFixedAngle(double a)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    if(turret_mode != FIXED)
    {
        turret_fixed_pid.resetError();
    }
    turret_mode = FIXED;
    turret_fixed_target = a;
}

void ShooterSubsystem::calibrateTurret()
{
    //turret_limit_switch_debounce.setValue(turret_limit_switch.Get());
    turret_limit_switch_debounce.setValue(robot_context->pilot_controller.GetBackButton());
    if(turret_limit_switch_debounce.getValue())
    {
        setTurretOffset();
    }
}

double ShooterSubsystem::getTurretAbsoluteAngle()
{
    double turret_position = turret_motor.GetSelectedSensorPosition();
    double ticks_from_center = -(turret_offset - turret_position);
    double degrees_from_center = ticks_from_center / TURRET_TICKS_PER_DEGREE;
    double absolute_turret_angle = degrees_from_center + robot_context->robot_start_angle - robot_context->robot_angle_offset + robot_context->navx.GetYaw();
    return absolute_turret_angle;
}

double ShooterSubsystem::getTurretRelativeAngle()
{
    double turret_position = turret_motor.GetSelectedSensorPosition();
    double ticks_from_center = -(turret_offset - turret_position);
    return ticks_from_center / TURRET_TICKS_PER_DEGREE;
}

ShooterSubsystem::TurretMode ShooterSubsystem::getTurretMode()
{
    return turret_mode;
}

std::string ShooterSubsystem::turretModeToString(ShooterSubsystem::TurretMode m)
{
    switch (m)
    {
        case ShooterSubsystem::AUTO:
        {
            return "Auto";
        }
        case ShooterSubsystem::MANUAL:
        {
            return "Manual";
        }
        case ShooterSubsystem::FIXED:
        {
            return "Fixed";
        }
        case ShooterSubsystem::SET:
        {
            return "Set";
        }
        default:
        {
            return "Unknown";
        }
    }
}

bool ShooterSubsystem::isTurretCalibrated()
{
    return turret_start_position != turret_end_position;
}

void ShooterSubsystem::setBackspinSpeed(double s)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    backspin_speed = s;
}